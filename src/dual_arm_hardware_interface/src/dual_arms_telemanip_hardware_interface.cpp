#include "dual_arm_hardware_interface/dual_arms_telemanip_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dual_arm_hardware_interface
{

namespace {
constexpr const char* kDevLeft  = "left";
constexpr const char* kDevHead  = "head";
constexpr const char* kDevRight = "right";
}  // namespace

// ============================================================
// on_init_impl
//
// The base class has already:
//   - called hardware_interface::SystemInterface::on_init(info)
//   - stored info into info_
//   - created logger_ = rclcpp::get_logger(get_name() + "_hardware_interface")
// so we start straight from parsing parameters.
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_init_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_init: joints=%zu", get_name().c_str(), info_.joints.size());

  const size_t n = info_.joints.size();

  // ---- Shared buffers + layout ----
  bufs_.allocate(n);
  layout_.build(info_);
  layout_.log(logger_, get_name().c_str());

  // ---- URDF parameters ----
  HardwareParams params(info_);

  if (!params.has("cell_id")) 
  {
    RCLCPP_ERROR(logger_, "[%s] missing required parameter 'cell_id'", get_name().c_str());
    return CallbackReturn::ERROR;
  }
  cell_id_ = params.get_string("cell_id");

  period_ms_     = params.get_u64("period_ms",     10);
  agent_wait_ms_ = params.get_u64("agent_wait_ms", 10000);

  // ---- zenoh.json5 path (optional: derive from package share if not given) ----
  if (params.has("zenoh_config")) 
  {
    zenoh_config_path_ = params.get_string("zenoh_config");
  } 
  else 
  {
    try 
    {
      const std::string share = ament_index_cpp::get_package_share_directory("dual_arm_hardware_interface");
      zenoh_config_path_ = share + "/config/zenoh.json5";
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(logger_,
        "[%s] cannot resolve zenoh.json5: %s "
        "(set the 'zenoh_config' hardware parameter)",
        get_name().c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_,
    "[%s] cell_id='%s' period_ms=%lu agent_wait_ms=%lu zenoh='%s'",
    get_name().c_str(), cell_id_.c_str(),
    static_cast<unsigned long>(period_ms_),
    static_cast<unsigned long>(agent_wait_ms_),
    zenoh_config_path_.c_str());

  // ---- Trajectory parameters (initial values from URDF) ----
  trajectory_.load(params);

  RCLCPP_INFO(logger_, "[%s] initial trajectory_mode=%u, radio=%u",
    get_name().c_str(),
    static_cast<unsigned>(trajectory_.mode.load()),
    static_cast<unsigned>(trajectory_.radio.load()));

  // ---- Joint limiter parameters (optional, default disabled = 0) ----
  const double max_vel_deg_s  = params.get_double("max_joint_velocity_deg_s",     0.0);
  const double max_acc_deg_s2 = params.get_double("max_joint_acceleration_deg_s2", 0.0);

  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  max_joint_vel_rad_s_  = max_vel_deg_s  * kDegToRad;
  max_joint_acc_rad_s2_ = max_acc_deg_s2 * kDegToRad;

  joint_limiters_.resize(n);
  for (auto& jl : joint_limiters_) 
  {
    jl.max_vel     = max_joint_vel_rad_s_;
    jl.max_acc     = max_joint_acc_rad_s2_;
    jl.initialized = false;
  }

  RCLCPP_INFO(logger_,
      "[%s] joint limiter: max_vel=%.1f deg/s (%.4f rad/s), "
      "max_acc=%.1f deg/s^2 (%.4f rad/s^2)",
      get_name().c_str(), max_vel_deg_s, max_joint_vel_rad_s_,
      max_acc_deg_s2, max_joint_acc_rad_s2_);

  // ---- Command buffers ----
  cmdbuf_.allocate(layout_.left_indices_.size(),
                   layout_.head_indices_.size(),
                   layout_.right_indices_.size());

  // ---- Telemanip session ----
  std::vector<std::string> device_ids;
  if (!layout_.left_indices_ .empty())
    device_ids.emplace_back(kDevLeft);
  if (!layout_.head_indices_ .empty()) 
    device_ids.emplace_back(kDevHead);
  if (!layout_.right_indices_.empty()) 
    device_ids.emplace_back(kDevRight);

  if (device_ids.empty()) 
  {
    RCLCPP_ERROR(logger_, "[%s] no joints declared in URDF", get_name().c_str());
    return CallbackReturn::ERROR;
  }

  try 
  {
    session_ = std::make_unique<TelemanipSession>(
        logger_, cell_id_, device_ids, zenoh_config_path_,
        period_ms_, agent_wait_ms_);
  } 
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR(logger_, "[%s] TelemanipSession construction failed: %s",
                 get_name().c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "[%s] TelemanipSession created", get_name().c_str());

  // ---- Embedded helpers: CSV logger + tuner node ----
  csv_ = std::make_unique<CsvLogger>(logger_, get_name());
  csv_->start();

  tuner_ = std::make_unique<TrajectoryTuner>(
      logger_, get_name(),
      /*on_tuner=*/ [this](uint8_t m, uint16_t r) {
        trajectory_.set(m, r);
        RCLCPP_INFO(logger_, "[%s] trajectory_tuner -> mode=%u, radio=%u",
            get_name().c_str(),
            static_cast<unsigned>(m), static_cast<unsigned>(r));
      },
      /*on_csv=*/ [this](bool on) { csv_->request(on); });
  tuner_->start();

  RCLCPP_INFO(logger_, "[%s] on_init done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure_impl — wait for first state and seed command buffers
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_configure_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_configure", get_name().c_str());

  if (!session_ || !session_->valid()) 
  {
    RCLCPP_ERROR(logger_, "[%s] session_ not available", get_name().c_str());
    return CallbackReturn::ERROR;
  }

  if (!wait_and_seed_all_groups(2000)) 
  {
    RCLCPP_ERROR(logger_, "[%s] on_configure: no initial state",
                 get_name().c_str());
    return CallbackReturn::ERROR;
  }

  reset_joint_limiters();

  RCLCPP_INFO(logger_, "[%s] on_configure done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate_impl — enable all groups and wait for state
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_activate_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_activate", get_name().c_str());

  if (!session_ || !session_->valid()) 
  {
    RCLCPP_ERROR(logger_, "[%s] session_ not available", get_name().c_str());
    return CallbackReturn::ERROR;
  }

  // 1. Let the zenoh channel settle (avoid first frames dropped after activate)
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 2. Re-read current positions: after deactivate → activate the robot may
  //    have moved, so seed again with the live values.
  if (!wait_and_seed_all_groups(2000)) 
  {
    RCLCPP_ERROR(logger_,
      "[%s] on_activate: failed to read current state",
      get_name().c_str());
    return CallbackReturn::ERROR;
  }

  // 3. Reset limiters so the next limit() starts from the seeded position.
  reset_joint_limiters();

  // 4. Enable groups: left → right → head
  if (!layout_.left_indices_.empty()) 
  {
    if (!session_->group_enable(kDevLeft, true)) 
    {
      RCLCPP_ERROR(logger_, "[%s] group_enable(left, true) failed",
                   get_name().c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "[%s] group_enable(left, true) OK", get_name().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!layout_.right_indices_.empty()) 
  {
    if (!session_->group_enable(kDevRight, true)) 
    {
      RCLCPP_ERROR(logger_, "[%s] group_enable(right, true) failed",
                   get_name().c_str());
      session_->group_enable(kDevLeft, false);   // roll back
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "[%s] group_enable(right, true) OK", get_name().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!layout_.head_indices_.empty()) 
  {
    if (!session_->group_enable(kDevHead, true))
    {
      RCLCPP_WARN(logger_, "[%s] group_enable(head, true) failed (continuing)",
                  get_name().c_str());
    } 
    else 
    {
      RCLCPP_INFO(logger_, "[%s] group_enable(head, true) OK", get_name().c_str());
    }
  }

  is_activated_ = true;
  RCLCPP_INFO(logger_, "[%s] on_activate done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate_impl — disable all groups
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_deactivate_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_deactivate", get_name().c_str());

  if (session_ && session_->valid()) 
  {
    if (!layout_.left_indices_.empty() && !session_->group_enable(kDevLeft, false)) 
    {
      RCLCPP_WARN(logger_,
        "[%s] group_enable(left, false) rejected locally (expected)",
        get_name().c_str());
    }
    if (!layout_.right_indices_.empty() && !session_->group_enable(kDevRight, false)) 
    {
      RCLCPP_DEBUG(logger_,
        "[%s] group_enable(right, false) rejected locally (expected)",
        get_name().c_str());
    }
    if (!layout_.head_indices_.empty()) 
    {
      session_->group_enable(kDevHead, false);   // best-effort
    }
  }

  is_activated_ = false;
  RCLCPP_INFO(logger_, "[%s] on_deactivate done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_cleanup_impl
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_cleanup_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_cleanup", get_name().c_str());

  if (csv_)   
    csv_->stop();
  if (tuner_) 
    tuner_->stop();

  csv_.reset();
  tuner_.reset();

  session_.reset();
  is_activated_ = false;

  RCLCPP_INFO(logger_, "[%s] on_cleanup done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_shutdown_impl
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_shutdown_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_shutdown", get_name().c_str());

  if (csv_)   
    csv_->stop();
  if (tuner_) 
    tuner_->stop();

  csv_.reset();
  tuner_.reset();

  session_.reset();
  is_activated_ = false;

  RCLCPP_INFO(logger_, "[%s] on_shutdown done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface>
DualArmsTelemanipHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(info_.joints.size() * 3);

  for (const auto& joint : info_.joints) 
  {
    auto it = layout_.joint_indices_.find(joint.name);
    if (it == layout_.joint_indices_.end()) 
      continue;
    const size_t idx = it->second;

    for (const auto& s : joint.state_interfaces) 
    {
      if      (s.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                         &bufs_.hw_position_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_VELOCITY)
        out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                         &bufs_.hw_velocity_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_EFFORT)
        out.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT,
                         &bufs_.hw_effort_states_[idx]);
    }
  }
  return out;
}

// ============================================================
// export_command_interfaces
// ============================================================
std::vector<hardware_interface::CommandInterface>
DualArmsTelemanipHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(info_.joints.size());

  for (const auto& joint : info_.joints) 
  {
    auto it = layout_.joint_indices_.find(joint.name);
    if (it == layout_.joint_indices_.end()) 
      continue;
    const size_t idx = it->second;

    for (const auto& c : joint.command_interfaces) 
    {
      if (c.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                         &bufs_.hw_position_commands_[idx]);
    }
  }
  return out;
}

// ============================================================
// read
// ============================================================
hardware_interface::return_type DualArmsTelemanipHardwareInterface::read(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!session_ || !session_->valid()) 
  {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Zero velocities first, in case the SDK has no fresh data this tick
  for (size_t i = 0; i < bufs_.hw_velocity_states_.size(); ++i)
    bufs_.hw_velocity_states_[i] = 0.0;

  auto update_group = [&](const std::string& dev, const std::vector<size_t>& idx)
  {
    if (idx.empty()) 
      return;

    auto st = session_->read_state(dev);
    if (!st) 
      return;   // no snapshot → keep previous values

    if (st->joints && st->joints_len == idx.size()) 
    {
      for (size_t k = 0; k < idx.size(); ++k)
        bufs_.hw_position_states_[idx[k]] = st->joints[k];
    }

    if (st->velocity && st->velocity_len == idx.size()) 
    {
      for (size_t k = 0; k < idx.size(); ++k)
        bufs_.hw_velocity_states_[idx[k]] = st->velocity[k];
    }
  };

  update_group(kDevLeft,  layout_.left_indices_);
  update_group(kDevRight, layout_.right_indices_);
  update_group(kDevHead,  layout_.head_indices_);

  // Effort is always 0 (SDK does not provide it)
  for (size_t i = 0; i < bufs_.hw_effort_states_.size(); ++i)
    bufs_.hw_effort_states_[i] = 0.0;

  // ---- Debug print (throttled) ----
  if (++debug_read_tick_ >= debug_every_n_) 
  {
    debug_read_tick_ = 0;
    auto to_str_by_indices = [&](const std::vector<size_t>& idx) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4) << "[";
      for (size_t k = 0; k < idx.size(); ++k) 
      {
        oss << bufs_.hw_position_states_[idx[k]];
        if (k + 1 < idx.size()) 
          oss << ", ";
      }
      oss << "]";
      return oss.str();
    };
    RCLCPP_INFO(logger_, "[%s] read:\n  left:  %s\n  head:  %s\n  right: %s",
                get_name().c_str(),
                to_str_by_indices(layout_.left_indices_ ).c_str(),
                to_str_by_indices(layout_.head_indices_ ).c_str(),
                to_str_by_indices(layout_.right_indices_).c_str());
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write
// ============================================================
hardware_interface::return_type DualArmsTelemanipHardwareInterface::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& period)
{
  if (!session_ || !session_->valid() || !is_active()) 
  {
    return hardware_interface::return_type::OK;
  }

  // ---- dt measurement ----
  static thread_local std::chrono::steady_clock::time_point prev_entry{};
  const auto t_entry = std::chrono::steady_clock::now();

  double actual_dt_ms = 0.0;
  if (prev_entry.time_since_epoch().count() != 0) 
  {
    actual_dt_ms = std::chrono::duration<double, std::milli>(t_entry - prev_entry).count();
  }
  prev_entry = t_entry;

  // ============================================================
  // 1. Snapshot commands under the mutex
  // ============================================================
  cmdbuf_.gather(layout_, bufs_, mutex_);

  const uint8_t  mode  = trajectory_.mode.load();
  const uint16_t radio = trajectory_.radio.load();

  // ============================================================
  // 2. Apply joint limiter (velocity + acceleration)
  // ============================================================
  const double dt_s = period.seconds();
  if (dt_s > 0.0 && max_joint_vel_rad_s_ > 0.0) 
  {
    cmdbuf_.apply_limiters(joint_limiters_, layout_, dt_s);
  }

  cmdbuf_.pack();

  // ============================================================
  // 3. Stage joint commands (never blocks, never sends)
  // ============================================================
  const auto t0 = std::chrono::steady_clock::now();

  bool stage_ok = true;
  if (!layout_.left_indices_.empty())
    stage_ok &= session_->stage_joint(kDevLeft,
                                      cmdbuf_.left_cmd_buf_.data(),
                                      cmdbuf_.left_cmd_buf_.size(),
                                      true, mode, radio);
  if (!layout_.right_indices_.empty())
    stage_ok &= session_->stage_joint(kDevRight,
                                      cmdbuf_.right_cmd_buf_.data(),
                                      cmdbuf_.right_cmd_buf_.size(),
                                      true, mode, radio);
  // if (!layout_.head_indices_.empty())
  //   stage_ok &= session_->stage_joint(kDevHead,
  //                                     cmdbuf_.head_cmd_buf_.data(),
  //                                     cmdbuf_.head_cmd_buf_.size(),
  //                                     true, mode, radio);

  const auto t1 = std::chrono::steady_clock::now();

  if (!stage_ok) 
  {
    ++error_streak_;
    if (error_streak_ == 1 || error_streak_ % 100 == 0) 
    {
      RCLCPP_WARN(logger_, "[%s] stage_joint failed (streak=%d)",
                  get_name().c_str(), error_streak_);
    }
  } 
  else 
  {
    error_streak_ = 0;
  }

  // ============================================================
  // 4. Timing publish + rolling stats
  // ============================================================
  const double call_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double period_ms = period.seconds() * 1000.0;
  const double threshold_ms = period_ms * 1.5;

  if (tuner_) 
    tuner_->publish_timing(call_ms, period_ms, threshold_ms, actual_dt_ms);

  {
    double dm, ds, dmin, dmax, cm, cmax;
    size_t n_samples = 0;
    if (timing_stats_.update(actual_dt_ms, call_ms, period_ms, dm, ds, dmin, dmax, cm, cmax, n_samples))
    {
      RCLCPP_WARN(logger_,
          "[%s] write dt stats (N=%zu): "
          "mean=%.2f sd=%.2f min=%.2f max=%.2f ms (period=%.2f) | "
          "call mean=%.2f max=%.2f ms",
          get_name().c_str(), n_samples,
          dm, ds, dmin, dmax, period_ms, cm, cmax);
    }
  }

  // ============================================================
  // 5. Debug print of what we staged (throttled)
  // ============================================================
  if (++debug_write_tick_ >= debug_every_n_) 
  {
    debug_write_tick_ = 0;

    char buf_l[256], buf_h[256], buf_r[256];
    auto write_arr = [](char* buf, size_t buflen, const std::vector<float>& v) {
      size_t pos = 0;
      pos += std::snprintf(buf + pos, buflen - pos, "[");
      for (size_t k = 0; k < v.size() && pos < buflen - 16; ++k)
        pos += std::snprintf(buf + pos, buflen - pos, "%s%.4f",
                             (k ? ", " : ""), v[k]);
      std::snprintf(buf + pos, buflen - pos, "]");
    };
    write_arr(buf_l, sizeof(buf_l), cmdbuf_.left_cmd_buf_);
    write_arr(buf_h, sizeof(buf_h), cmdbuf_.head_cmd_buf_);
    write_arr(buf_r, sizeof(buf_r), cmdbuf_.right_cmd_buf_);

    RCLCPP_INFO(logger_,
        "[%s] write:\n  left:  %s\n  head:  %s\n  right: %s mode: %u radio: %u",
        get_name().c_str(), buf_l, buf_h, buf_r,
        static_cast<unsigned>(mode), static_cast<unsigned>(radio));

    // Limiter statistics
    uint64_t total_clamps = 0;
    size_t   worst_joint  = 0;
    uint64_t worst_count  = 0;
    for (size_t i = 0; i < joint_limiters_.size(); ++i) 
    {
      total_clamps += joint_limiters_[i].clamp_count;
      if (joint_limiters_[i].clamp_count > worst_count) 
      {
        worst_count = joint_limiters_[i].clamp_count;
        worst_joint = i;
      }
    }
    RCLCPP_INFO(logger_,
        "[%s] limiter stats: total_clamps=%lu, worst=%s (%lu)",
        get_name().c_str(),
        static_cast<unsigned long>(total_clamps),
        info_.joints[worst_joint].name.c_str(),
        static_cast<unsigned long>(worst_count));
  }

  // ============================================================
  // 6. Delta publisher (throttled subscription check)
  // ============================================================
  if (tuner_) 
    tuner_->publish_delta(cmdbuf_.cmd_buffer_, cmdbuf_.last_cmd_buffer_);

  // ============================================================
  // 7. CSV push (lock-free, may drop on overflow)
  // ============================================================
  if (csv_ && csv_->is_open()) 
  {
    csv_->push(make_csv_row(
      cmdbuf_.cmd_buffer_, layout_,
      std::chrono::duration<double>(t1.time_since_epoch()).count(),
      /*follow=*/1, mode, radio));
  }

  // ============================================================
  // 8. Swap cmd_buffer_ <-> last_cmd_buffer_ (zero-copy)
  // ============================================================
  cmdbuf_.swap();
  return hardware_interface::return_type::OK;
}

// ============================================================
// wait_and_seed_all_groups
// ============================================================
bool DualArmsTelemanipHardwareInterface::wait_and_seed_all_groups(int timeout_ms)
{
  if (!session_ || !session_->valid()) 
  {
    RCLCPP_ERROR(logger_, "[%s] wait_and_seed: session_ not available", get_name().c_str());
    return false;
  }

  constexpr int    kPollMs  = 20;
  constexpr double kZeroEps = 1e-6;

  std::vector<float> left_now, head_now, right_now;

  // ---- helper: poll a joint group with timeout ----
  auto read_arm = [&](const char* dev, std::vector<float>& out, size_t n) -> bool {
    out.assign(n, 0.0F);
    bool present = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) 
    {
      if (session_->read_joint(dev, out.data(), out.size(), &present) && present)
        return true;

      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
  };

  // ---- left arm ----
  if (!layout_.left_indices_.empty() && !read_arm(kDevLeft, left_now, layout_.left_indices_.size()))
  {
    RCLCPP_WARN(logger_, "[%s] no LEFT state within %d ms", get_name().c_str(), timeout_ms);
    return false;
  }

  // ---- right arm ----
  if (!layout_.right_indices_.empty() && !read_arm(kDevRight, right_now, layout_.right_indices_.size()))
  {
    RCLCPP_WARN(logger_, "[%s] no RIGHT state within %d ms", get_name().c_str(), timeout_ms);
    return false;
  }

  // ---- head (2 joints, must use read_state) ----
  if (!layout_.head_indices_.empty()) 
  {
    head_now.assign(layout_.head_indices_.size(), 0.0F);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    bool ok = false;
    while (std::chrono::steady_clock::now() < deadline) 
    {
      auto st = session_->read_state(kDevHead);
      if (st && st->joints && st->joints_len == head_now.size()) 
      {
        for (size_t k = 0; k < st->joints_len; ++k) 
          head_now[k] = st->joints[k];
        ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    if (!ok) 
    {
      RCLCPP_WARN(logger_, "[%s] no HEAD state within %d ms", get_name().c_str(), timeout_ms);
      return false;
    }
  }

  // ---- seed state + command buffers ----
  bufs_.seed(layout_, left_now, head_now, right_now);

  RCLCPP_INFO(logger_,
      "[%s] wait_and_seed: left=%zu head=%zu right=%zu",
      get_name().c_str(),
      left_now.size(), head_now.size(), right_now.size());

  // ---- all-zero warning (no failure) ----
  if ((left_now .empty() || JointBuffers::all_zero(left_now,  kZeroEps)) &&
      (head_now .empty() || JointBuffers::all_zero(head_now,  kZeroEps)) &&
      (right_now.empty() || JointBuffers::all_zero(right_now, kZeroEps)))
  {
    RCLCPP_WARN(logger_,
      "[%s] all initial states are ZERO — agent may be in --sim with zero seed",
      get_name().c_str());
  }

  return true;
}

// ============================================================
// reset_joint_limiters
// ============================================================
void DualArmsTelemanipHardwareInterface::reset_joint_limiters()
{
  for (auto& jl : joint_limiters_) 
  {
    jl.initialized = false;
    jl.last_vel    = 0.0;
    jl.clamp_count = 0;
  }
  RCLCPP_DEBUG(logger_, "[%s] joint limiters reset", get_name().c_str());
}

}  // namespace dual_arm_hardware_interface

// ============================================================
// pluginlib export
// ============================================================
PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::DualArmsTelemanipHardwareInterface,
    hardware_interface::SystemInterface)