#include "dual_arm_hardware_interface/dual_arms_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <sstream>
#include <thread>

#include <pluginlib/class_list_macros.hpp>

namespace dual_arm_hardware_interface
{

using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;

namespace
{
template <typename T>
std::string vec_to_str(const std::vector<T>& v)
{
  std::ostringstream oss;
  oss << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    oss << v[i];
    if (i + 1 < v.size()) oss << ", ";
  }
  oss << ']';
  return oss.str();
}
}  // namespace

// ============================================================
// on_init_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_init_impl()
{
  const size_t n = info_.joints.size();

  bufs_.allocate(n);
  rt_control_states_.assign(n, 0.0);

  layout_.build(info_);
  layout_.log(logger_, component_name());

  cmdbuf_.allocate(layout_.left_indices_.size(),
                   layout_.head_indices_.size(),
                   layout_.right_indices_.size());

  // ---- URDF parameters ----
  HardwareParams params(info_);

  sim_left_  = params.get_bool("sim_left",  false);
  sim_right_ = params.get_bool("sim_right", false);
  RCLCPP_INFO(logger_, "[%s] sim_left=%d, sim_right=%d",
              component_name(),
              static_cast<int>(sim_left_), static_cast<int>(sim_right_));

  trajectory_.load(params);

  RCLCPP_INFO(logger_, "[%s] initial trajectory_mode=%u, radio=%u",
            component_name(),
            static_cast<unsigned>(trajectory_.mode.load()),
            static_cast<unsigned>(trajectory_.radio.load()));

  // ---- Common helpers: CSV logger + tuner node ----
  csv_ = std::make_unique<CsvLogger>(logger_, component_name());
  csv_->start();

  tuner_ = std::make_unique<TrajectoryTuner>(
    logger_, get_name(),
    /*on_tuner=*/ [this](uint8_t m, uint16_t r) {
      const uint8_t  prev_mode  = trajectory_.mode.load();
      const uint16_t prev_radio = trajectory_.radio.load();
      if (prev_mode == m && prev_radio == r) 
      {
        RCLCPP_INFO(logger_,
          "[%s] trajectory_tuner: mode/radio unchanged (%u, %u); no restart",
          component_name(),
          static_cast<unsigned>(prev_mode), static_cast<unsigned>(prev_radio));
        return;
      }
      pending_mode_.store(m, std::memory_order_release);
      pending_radio_.store(r, std::memory_order_release);
      stream_restart_requested_.store(true, std::memory_order_release);

      RCLCPP_INFO(logger_,
        "[%s] trajectory_tuner: request restart mode %u->%u, radio %u->%u",
        component_name(),
        static_cast<unsigned>(prev_mode),  static_cast<unsigned>(m),
        static_cast<unsigned>(prev_radio), static_cast<unsigned>(r));
    },
    /*on_csv=*/ [this](bool on) { csv_->request(on); });
  tuner_->start();

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_configure_impl()
{
  constexpr int      timeout_ms = 2000;
  constexpr int      poll_ms    = 20;
  constexpr double   zero_eps   = 1e-6;

  std::vector<float> left_now, head_now, right_now;

  auto wait_for_joint_state = [&](Group group, std::vector<float>& joints, int timeout_ms) -> bool
  {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    do 
    {
      auto [code, value] = api_->get_joint(group);
      if (code == RetCode::SUCCESS && !value.empty()) 
      {
        joints = std::move(value);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  };

  if (!layout_.left_indices_.empty()) 
  {
    if (!wait_for_joint_state(Group::LEFT_ARM, left_now, timeout_ms)) 
    {
      RCLCPP_WARN(logger_, "[%s] no LEFT_ARM state within %d ms",
                  component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }
    if (left_now.size() != layout_.left_indices_.size()) 
    {
      RCLCPP_ERROR(logger_, "[%s] LEFT_ARM size mismatch: got %zu, expected %zu",
                   component_name(), left_now.size(), layout_.left_indices_.size());
      return CallbackReturn::ERROR;
    }
  }
  if (!layout_.head_indices_.empty()) 
  {
    if (!wait_for_joint_state(Group::HEAD, head_now, timeout_ms)) 
    {
      RCLCPP_WARN(logger_, "[%s] no HEAD state within %d ms",
                  component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }
    if (head_now.size() != layout_.head_indices_.size()) 
    {
      RCLCPP_ERROR(logger_, "[%s] HEAD size mismatch: got %zu, expected %zu",
                   component_name(), head_now.size(), layout_.head_indices_.size());
      return CallbackReturn::ERROR;
    }
  }
  if (!layout_.right_indices_.empty()) 
  {
    if (!wait_for_joint_state(Group::RIGHT_ARM, right_now, timeout_ms)) 
    {
      RCLCPP_WARN(logger_, "[%s] no RIGHT_ARM state within %d ms",
                  component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }
    if (right_now.size() != layout_.right_indices_.size()) 
    {
      RCLCPP_ERROR(logger_, "[%s] RIGHT_ARM size mismatch: got %zu, expected %zu",
                   component_name(), right_now.size(), layout_.right_indices_.size());
      return CallbackReturn::ERROR;
    }
  }

  bufs_.seed(layout_, left_now, head_now, right_now);

  RCLCPP_INFO(logger_, "[%s] initial state captured: left=%zu head=%zu right=%zu",
              component_name(), left_now.size(), head_now.size(), right_now.size());

  if ((left_now.empty()  || JointBuffers::all_zero(left_now,  zero_eps)) &&
      (head_now.empty()  || JointBuffers::all_zero(head_now,  zero_eps)) &&
      (right_now.empty() || JointBuffers::all_zero(right_now, zero_eps)))
  {
    RCLCPP_WARN(logger_,
      "[%s] all initial states are ZERO — rt_control may not be "
      "simulating / connected, or it is at startup.", component_name());
  } 
  else 
  {
    RCLCPP_INFO(logger_, "[%s] non-zero seed applied", component_name());
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_activate_impl()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (api_->set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS) 
  {
    RCLCPP_ERROR(logger_, "[%s] set_group(LEFT_ARM, true) failed", component_name());
    api_->set_group(Group::LEFT_ARM, false);
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (api_->set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS) 
  {
    RCLCPP_ERROR(logger_, "[%s] set_group(RIGHT_ARM, true) failed", component_name());
    api_->set_group(Group::RIGHT_ARM, false);
    return CallbackReturn::ERROR;
  }

  const RetCode ret = api_->start_dual_arm_streaming(true, trajectory_.mode.load(), trajectory_.radio.load(), 100);
  if (ret != RetCode::SUCCESS) 
  {
    RCLCPP_ERROR(logger_, "[%s] start_dual_arm_streaming failed: %d", component_name(), static_cast<int>(ret));
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "[%s] on_activate_impl done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_deactivate_impl()
{
  api_->set_group(Group::LEFT_ARM,  false);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  api_->set_group(Group::RIGHT_ARM, false);

  stream_restart_requested_.store(false, std::memory_order_release);

  api_->stop_dual_arm_streaming();

  RCLCPP_INFO(logger_, "[%s] on_deactivate_impl done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_cleanup_impl / on_shutdown_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_cleanup_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_cleanup_impl", component_name());
  if (csv_)   
    csv_->stop();
  if (tuner_) 
    tuner_->stop();
  csv_.reset();
  tuner_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualArmsHardwareInterface::on_shutdown_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_shutdown_impl", component_name());
  if (csv_)   
    csv_->stop();
  if (tuner_) 
    tuner_->stop();
  csv_.reset();
  tuner_.reset();
  return CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface>
DualArmsHardwareInterface::export_state_interfaces()
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
DualArmsHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(info_.joints.size());

  for (const auto& joint : info_.joints) {
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
hardware_interface::return_type DualArmsHardwareInterface::read(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  if (!api_) 
    return hardware_interface::return_type::OK;

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t expected = layout_.total();

  auto [ret_p, positions] = api_->get_joint(Group::ALL);
  if (ret_p == RetCode::SUCCESS && positions.size() == expected) 
  {
    size_t off = 0;
    for (size_t i : layout_.left_indices_) 
    {
      rt_control_states_[i]  = positions[off];
      bufs_.hw_position_states_[i] = positions[off++];
    }
    for (size_t i : layout_.head_indices_) 
    {
      rt_control_states_[i]  = positions[off];
      bufs_.hw_position_states_[i] = positions[off++];
    }
    for (size_t i : layout_.right_indices_) 
    {
      rt_control_states_[i]  = positions[off];
      bufs_.hw_position_states_[i] = positions[off++];
    }
  }

  if (sim_left_)
    for (size_t i : layout_.left_indices_)
      bufs_.hw_position_states_[i] = bufs_.hw_position_commands_[i];
  if (sim_right_)
    for (size_t i : layout_.right_indices_)
      bufs_.hw_position_states_[i] = bufs_.hw_position_commands_[i];

  auto [ret_v, velocities] = api_->get_joint_velocity(Group::ALL);
  if (ret_v == RetCode::SUCCESS && velocities.size() == expected) 
  {
    size_t off = 0;
    for (size_t i : layout_.left_indices_)  
      bufs_.hw_velocity_states_[i] = velocities[off++];
    for (size_t i : layout_.head_indices_)  
      bufs_.hw_velocity_states_[i] = velocities[off++];
    for (size_t i : layout_.right_indices_) 
      bufs_.hw_velocity_states_[i] = velocities[off++];
  }

  if (sim_left_)
    for (size_t i : layout_.left_indices_)  
      bufs_.hw_velocity_states_[i] = 0.0;
  if (sim_right_)
    for (size_t i : layout_.right_indices_) 
      bufs_.hw_velocity_states_[i] = 0.0;

  if (++debug_read_tick_ >= debug_every_n_) 
  {
    debug_read_tick_ = 0;
    auto to_str = [&](const std::vector<size_t>& idx) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4) << "[";
      for (size_t k = 0; k < idx.size(); ++k) 
      {
        oss << bufs_.hw_position_states_[idx[k]];
        if (k + 1 < idx.size()) oss << ", ";
      }
      oss << "]";
      return oss.str();
    };
    RCLCPP_INFO(logger_, "[%s] read:\n  left:  %s\n  head:  %s\n  right: %s",
                component_name(),
                to_str(layout_.left_indices_).c_str(),
                to_str(layout_.head_indices_).c_str(),
                to_str(layout_.right_indices_).c_str());
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write
// ============================================================
hardware_interface::return_type DualArmsHardwareInterface::write(
  const rclcpp::Time&, const rclcpp::Duration& period)
{
  if (!api_ || !is_active()) 
    return hardware_interface::return_type::OK;

  // ---- Stream restart requested by the tuner ----
  if (stream_restart_requested_.exchange(false, std::memory_order_acq_rel)) 
  {
    const uint8_t  new_mode  = pending_mode_.load(std::memory_order_acquire);
    const uint16_t new_radio = pending_radio_.load(std::memory_order_acquire);

    api_->stop_dual_arm_streaming();
    const RetCode sr = api_->start_dual_arm_streaming(true, new_mode, new_radio, 100);

    if (sr != RetCode::SUCCESS) 
    {
      RCLCPP_ERROR(logger_,
        "[%s] start_dual_arm_streaming during tuner restart failed: %d (mode=%u radio=%u)",
        component_name(), static_cast<int>(sr),
        static_cast<unsigned>(new_mode), static_cast<unsigned>(new_radio));
    } 
    else 
    {
      trajectory_.set(new_mode, new_radio);
      RCLCPP_INFO(logger_, "[%s] stream restarted: mode=%u radio=%u",
                  component_name(),
                  static_cast<unsigned>(new_mode), static_cast<unsigned>(new_radio));
    }
  }

  static thread_local std::chrono::steady_clock::time_point prev_entry{};
  const auto t_entry = std::chrono::steady_clock::now();
  double actual_dt_ms = 0.0;
  if (prev_entry.time_since_epoch().count())
    actual_dt_ms = std::chrono::duration<double, std::milli>(t_entry - prev_entry).count();
  prev_entry = t_entry;

  // ---- 1) Snapshot commands (handles sim_left_/sim_right_) ----
  cmdbuf_.gather(layout_, bufs_, mutex_, sim_left_, sim_right_, &rt_control_states_);
  cmdbuf_.pack();

  const uint8_t  mode  = trajectory_.mode.load();
  const uint16_t radio = trajectory_.radio.load();

  if (++debug_write_tick_ >= debug_every_n_) 
  {
    debug_write_tick_ = 0;
    char buf_l[256], buf_h[256], buf_r[256];
    auto w = [](char* b, size_t n, const std::vector<float>& v) {
      size_t p = 0;
      p += std::snprintf(b + p, n - p, "[");
      for (size_t k = 0; k < v.size() && p < n - 16; ++k)
        p += std::snprintf(b + p, n - p, "%s%.4f", (k ? ", " : ""), v[k]);
      std::snprintf(b + p, n - p, "]");
    };
    w(buf_l, sizeof(buf_l), cmdbuf_.left_cmd_buf_);
    w(buf_h, sizeof(buf_h), cmdbuf_.head_cmd_buf_);
    w(buf_r, sizeof(buf_r), cmdbuf_.right_cmd_buf_);
    RCLCPP_INFO(logger_,
      "[%s] write:\n  left:  %s\n  head:  %s\n  right: %s mode: %u radio: %u",
      component_name(), buf_l, buf_h, buf_r,
      static_cast<unsigned>(mode), static_cast<unsigned>(radio));
  }

  // ---- 2) Transport send ----
  const auto t0 = std::chrono::steady_clock::now();
  RetCode ret = RetCode::SUCCESS;

  const size_t n_left  = layout_.left_indices_.size();
  const size_t n_head  = layout_.head_indices_.size();
  const size_t n_right = layout_.right_indices_.size();
  const float* base    = cmdbuf_.cmd_buffer_.data();

  if (n_left > 0) 
  {
    const std::vector<float> lc(base, base + n_left);
    ret = api_->set_left_joint_target(lc);
  }
  if (ret == RetCode::SUCCESS && n_right > 0) 
  {
    const std::vector<float> rc(base + n_left + n_head,
                                base + n_left + n_head + n_right);
    ret = api_->set_right_joint_target(rc);
  }
  const auto t1 = std::chrono::steady_clock::now();

  if (ret != RetCode::SUCCESS) 
  {
    ++error_streak_;
    if (error_streak_ == 1 || error_streak_ % 100 == 0)
      RCLCPP_WARN(logger_, "[%s] set_*_joint_target failed: %d (streak=%d)",
                  component_name(), static_cast<int>(ret), error_streak_);
  } 
  else 
  {
    error_streak_ = 0;
  }

  // ---- 3) Timing publish + stats ----
  const double call_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double period_ms = period.seconds() * 1000.0;

  if (tuner_) 
    tuner_->publish_timing(call_ms, period_ms, period_ms * 1.5, actual_dt_ms);

  double dm, ds, dmin, dmax, cm, cmax;
  size_t n_samples;
  if (timing_stats_.update(actual_dt_ms, call_ms, period_ms, dm, ds, dmin, dmax, cm, cmax, n_samples))
  {
    RCLCPP_WARN(logger_,
      "[%s] write dt stats (N=%zu): mean=%.2f sd=%.2f min=%.2f max=%.2f ms "
      "(period=%.2f) | call mean=%.2f max=%.2f ms",
      component_name(), n_samples, dm, ds, dmin, dmax, period_ms, cm, cmax);
  }

  // ---- 4) Delta publish ----
  if (tuner_) 
    tuner_->publish_delta(cmdbuf_.cmd_buffer_, cmdbuf_.last_cmd_buffer_);

  // ---- 5) CSV push ----
  if (csv_ && csv_->is_open()) 
  {
    csv_->push(make_csv_row(
      cmdbuf_.cmd_buffer_, layout_,
      std::chrono::duration<double>(t1.time_since_epoch()).count(),
      /*follow=*/1, mode, radio));
  }

  cmdbuf_.swap();
  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  dual_arm_hardware_interface::DualArmsHardwareInterface,
  hardware_interface::SystemInterface)