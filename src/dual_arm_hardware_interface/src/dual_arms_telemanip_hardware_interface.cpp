#include "dual_arm_hardware_interface/dual_arms_telemanip_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <sstream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dual_arm_hardware_interface
{

namespace {

// ============================================================
// Joint classification by name suffix:
//   *_l  -> left arm   (SDK device "left",  7 joints)
//   *_r  -> right arm  (SDK device "right", 7 joints)
//   else -> head       (SDK device "head",  2 joints)
// ============================================================
enum class JointSide { kLeft, kHead, kRight };

JointSide classify(const std::string& name)
{
  if (name.size() >= 2)
  {
    const std::string suffix = name.substr(name.size() - 2);
    if (suffix == "_l") return JointSide::kLeft;
    if (suffix == "_r") return JointSide::kRight;
  }
  return JointSide::kHead;
}

// ---- device_id constants (must match the agent's TOML) ----
constexpr const char* kDevLeft  = "left";
constexpr const char* kDevHead  = "head";
constexpr const char* kDevRight = "right";

}  // namespace

// ============================================================
// on_init
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger(std::string(get_name()) + "_hardware_interface");
  RCLCPP_INFO(logger_, "[%s] on_init: joints=%zu", get_name().c_str(), info.joints.size());

  // ============================================================
  // 1. Allocate per-joint buffers
  // ============================================================
  const size_t n = info_.joints.size();

  hw_position_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_    .assign(n, 0.0);
  hw_position_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());

  joint_indices_.clear();
  joint_indices_.reserve(n);
  left_indices_.clear();
  head_indices_.clear();
  right_indices_.clear();

  for (size_t i = 0; i < n; ++i)
  {
    const auto& joint = info_.joints[i];
    joint_indices_[joint.name] = i;

    switch (classify(joint.name))
    {
      case JointSide::kLeft:  left_indices_ .push_back(i); break;
      case JointSide::kHead:  head_indices_ .push_back(i); break;
      case JointSide::kRight: right_indices_.push_back(i); break;
    }
  }

  auto sort_by_name = [&](std::vector<size_t>& idx) {
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
      return info_.joints[a].name < info_.joints[b].name;
    });
  };
  sort_by_name(left_indices_);
  sort_by_name(head_indices_);
  sort_by_name(right_indices_);

  RCLCPP_INFO(logger_,
      "[%s] joint classification: left=%zu, head=%zu, right=%zu",
      get_name().c_str(),
      left_indices_.size(), head_indices_.size(), right_indices_.size());

  auto log_indices = [&](const char* label, const std::vector<size_t>& idx) {
    for (size_t i : idx)
      RCLCPP_INFO(logger_, "[%s]   %s: %s",
                  get_name().c_str(), label, info_.joints[i].name.c_str());
  };
  log_indices("left",  left_indices_);
  log_indices("head",  head_indices_);
  log_indices("right", right_indices_);

  // ============================================================
  // 2. Read URDF hardware parameters
  // ============================================================
  auto get_param = [&](const char* key) -> const std::string* {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty()) return nullptr;
    return &it->second;
  };

  auto get_u64 = [&](const char* key, uint64_t def) -> uint64_t {
    if (auto* v = get_param(key)) {
      try { return static_cast<uint64_t>(std::stoull(*v)); }
      catch (...) { return def; }
    }
    return def;
  };

  auto get_u16 = [&](const char* key, uint16_t def) -> uint16_t {
    if (auto* v = get_param(key)) {
      try { return static_cast<uint16_t>(std::stoi(*v)); }
      catch (...) { return def; }
    }
    return def;
  };

  // ---- cell_id (required) ----
  if (auto* v = get_param("cell_id")) {
    cell_id_ = *v;
  } else {
    RCLCPP_ERROR(logger_,
        "[%s] missing required hardware parameter 'cell_id'",
        get_name().c_str());
    return CallbackReturn::ERROR;
  }

  period_ms_     = get_u64("period_ms",     10);
  agent_wait_ms_ = get_u64("agent_wait_ms", 10000);

  // ---- zenoh.json5 path (optional: derive from package share if not given) ----
  if (auto* v = get_param("zenoh_config")) {
    zenoh_config_path_ = *v;
  } else {
    try {
      const std::string share =
          ament_index_cpp::get_package_share_directory("dual_arm_hardware_interface");
      zenoh_config_path_ = share + "/config/zenoh.json5";
    } catch (const std::exception& e) {
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

  // ============================================================
  // 3. Trajectory parameters (initial values from URDF)
  // ============================================================
  const uint16_t init_mode  = get_u16("trajectory_mode",  2);
  const uint16_t init_radio = get_u16("trajectory_radio", 500);
  trajectory_mode_.store(static_cast<uint8_t>(std::min<uint16_t>(init_mode, 2)));
  trajectory_radio_.store(init_radio);

  RCLCPP_INFO(logger_, "[%s] initial trajectory_mode=%u, radio=%u",
    get_name().c_str(),
    static_cast<unsigned>(trajectory_mode_.load()),
    static_cast<unsigned>(trajectory_radio_.load()));

  // ============================================================
  // 关节限幅器参数（可选，默认禁用 = 0）
  // ============================================================
  auto read_double = [&](const char* key, double def) -> double {
    if (auto* v = get_param(key)) {
      try { return std::stod(*v); } catch (...) { return def; }
    }
    return def;
  };

  const double max_vel_deg_s  = read_double("max_joint_velocity_deg_s",  0.0);
  const double max_acc_deg_s2 = read_double("max_joint_acceleration_deg_s2", 0.0);

  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  max_joint_vel_rad_s_  = max_vel_deg_s  * kDegToRad;
  max_joint_acc_rad_s2_ = max_acc_deg_s2 * kDegToRad;

  RCLCPP_INFO(logger_,
      "[%s] joint limiter: max_vel=%.1f deg/s (%.4f rad/s), "
      "max_acc=%.1f deg/s^2 (%.4f rad/s^2)",
      get_name().c_str(), max_vel_deg_s, max_joint_vel_rad_s_,
      max_acc_deg_s2, max_joint_acc_rad_s2_);

  // 分配
  joint_limiters_.resize(n);
  for (auto& jl : joint_limiters_) {
    jl.max_vel = max_joint_vel_rad_s_;
    jl.max_acc = max_joint_acc_rad_s2_;
    jl.initialized = false;
  }

  // ============================================================
  // 4. Prepare per-group command buffers
  // ============================================================
  left_cmd_buf_ .assign(left_indices_ .size(), 0.0F);
  head_cmd_buf_ .assign(head_indices_ .size(), 0.0F);
  right_cmd_buf_.assign(right_indices_.size(), 0.0F);

  cmd_buffer_     .assign(n, 0.0F);
  last_cmd_buffer_.assign(n, 0.0F);

  // ============================================================
  // 5. Create the TelemanipSession (blocking)
  // ============================================================
  std::vector<std::string> device_ids;
  if (!left_indices_ .empty()) device_ids.emplace_back(kDevLeft);
  if (!head_indices_ .empty()) device_ids.emplace_back(kDevHead);
  if (!right_indices_.empty()) device_ids.emplace_back(kDevRight);

  if (device_ids.empty()) {
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

  // ============================================================
  // 6. Start embedded helpers (tuner + CSV writer)
  // ============================================================
  start_tuner_node();
  start_csv_writer();

  RCLCPP_INFO(logger_, "[%s] on_init done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure — wait for first state and seed command buffers
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "[%s] on_configure", get_name().c_str());

  if (!session_ || !session_->valid()) {
    RCLCPP_ERROR(logger_, "[%s] session_ not available", get_name().c_str());
    return CallbackReturn::ERROR;
  }

  if (!wait_and_seed_all_groups(2000)) {
    RCLCPP_ERROR(logger_, "[%s] on_configure: no initial state",
                 get_name().c_str());
    return CallbackReturn::ERROR;
  }

  reset_joint_limiters();

  RCLCPP_INFO(logger_, "[%s] on_configure done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate — enable all groups and wait for state
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "[%s] on_activate", get_name().c_str());

  if (!session_ || !session_->valid()) {
    RCLCPP_ERROR(logger_, "[%s] session_ not available", get_name().c_str());
    return CallbackReturn::ERROR;
  }

  // ============================================================
  // 1. 让 zenoh channel 稳定（避免 activate 后头几帧丢包）
  // ============================================================
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // ============================================================
  // 2. 重新读取当前位置 —— deactivate → activate 后机器人可能动了，
  //    必须用当前值 seed，否则第一次 write() 会跳变。
  // ============================================================
  if (!wait_and_seed_all_groups(2000)) {
    RCLCPP_ERROR(logger_,
        "[%s] on_activate: failed to read current state",
        get_name().c_str());
    return CallbackReturn::ERROR;
  }

  // ============================================================
  // 3. 复位限幅器 —— 以 seed 后的位置作为限幅起点
  // ============================================================
  reset_joint_limiters();

  // ============================================================
  // 4. 逐个使能：left → right → head
  // ============================================================

  // ---- 左臂 ----
  if (!left_indices_.empty()) {
    if (!session_->group_enable(kDevLeft, true)) {
      RCLCPP_ERROR(logger_, "[%s] group_enable(left, true) failed",
                   get_name().c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "[%s] group_enable(left, true) OK",
                get_name().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ---- 右臂（失败时回滚左臂）----
  if (!right_indices_.empty()) {
    if (!session_->group_enable(kDevRight, true)) {
      RCLCPP_ERROR(logger_, "[%s] group_enable(right, true) failed",
                   get_name().c_str());
      session_->group_enable(kDevLeft, false);
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "[%s] group_enable(right, true) OK",
                get_name().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ---- 头（可选，失败不阻塞）----
  if (!head_indices_.empty()) {
    if (!session_->group_enable(kDevHead, true)) {
      RCLCPP_WARN(logger_, "[%s] group_enable(head, true) failed (continuing)",
                  get_name().c_str());
    } else {
      RCLCPP_INFO(logger_, "[%s] group_enable(head, true) OK",
                  get_name().c_str());
    }
  }

  is_activated_ = true;
  RCLCPP_INFO(logger_, "[%s] on_activate done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate — disable all groups
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "[%s] on_deactivate", get_name().c_str());

  if (session_ && session_->valid())
  {
    if (!left_indices_.empty() && !session_->group_enable(kDevLeft, false)) {
      RCLCPP_WARN(logger_, "[%s] group_enable(left, false) rejected locally (expected)",
               get_name().c_str());
    }
    if (!right_indices_.empty() && !session_->group_enable(kDevRight, false)) {
      RCLCPP_DEBUG(logger_, "[%s] group_enable(right, false) rejected locally (expected)",
                  get_name().c_str());
    }
    if (!head_indices_.empty()) {
      session_->group_enable(kDevHead, false);   // best-effort
    }
  }

  is_activated_ = false;
  RCLCPP_INFO(logger_, "[%s] on_deactivate done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_cleanup
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "[%s] on_cleanup", get_name().c_str());

  stop_csv_writer();
  stop_tuner_node();

  session_.reset();
  is_activated_ = false;

  RCLCPP_INFO(logger_, "[%s] on_cleanup done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_shutdown
// ============================================================
CallbackReturn DualArmsTelemanipHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(logger_, "[%s] on_shutdown", get_name().c_str());

  stop_csv_writer();
  stop_tuner_node();

  session_.reset();
  is_activated_ = false;

  RCLCPP_INFO(logger_, "[%s] on_shutdown done", get_name().c_str());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// is_active
// ============================================================
bool DualArmsTelemanipHardwareInterface::is_active() const
{
  return get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface>
DualArmsTelemanipHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(info_.joints.size() * 3);

  for (const auto& joint : info_.joints) {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end()) continue;
    const size_t idx = it->second;

    for (const auto& s : joint.state_interfaces) {
      if (s.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                         &hw_position_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_VELOCITY)
        out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                         &hw_velocity_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_EFFORT)
        out.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT,
                         &hw_effort_states_[idx]);
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

  for (const auto& joint : info_.joints) {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end()) continue;
    const size_t idx = it->second;

    for (const auto& c : joint.command_interfaces) {
      if (c.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                         &hw_position_commands_[idx]);
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
  if (!session_ || !session_->valid()) {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // ---- 先清零速度，避免 SDK 没数据时保留 NaN ----
  for (size_t i = 0; i < hw_velocity_states_.size(); ++i)
    hw_velocity_states_[i] = 0.0;

  // ============================================================
  // 每个组一次 read_state，同时处理位置 + 速度
  // ============================================================
  auto update_group = [&](const std::string& dev,
                          const std::vector<size_t>& idx)
  {
    if (idx.empty()) return;

    auto st = session_->read_state(dev);
    if (!st) return;   // 无快照 → 保留旧值

    // ---- position ----
    if (st->joints && st->joints_len == idx.size())
    {
      for (size_t k = 0; k < idx.size(); ++k)
        hw_position_states_[idx[k]] = st->joints[k];
    }

    // ---- velocity ----
    if (st->velocity && st->velocity_len == idx.size())
    {
      for (size_t k = 0; k < idx.size(); ++k)
        hw_velocity_states_[idx[k]] = st->velocity[k];
    }
  };

  update_group(kDevLeft,  left_indices_);
  update_group(kDevRight, right_indices_);
  update_group(kDevHead,  head_indices_);

  // ---- Effort 恒为 0（SDK 不提供）----
  for (size_t i = 0; i < hw_effort_states_.size(); ++i)
    hw_effort_states_[i] = 0.0;

  // ---- Debug print ----
  if (++debug_read_tick_ >= debug_every_n_) {
    debug_read_tick_ = 0;
    auto to_str_by_indices = [&](const std::vector<size_t>& idx) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4) << "[";
      for (size_t k = 0; k < idx.size(); ++k) {
        oss << hw_position_states_[idx[k]];
        if (k + 1 < idx.size()) oss << ", ";
      }
      oss << "]";
      return oss.str();
    };
    RCLCPP_INFO(logger_, "[%s] read:\n  left:  %s\n  head:  %s\n  right: %s",
                get_name().c_str(),
                to_str_by_indices(left_indices_ ).c_str(),
                to_str_by_indices(head_indices_ ).c_str(),
                to_str_by_indices(right_indices_).c_str());
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write
// ============================================================
hardware_interface::return_type DualArmsTelemanipHardwareInterface::write(
    const rclcpp::Time& /* time */, const rclcpp::Duration& period)
{
  if (!session_ || !session_->valid() || !is_active()) {
    return hardware_interface::return_type::OK;
  }

  static thread_local std::chrono::steady_clock::time_point prev_entry{};
  const auto t_entry = std::chrono::steady_clock::now();

  double actual_dt_ms = 0.0;
  if (prev_entry.time_since_epoch().count() != 0) {
    actual_dt_ms = std::chrono::duration<double, std::milli>(t_entry - prev_entry).count();
  }
  prev_entry = t_entry;

  // ============================================================
  // 1. Snapshot commands under the mutex
  // ============================================================
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t k = 0; k < left_indices_.size(); ++k)
      left_cmd_buf_[k] = static_cast<float>(hw_position_commands_[left_indices_[k]]);
    for (size_t k = 0; k < head_indices_.size(); ++k)
      head_cmd_buf_[k] = static_cast<float>(hw_position_commands_[head_indices_[k]]);
    for (size_t k = 0; k < right_indices_.size(); ++k)
      right_cmd_buf_[k] = static_cast<float>(hw_position_commands_[right_indices_[k]]);
  }

  const uint8_t  mode  = trajectory_mode_.load();
  const uint16_t radio = trajectory_radio_.load();

  // ============================================================
  // ★ 应用关节限幅（速度 + 加速度）
  // ============================================================
  const double dt_s = period.seconds();

  if (dt_s > 0.0 && max_joint_vel_rad_s_ > 0.0)
  {
    // 左臂
    for (size_t k = 0; k < left_indices_.size(); ++k) {
      const size_t gi = left_indices_[k];
      left_cmd_buf_[k] = static_cast<float>(
          joint_limiters_[gi].limit(left_cmd_buf_[k], dt_s));
    }
    // 头
    for (size_t k = 0; k < head_indices_.size(); ++k) {
      const size_t gi = head_indices_[k];
      head_cmd_buf_[k] = static_cast<float>(
          joint_limiters_[gi].limit(head_cmd_buf_[k], dt_s));
    }
    // 右臂
    for (size_t k = 0; k < right_indices_.size(); ++k) {
      const size_t gi = right_indices_[k];
      right_cmd_buf_[k] = static_cast<float>(
          joint_limiters_[gi].limit(right_cmd_buf_[k], dt_s));
    }
  }

  {
    size_t k = 0;
    for (float v : left_cmd_buf_ ) cmd_buffer_[k++] = v;
    for (float v : head_cmd_buf_ ) cmd_buffer_[k++] = v;
    for (float v : right_cmd_buf_) cmd_buffer_[k++] = v;
  }

  // ============================================================
  // 2. Stage joint commands (never blocks, never sends)
  // ============================================================
  const auto t0 = std::chrono::steady_clock::now();

  bool stage_ok = true;
  if (!left_indices_.empty())
    stage_ok &= session_->stage_joint(kDevLeft, left_cmd_buf_.data(),
                                      left_cmd_buf_.size(), true, mode, radio);
  if (!right_indices_.empty())
    stage_ok &= session_->stage_joint(kDevRight, right_cmd_buf_.data(),
                                      right_cmd_buf_.size(), true, mode, radio);
  if (!head_indices_.empty())
    stage_ok &= session_->stage_joint(kDevHead, head_cmd_buf_.data(),
                                      head_cmd_buf_.size(), true, mode, radio);

  const auto t1 = std::chrono::steady_clock::now();

  if (!stage_ok) {
    ++error_streak_;
    if (error_streak_ == 1 || error_streak_ % 100 == 0) {
      RCLCPP_WARN(logger_, "[%s] stage_joint failed (streak=%d)",
                  get_name().c_str(), error_streak_);
    }
  } else {
    error_streak_ = 0;
  }

  // ============================================================
  // 3. Timing publish + rolling stats
  // ============================================================
  const double call_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double period_ms    = period.seconds() * 1000.0;
  const double threshold_ms = period_ms * 1.5;

  if (timing_pub_) {
    if (++timing_sub_check_tick_ >= 100) {
      timing_sub_check_tick_ = 0;
      timing_has_sub_ = (timing_pub_->get_subscription_count() > 0);
    }
    if (timing_has_sub_) {
      auto msg = std_msgs::msg::Float32MultiArray();
      msg.data.resize(4);
      msg.data[0] = static_cast<float>(call_ms);
      msg.data[1] = static_cast<float>(period_ms);
      msg.data[2] = static_cast<float>(threshold_ms);
      msg.data[3] = static_cast<float>(actual_dt_ms);
      timing_pub_->publish(msg);
    }
  }

  {
    static thread_local std::array<double, 100> dt_history{};
    static thread_local std::array<double, 100> call_history{};
    static thread_local size_t dt_idx    = 0;
    static thread_local size_t dt_filled = 0;

    dt_history  [dt_idx] = actual_dt_ms;
    call_history[dt_idx] = call_ms;

    ++dt_idx;
    if (dt_filled < dt_history.size()) ++dt_filled;

    if (dt_idx >= dt_history.size()) {
      dt_idx = 0;

      double dt_sum = 0.0, dt_sum2 = 0.0, dt_max = 0.0, dt_min = 1e9;
      double cl_sum = 0.0, cl_max = 0.0;
      const size_t n = dt_filled;

      for (size_t i = 0; i < n; ++i) {
        const double d = dt_history[i];
        const double c = call_history[i];
        dt_sum += d; dt_sum2 += d * d;
        if (d > dt_max) dt_max = d;
        if (d < dt_min) dt_min = d;
        cl_sum += c;
        if (c > cl_max) cl_max = c;
      }

      const double dt_mean = dt_sum / n;
      const double dt_var  = std::max(0.0, dt_sum2 / n - dt_mean * dt_mean);
      const double dt_sd   = std::sqrt(dt_var);
      const double cl_mean = cl_sum / n;

      const bool mean_drift = std::abs(dt_mean - period_ms) > (period_ms * 0.15);
      const bool high_sd    = dt_sd > (period_ms * 0.10);
      const bool tail_spike = dt_max > (period_ms * 2.0);

      if (mean_drift || high_sd || tail_spike) {
        RCLCPP_WARN(logger_,
            "[%s] write dt stats (N=%zu): "
            "mean=%.2f sd=%.2f min=%.2f max=%.2f ms (period=%.2f) | "
            "call mean=%.2f max=%.2f ms%s%s%s",
            get_name().c_str(), n,
            dt_mean, dt_sd, dt_min, dt_max, period_ms,
            cl_mean, cl_max,
            mean_drift ? " [mean_drift]" : "",
            high_sd    ? " [high_sd]"    : "",
            tail_spike ? " [tail_spike]" : "");
      } else {
        RCLCPP_DEBUG(logger_,
            "[%s] write dt stats (N=%zu): mean=%.2f sd=%.2f max=%.2f ms",
            get_name().c_str(), n, dt_mean, dt_sd, dt_max);
      }
    }
  }

  // ============================================================
  // 4. Delta publisher (throttled subscription check)
  // ============================================================
  if (delta_pub_) {
    if (++delta_sub_check_tick_ >= 100) {
      delta_sub_check_tick_ = 0;
      delta_has_sub_ = (delta_pub_->get_subscription_count() > 0);
    }
    if (delta_has_sub_ && last_cmd_buffer_.size() == cmd_buffer_.size()) {
      auto msg = std_msgs::msg::Float32MultiArray();
      msg.data.resize(cmd_buffer_.size());
      for (size_t k = 0; k < cmd_buffer_.size(); ++k)
        msg.data[k] = cmd_buffer_[k] - last_cmd_buffer_[k];
      delta_pub_->publish(msg);
    }
  }

  // ============================================================
  // 5. Debug print of what we staged (throttled)
  // ============================================================
  if (++debug_write_tick_ >= debug_every_n_) {
    debug_write_tick_ = 0;

    char buf_l[256], buf_h[256], buf_r[256];
    auto write_arr = [](char* buf, size_t buflen,
                        const std::vector<float>& v) {
      size_t pos = 0;
      pos += std::snprintf(buf + pos, buflen - pos, "[");
      for (size_t k = 0; k < v.size() && pos < buflen - 16; ++k)
        pos += std::snprintf(buf + pos, buflen - pos, "%s%.4f",
                             (k ? ", " : ""), v[k]);
      std::snprintf(buf + pos, buflen - pos, "]");
    };
    write_arr(buf_l, sizeof(buf_l), left_cmd_buf_);
    write_arr(buf_h, sizeof(buf_h), head_cmd_buf_);
    write_arr(buf_r, sizeof(buf_r), right_cmd_buf_);

    RCLCPP_INFO(logger_,
        "[%s] write:\n  left:  %s\n  head:  %s\n  right: %s mode: %u radio: %u",
        get_name().c_str(), buf_l, buf_h, buf_r,
        static_cast<unsigned>(mode), static_cast<unsigned>(radio));

    // ★ 附加限幅统计
    uint64_t total_clamps = 0;
    uint64_t worst_joint = 0;
    uint64_t worst_count = 0;
    for (size_t i = 0; i < joint_limiters_.size(); ++i) {
      total_clamps += joint_limiters_[i].clamp_count;
      if (joint_limiters_[i].clamp_count > worst_count) {
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
  // 6. CSV push (lock-free, may drop on overflow)
  // ============================================================
  if (csv_is_open_.load(std::memory_order_acquire)) {
    CsvRow row{};
    row.t_s = std::chrono::duration<double>(t1.time_since_epoch()).count();

    const size_t n_left  = left_indices_.size();
    const size_t n_right = right_indices_.size();
    const size_t off_right = n_left + head_indices_.size();

    if (n_left == 7 && n_right == 7) {
      std::memcpy(row.left,  cmd_buffer_.data() + 0,         7 * sizeof(float));
      std::memcpy(row.right, cmd_buffer_.data() + off_right, 7 * sizeof(float));
    } else {
      for (size_t k = 0; k < 7; ++k)
        row.left[k]  = (k < n_left)  ? cmd_buffer_[k] : std::numeric_limits<float>::quiet_NaN();
      for (size_t k = 0; k < 7; ++k)
        row.right[k] = (k < n_right) ? cmd_buffer_[off_right + k] : std::numeric_limits<float>::quiet_NaN();
    }

    row.follow = 1;
    row.mode   = mode;
    row.radio  = radio;

    if (!csv_queue_.push(row))
      csv_dropped_.fetch_add(1, std::memory_order_relaxed);
  }

  // ============================================================
  // 7. Swap cmd_buffer_ <-> last_cmd_buffer_ (zero-copy)
  // ============================================================
  std::swap(cmd_buffer_, last_cmd_buffer_);

  return hardware_interface::return_type::OK;
}

// ============================================================
// Tuner node
// ============================================================
void DualArmsTelemanipHardwareInterface::start_tuner_node()
{
  if (tuner_node_) return;

  const std::string node_name = std::string(get_name()) + "_tuner";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + node_name});
  options.parameter_overrides({});
  options.use_global_arguments(true);

  tuner_node_ = std::make_shared<rclcpp::Node>(node_name, options);

  const std::string ns            = "/" + std::string(get_name());
  const std::string tuner_topic   = ns + "/trajectory_tuner";
  const std::string delta_topic   = ns + "/cmd_delta";
  const std::string timing_topic  = ns + "/write_timing";
  const std::string csv_topic     = ns + "/csv_enable";

  delta_pub_  = tuner_node_->create_publisher<std_msgs::msg::Float32MultiArray>(delta_topic,  1);
  timing_pub_ = tuner_node_->create_publisher<std_msgs::msg::Float32MultiArray>(timing_topic, 1);

  tuner_sub_ = tuner_node_->create_subscription<std_msgs::msg::Int16MultiArray>(
      tuner_topic, 1,
      [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
          RCLCPP_WARN(logger_,
              "[%s] trajectory_tuner: expected [mode, radio], got %zu element(s)",
              get_name().c_str(), msg->data.size());
          return;
        }
        const int16_t raw_mode = msg->data[0];
        if (raw_mode < 0 || raw_mode > 2) {
          RCLCPP_WARN(logger_,
              "[%s] trajectory_mode=%d out of range (valid 0,1,2); ignoring",
              get_name().c_str(), static_cast<int>(raw_mode));
          return;
        }
        int32_t raw_radio = msg->data[1];
        uint16_t clamped_radio;
        if (raw_mode == 1)
          clamped_radio = static_cast<uint16_t>(std::clamp<int32_t>(raw_radio, 0, 100));
        else
          clamped_radio = static_cast<uint16_t>(std::clamp<int32_t>(raw_radio, 0, 999));

        trajectory_mode_.store(static_cast<uint8_t>(raw_mode));
        trajectory_radio_.store(clamped_radio);

        RCLCPP_INFO(logger_,
            "[%s] trajectory_tuner -> mode=%u, radio=%u",
            get_name().c_str(),
            static_cast<unsigned>(trajectory_mode_.load()),
            static_cast<unsigned>(trajectory_radio_.load()));
      });

  csv_enable_sub_ = tuner_node_->create_subscription<std_msgs::msg::Bool>(
      csv_topic, 1,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        csv_requested_.store(msg->data);
        RCLCPP_INFO(logger_, "[%s] csv_enable -> %s",
                    get_name().c_str(), msg->data ? "true" : "false");
      });

  RCLCPP_INFO(logger_, "[%s] write_timing publisher on %s", get_name().c_str(), timing_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] cmd_delta publisher on %s",    get_name().c_str(), delta_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] csv_enable subscriber on %s",  get_name().c_str(), csv_topic.c_str());

  tuner_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  tuner_executor_->add_node(tuner_node_);
  tuner_running_.store(true);

  tuner_thread_ = std::thread([this]() {
    while (rclcpp::ok() && tuner_running_.load()) {
      tuner_executor_->spin_once(std::chrono::milliseconds(10));
    }
  });

  RCLCPP_INFO(logger_, "[%s] tuner node '%s' started (topic: %s)",
              get_name().c_str(), node_name.c_str(), tuner_topic.c_str());
}

void DualArmsTelemanipHardwareInterface::stop_tuner_node()
{
  if (!tuner_node_) return;

  tuner_running_.store(false);

  if (tuner_executor_) tuner_executor_->cancel();

  if (tuner_thread_.joinable()) tuner_thread_.join();

  timing_pub_.reset();
  delta_pub_.reset();
  tuner_sub_.reset();
  csv_enable_sub_.reset();
  tuner_executor_.reset();
  tuner_node_.reset();

  RCLCPP_INFO(logger_, "[%s] tuner node stopped", get_name().c_str());
}

// ============================================================
// CSV writer thread
// ============================================================
void DualArmsTelemanipHardwareInterface::start_csv_writer()
{
  if (csv_writer_thread_.joinable()) {
    RCLCPP_WARN(logger_, "[%s] CSV writer already running", get_name().c_str());
    return;
  }

  csv_writer_running_.store(true, std::memory_order_release);

  try {
    csv_writer_thread_ = std::thread(&DualArmsTelemanipHardwareInterface::csv_writer_loop, this);
  } catch (const std::exception& e) {
    csv_writer_running_.store(false, std::memory_order_release);
    RCLCPP_ERROR(logger_, "[%s] failed to start CSV writer thread: %s",
                 get_name().c_str(), e.what());
    throw;
  }

  RCLCPP_INFO(logger_, "[%s] CSV writer thread started (queue capacity=%zu)",
              get_name().c_str(), kCsvQueueCapacity);
}

void DualArmsTelemanipHardwareInterface::stop_csv_writer()
{
  if (!csv_writer_thread_.joinable()) return;

  csv_writer_running_.store(false, std::memory_order_release);
  csv_writer_thread_.join();

  csv_is_open_.store(false, std::memory_order_release);
  csv_requested_.store(false, std::memory_order_release);

  RCLCPP_INFO(logger_, "[%s] CSV writer thread stopped (total dropped=%lu)",
              get_name().c_str(),
              static_cast<unsigned long>(csv_dropped_.load()));
}

void DualArmsTelemanipHardwareInterface::csv_writer_loop()
{
  std::ofstream file;
  bool          file_open = false;
  std::string   path;
  std::size_t   row_idx = 0;
  double        session_start_s = 0.0;
  std::size_t   writes_since_flush = 0;

  constexpr std::size_t kFlushEvery = 500;
  constexpr int         kBatchMax   = 200;
  constexpr int         kPollMs     = 2;

  CsvRow row{};

  while (csv_writer_running_.load(std::memory_order_acquire))
  {
    try
    {
      const bool want = csv_requested_.load(std::memory_order_acquire);

      // ---- open ----
      if (want && !file_open)
      {
        csv_is_open_.store(false, std::memory_order_release);
        csv_queue_.consume_all([](const CsvRow&){});

        const auto sys_now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(sys_now);
        std::tm tm_buf{};
        localtime_r(&tt, &tm_buf);

        std::ostringstream name;
        name << "/tmp/test_"
             << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
             << "_" << ::getpid()
             << ".csv";
        path = name.str();

        file.open(path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
          RCLCPP_ERROR(logger_, "[%s] cannot open CSV: %s",
                       get_name().c_str(), path.c_str());
          csv_requested_.store(false, std::memory_order_release);
        } else {
          file << "idx,timestamp_s";
          for (int j = 1; j <= 7; ++j) file << ",left_j"  << j;
          for (int j = 1; j <= 7; ++j) file << ",right_j" << j;
          file << ",follow,mode,radio\n";
          file << std::fixed << std::setprecision(6);
          file.flush();

          if (file.fail()) {
            RCLCPP_ERROR(logger_, "[%s] CSV header flush failed: %s",
                         get_name().c_str(), path.c_str());
            file.close();
            csv_requested_.store(false, std::memory_order_release);
          } else {
            row_idx = 0;
            writes_since_flush = 0;
            session_start_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            file_open = true;
            csv_path_ = path;
            csv_dropped_.store(0, std::memory_order_relaxed);
            csv_is_open_.store(true, std::memory_order_release);

            RCLCPP_INFO(logger_, "[%s] CSV log STARTED: %s",
                        get_name().c_str(), path.c_str());
          }
        }
      }
      // ---- close ----
      else if (!want && file_open)
      {
        csv_is_open_.store(false, std::memory_order_release);

        while (csv_queue_.pop(row)) {
          const double ts_s = row.t_s - session_start_s;
          file << row_idx++ << ',' << ts_s;
          for (int k = 0; k < 7; ++k) file << ',' << row.left[k];
          for (int k = 0; k < 7; ++k) file << ',' << row.right[k];
          file << ',' << static_cast<int>(row.follow)
               << ',' << static_cast<int>(row.mode)
               << ',' << static_cast<int>(row.radio) << '\n';
        }
        file.flush();
        const bool write_ok = !file.fail();
        file.close();
        file_open = false;

        if (!write_ok) {
          RCLCPP_ERROR(logger_, "[%s] CSV flush failed on close: %s (%zu rows)",
                       get_name().c_str(), path.c_str(), row_idx);
        }
        RCLCPP_INFO(logger_, "[%s] CSV log STOPPED: %s (%zu rows, %lu dropped)",
                    get_name().c_str(), path.c_str(), row_idx,
                    static_cast<unsigned long>(csv_dropped_.load(std::memory_order_relaxed)));
      }

      // ---- batch consume ----
      if (file_open) {
        int processed = 0;
        while (processed < kBatchMax && csv_queue_.pop(row)) {
          const double ts_s = row.t_s - session_start_s;
          file << row_idx++ << ',' << ts_s;
          for (int k = 0; k < 7; ++k) file << ',' << row.left[k];
          for (int k = 0; k < 7; ++k) file << ',' << row.right[k];
          file << ',' << static_cast<int>(row.follow)
               << ',' << static_cast<int>(row.mode)
               << ',' << static_cast<int>(row.radio) << '\n';

          ++processed;
          ++writes_since_flush;

          if (writes_since_flush >= kFlushEvery) {
            file.flush();
            writes_since_flush = 0;
            if (file.fail()) {
              RCLCPP_ERROR(logger_, "[%s] CSV write failed after %zu rows",
                           get_name().c_str(), row_idx);
              csv_is_open_.store(false, std::memory_order_release);
              file.close();
              file_open = false;
              csv_requested_.store(false, std::memory_order_release);
              break;
            }
          }
        }
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "[%s] CSV writer exception: %s",
                   get_name().c_str(), e.what());
      csv_is_open_.store(false, std::memory_order_release);
      if (file_open) { try { file.flush(); } catch (...) {} try { file.close(); } catch (...) {} file_open = false; }
      csv_requested_.store(false, std::memory_order_release);
    }
    catch (...) {
      RCLCPP_ERROR(logger_, "[%s] CSV writer unknown exception", get_name().c_str());
      csv_is_open_.store(false, std::memory_order_release);
      if (file_open) { try { file.flush(); } catch (...) {} try { file.close(); } catch (...) {} file_open = false; }
      csv_requested_.store(false, std::memory_order_release);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  // ---- shutdown flush ----
  try {
    if (file_open) {
      csv_is_open_.store(false, std::memory_order_release);
      while (csv_queue_.pop(row)) {
        const double ts_s = row.t_s - session_start_s;
        file << row_idx++ << ',' << ts_s;
        for (int k = 0; k < 7; ++k) file << ',' << row.left[k];
        for (int k = 0; k < 7; ++k) file << ',' << row.right[k];
        file << ',' << static_cast<int>(row.follow)
             << ',' << static_cast<int>(row.mode)
             << ',' << static_cast<int>(row.radio) << '\n';
      }
      file.flush();
      const bool write_ok = !file.fail();
      file.close();
      file_open = false;
      if (!write_ok)
        RCLCPP_ERROR(logger_, "[%s] CSV flush failed on shutdown: %s",
                     get_name().c_str(), path.c_str());
      else
        RCLCPP_INFO(logger_, "[%s] CSV log closed on shutdown: %s (%zu rows)",
                    get_name().c_str(), path.c_str(), row_idx);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown exception: %s",
                 get_name().c_str(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown unknown exception",
                 get_name().c_str());
  }
}

bool DualArmsTelemanipHardwareInterface::wait_and_seed_all_groups(int timeout_ms)
{
  if (!session_ || !session_->valid()) {
    RCLCPP_ERROR(logger_, "[%s] wait_and_seed: session_ not available",
                 get_name().c_str());
    return false;
  }

  constexpr int    kPollMs  = 20;
  constexpr double kZeroEps = 1e-6;

  std::vector<float> left_now, head_now, right_now;

  // ---- 左臂 ----
  if (!left_indices_.empty()) {
    left_now.assign(left_indices_.size(), 0.0F);
    bool present = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    bool ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (session_->read_joint(kDevLeft, left_now.data(), left_now.size(),
                               &present) && present) {
        ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    if (!ok) {
      RCLCPP_WARN(logger_, "[%s] no LEFT state within %d ms",
                  get_name().c_str(), timeout_ms);
      return false;
    }
  }

  // ---- 右臂 ----
  if (!right_indices_.empty()) {
    right_now.assign(right_indices_.size(), 0.0F);
    bool present = false;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    bool ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (session_->read_joint(kDevRight, right_now.data(), right_now.size(),
                               &present) && present) {
        ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    if (!ok) {
      RCLCPP_WARN(logger_, "[%s] no RIGHT state within %d ms",
                  get_name().c_str(), timeout_ms);
      return false;
    }
  }

  // ---- 头（head 只有 2 关节，必须用 read_state）----
  if (!head_indices_.empty()) {
    head_now.assign(head_indices_.size(), 0.0F);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    bool ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto st = session_->read_state(kDevHead);
      if (st && st->joints && st->joints_len == head_indices_.size()) {
        for (size_t k = 0; k < st->joints_len; ++k)
          head_now[k] = st->joints[k];
        ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    if (!ok) {
      RCLCPP_WARN(logger_, "[%s] no HEAD state within %d ms",
                  get_name().c_str(), timeout_ms);
      return false;
    }
  }

  // ---- 写入 state + command ----
  auto seed = [&](const std::vector<size_t>& idx, const std::vector<float>& vals) {
    for (size_t k = 0; k < idx.size(); ++k) {
      hw_position_states_  [idx[k]] = vals[k];
      hw_position_commands_[idx[k]] = vals[k];
    }
  };
  seed(left_indices_,  left_now);
  seed(head_indices_,  head_now);
  seed(right_indices_, right_now);

  // ---- 全零检查（只警告，不失败）----
  auto all_zero = [](const std::vector<float>& v, double eps) {
    for (float x : v) if (std::abs(x) > eps) return false;
    return true;
  };

  RCLCPP_INFO(logger_,
      "[%s] wait_and_seed: left=%zu head=%zu right=%zu",
      get_name().c_str(),
      left_now.size(), head_now.size(), right_now.size());

  if ((left_now .empty() || all_zero(left_now,  kZeroEps)) &&
      (head_now .empty() || all_zero(head_now,  kZeroEps)) &&
      (right_now.empty() || all_zero(right_now, kZeroEps)))
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
  for (auto& jl : joint_limiters_) {
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