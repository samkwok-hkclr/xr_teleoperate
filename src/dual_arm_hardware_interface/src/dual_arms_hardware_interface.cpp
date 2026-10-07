#include "dual_arm_hardware_interface/dual_arms_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <thread>
#include <iomanip>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

namespace dual_arm_hardware_interface
{

using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;

namespace {

enum class JointSide { kLeft, kHead, kRight };

// Classify a joint by name suffix:
//   *_l  -> left arm
//   *_r  -> right arm
//   else -> head
JointSide classify(const std::string& name)
{
  if (name.size() >= 2)
  {
    const std::string suffix = name.substr(name.size() - 2);
    if (suffix == "_l") 
      return JointSide::kLeft;
    if (suffix == "_r") 
      return JointSide::kRight;
  }
  return JointSide::kHead;
}

}  // namespace

// ============================================================
// on_init_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_init_impl()
{
  const size_t n = info_.joints.size();

  hw_position_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  // hw_effort_states_    .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_    .assign(n, 0.0);
  hw_position_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());

  joint_indices_.clear();
  joint_indices_.reserve(n);
  left_indices_.clear();
  head_indices_.clear();
  right_indices_.clear();
  rt_control_states_.assign(n, 0.0);

  cmd_buffer_.reserve(n);
  last_cmd_buffer_.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    const auto& joint = info_.joints[i];
    joint_indices_[joint.name] = i;

    switch (classify(joint.name))
    {
      case JointSide::kLeft:  
        left_indices_ .push_back(i); 
        break;
      case JointSide::kHead:  
        head_indices_ .push_back(i); 
        break;
      case JointSide::kRight: 
        right_indices_.push_back(i); 
        break;
    }
  }

  // Sort each list by joint name so the command buffer is deterministic
  // and matches the J1..Jn order used by the SDK configuration.
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
              component_name(),
              left_indices_.size(), head_indices_.size(),
              right_indices_.size());

  auto log_indices = [&](const char* label, const std::vector<size_t>& idx) {
    for (size_t i : idx)
      RCLCPP_INFO(logger_, "[%s]   %s: %s", component_name(), label, info_.joints[i].name.c_str());
  };
  log_indices("left",  left_indices_);
  log_indices("head",  head_indices_);
  log_indices("right", right_indices_);

  // ---- Optional sim flags ----
  auto read_bool = [&](const char* key, bool def) -> bool {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty()) 
      return def;
    const std::string& v = it->second;
    return (v == "true" || v == "True" || v == "1");
  };
  sim_left_  = read_bool("sim_left",  false);
  sim_right_ = read_bool("sim_right", false);

  RCLCPP_INFO(logger_, "[%s] sim_left=%d, sim_right=%d",
              component_name(),
              static_cast<int>(sim_left_), 
              static_cast<int>(sim_right_));

  auto read_u16 = [&](const char* key, uint16_t def) -> uint16_t {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty()) 
      return def;
    try 
    { 
      return static_cast<uint16_t>(std::stoi(it->second)); 
    }
    catch (...) 
    { 
      return def; 
    }
  };

  trajectory_mode_.store(static_cast<uint8_t>(read_u16("trajectory_mode", 2)));
  trajectory_radio_.store(read_u16("trajectory_radio", 500));

  RCLCPP_INFO(logger_, "[%s] initial trajectory_mode=%u, radio=%u",
              component_name(),
              static_cast<unsigned>(trajectory_mode_.load()),
              static_cast<unsigned>(trajectory_radio_.load()));

  start_tuner_node();


  start_csv_writer();

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_configure_impl()
{
  constexpr int      timeout_ms  = 2000;   // 每组最多等 2 秒
  constexpr int      poll_ms     = 20;     // 轮询间隔
  constexpr double   zero_eps    = 1e-6;

  std::vector<float> left_now;
  std::vector<float> head_now;
  std::vector<float> right_now;

  auto wait_for_joint_state = [&](Group group, std::vector<float>& joints, int timeout_ms) -> bool {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
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

  // ---- Left arm ----
  if (!left_indices_.empty())
  {
    if (!wait_for_joint_state(Group::LEFT_ARM, left_now, timeout_ms))
    {
      RCLCPP_WARN(logger_, "[%s] no LEFT_ARM state within %d ms", component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }

    if (left_now.size() != left_indices_.size())
    {
      RCLCPP_ERROR(logger_, "[%s] LEFT_ARM size mismatch: got %zu, expected %zu",
        component_name(), left_now.size(), left_indices_.size());
      return CallbackReturn::ERROR;
    }
  }
  // ---- Head ----
  if (!head_indices_.empty())
  {
    if (!wait_for_joint_state(Group::HEAD, head_now, timeout_ms))
    {
      RCLCPP_WARN(logger_, "[%s] no HEAD state within %d ms", component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }

    if (head_now.size() != head_indices_.size())
    {
      RCLCPP_ERROR(logger_, "[%s] HEAD size mismatch: got %zu, expected %zu",
        component_name(), head_now.size(), head_indices_.size());
      return CallbackReturn::ERROR;
    }
  }

  // ---- Right arm ----
  if (!right_indices_.empty())
  {
    if (!wait_for_joint_state(Group::RIGHT_ARM, right_now, timeout_ms))
    {
      RCLCPP_WARN(logger_, "[%s] no RIGHT_ARM state within %d ms", component_name(), timeout_ms);
      return CallbackReturn::ERROR;
    }
    if (right_now.size() != right_indices_.size())
    {
      RCLCPP_ERROR(logger_, "[%s] RIGHT_ARM size mismatch: got %zu, expected %zu",
        component_name(), right_now.size(), right_indices_.size());
      return CallbackReturn::ERROR;
    }
  }

  // ---- write to state and command  ----
  auto seed = [&](const std::vector<size_t>& idx, const std::vector<float>& vals)
  {
    for (size_t k = 0; k < idx.size(); ++k)
    {
      hw_position_states_  [idx[k]] = vals[k];
      hw_position_commands_[idx[k]] = vals[k];
    }
  };
  seed(left_indices_,  left_now);
  seed(head_indices_,  head_now);
  seed(right_indices_, right_now);

  auto all_zero = [](const std::vector<float>& v, double eps) 
  {
    for (float x : v)
      if (std::abs(x) > eps) 
        return false;
    return true;
  };

  RCLCPP_INFO(logger_, "[%s] initial state captured: left=%zu head=%zu right=%zu",
    component_name(), left_now.size(), head_now.size(), right_now.size());

  if ((left_now .empty() || all_zero(left_now,  zero_eps)) &&
      (head_now .empty() || all_zero(head_now,  zero_eps)) &&
      (right_now.empty() || all_zero(right_now, zero_eps)))
  {
    RCLCPP_WARN(logger_,
      "[%s] all initial states are ZERO — rt_control may not be "
      "simulating / connected, or it is at startup.",
      component_name());
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

  const RetCode ret_left = api_->set_group(Group::LEFT_ARM, true);
  if (ret_left != RetCode::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[%s] set_group(LEFT_ARM, true) failed: %d",
                 component_name(), static_cast<int>(ret_left));
    api_->set_group(Group::LEFT_ARM, false);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "[%s] set_group(LEFT_ARM, true) OK", component_name());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const RetCode ret_right = api_->set_group(Group::RIGHT_ARM, true);
  if (ret_right != RetCode::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[%s] set_group(RIGHT_ARM, true) failed: %d",
                 component_name(), static_cast<int>(ret_right));
    api_->set_group(Group::RIGHT_ARM, false);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "[%s] set_group(RIGHT_ARM, true) OK", component_name());

  RCLCPP_INFO(logger_, "[%s] on_activate_impl done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_deactivate_impl()
{
  const RetCode ret_left = api_->set_group(Group::LEFT_ARM, false);
  if (ret_left == RetCode::SUCCESS)
  {
    RCLCPP_INFO(logger_, "[%s] set_group(LEFT_ARM, false) OK", component_name());
  }
  else
  {
    RCLCPP_WARN(logger_, "[%s] set_group(LEFT, false) failed: %d", component_name(), static_cast<int>(ret_left));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const RetCode ret_right = api_->set_group(Group::RIGHT_ARM, false);
  if (ret_right == RetCode::SUCCESS)
  {
    RCLCPP_INFO(logger_, "[%s] set_group(RIGHT_ARM, false) OK", component_name());
  }
  else
  {
    RCLCPP_WARN(logger_, "[%s] set_group(RIGHT, false) failed: %d",
                component_name(), static_cast<int>(ret_right));
  }

  // const std::vector<float> speed(7, 1.0471975512);
  // if (api_->set_joint_max_speed(Group::LEFT_ARM, speed).size() != speed.size()) 
  // {
  //   RCLCPP_WARN(logger_, "set_joint_max_speed(LEFT failed");
  //   return CallbackReturn::ERROR;
  // }
  // if (api_->set_joint_max_speed(Group::RIGHT_ARM, speed).size() != speed.size()) 
  // {
  //   RCLCPP_WARN(logger_, "set_joint_max_speed(RIGHT failed");
  //   return CallbackReturn::ERROR;
  // }

  RCLCPP_INFO(logger_, "[%s] on_deactivate_impl done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_cleanup_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_cleanup_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_cleanup_impl", component_name());

  stop_csv_writer();
  stop_tuner_node();

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_shutdown_impl
// ============================================================
CallbackReturn DualArmsHardwareInterface::on_shutdown_impl()
{
  RCLCPP_INFO(logger_, "[%s] on_shutdown_impl", component_name());

  stop_csv_writer();
  stop_tuner_node();

  return CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface> DualArmsHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(info_.joints.size() * 3);

  for (const auto& joint : info_.joints)
  {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end()) continue;
    const size_t idx = it->second;

    for (const auto& s : joint.state_interfaces)
    {
      if (s.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_VELOCITY)
        out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[idx]);
      else if (s.name == hardware_interface::HW_IF_EFFORT)
        out.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[idx]);
    }
  }
  return out;
}

// ============================================================
// export_command_interfaces
// ============================================================
std::vector<hardware_interface::CommandInterface> DualArmsHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(info_.joints.size());

  for (const auto& joint : info_.joints)
  {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end()) 
      continue;
    const size_t idx = it->second;

    for (const auto& c : joint.command_interfaces)
    {
      if (c.name == hardware_interface::HW_IF_POSITION)
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[idx]);
    }
  }
  return out;
}

// ============================================================
// read
// ============================================================
hardware_interface::return_type DualArmsHardwareInterface::read(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!api_)
  {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const size_t expected = left_indices_.size() + head_indices_.size() + right_indices_.size();

  auto [ret_p, positions] = api_->get_joint(Group::ALL);
  if (ret_p == RetCode::SUCCESS && positions.size() == expected)
  {
    size_t off = 0;
    for (size_t i : left_indices_)
    {
      rt_control_states_[i] = positions[off];
      hw_position_states_[i] = positions[off];
      ++off;
    }
    for (size_t i : head_indices_)
    {
      rt_control_states_[i] = positions[off];
      hw_position_states_[i] = positions[off];
      ++off;
    }
    for (size_t i : right_indices_)
    {
      rt_control_states_[i] = positions[off];
      hw_position_states_[i] = positions[off];
      ++off;
    }
  }

  // ---- Step 2 ----
  if (sim_left_)
  {
    for (size_t i : left_indices_)
      hw_position_states_[i] = hw_position_commands_[i];
  }
  if (sim_right_)
  {
    for (size_t i : right_indices_)
      hw_position_states_[i] = hw_position_commands_[i];
  }

  // ---- Step 3: velocity ----
  auto [ret_v, velocities] = api_->get_joint_velocity(Group::ALL);
  if (ret_v == RetCode::SUCCESS && velocities.size() == expected)
  {
    size_t off = 0;
    for (size_t i : left_indices_)  
      hw_velocity_states_[i] = velocities[off++];
    for (size_t i : head_indices_)  
      hw_velocity_states_[i] = velocities[off++];
    for (size_t i : right_indices_) 
      hw_velocity_states_[i] = velocities[off++];
  }

  if (sim_left_)
    for (size_t i : left_indices_)  
      hw_velocity_states_[i] = 0.0;

  if (sim_right_)
    for (size_t i : right_indices_) 
      hw_velocity_states_[i] = 0.0;

  // ---- Step 4: effort ----
  // for (size_t i = 0; i < hw_effort_states_.size(); ++i)
  //   hw_effort_states_[i] = 0.0;
  
  // ---- Step 5 ----
  if (++debug_read_tick_ >= debug_every_n_)
  {
    debug_read_tick_ = 0;

    auto to_str_by_indices = [&](const std::vector<size_t>& idx) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4) << "[";
      for (size_t k = 0; k < idx.size(); ++k)
      {
        oss << hw_position_states_[idx[k]];
        if (k + 1 < idx.size()) 
          oss << ", ";
      }
      oss << "]";
      return oss.str();
    };

    RCLCPP_INFO(logger_, "[%s] read:\n  left:  %s\n  head:  %s\n  right: %s",
                component_name(),
                to_str_by_indices(left_indices_ ).c_str(),
                to_str_by_indices(head_indices_ ).c_str(),
                to_str_by_indices(right_indices_).c_str());
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write
// ============================================================
hardware_interface::return_type DualArmsHardwareInterface::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& period)
{
  if (!api_ || !is_active())
  {
    return hardware_interface::return_type::OK;
  }

  static thread_local std::chrono::steady_clock::time_point prev_entry{};
  const auto t_entry = std::chrono::steady_clock::now();

  double actual_dt_ms = 0.0;
  if (prev_entry.time_since_epoch().count() != 0)
  {
    actual_dt_ms = std::chrono::duration<double, std::milli>(t_entry - prev_entry).count();
  }
  prev_entry = t_entry;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t expected = left_indices_.size() + head_indices_.size() + right_indices_.size();
    cmd_buffer_.resize(expected);

    size_t k = 0;

    for (size_t i : left_indices_)
    {
      cmd_buffer_[k++] = sim_left_ 
        ? static_cast<float>(rt_control_states_[i])
        : static_cast<float>(hw_position_commands_[i]);
    }

    for (size_t i : head_indices_)
    {
      cmd_buffer_[k++] = static_cast<float>(hw_position_commands_[i]);
    }

    for (size_t i : right_indices_)
    {
      cmd_buffer_[k++] = sim_right_
        ? static_cast<float>(rt_control_states_[i])
        : static_cast<float>(hw_position_commands_[i]);
    }
  }

  if (delta_pub_)
  {
    if (++delta_sub_check_tick_ >= 100)
    {
      delta_sub_check_tick_ = 0;
      delta_has_sub_ = (delta_pub_->get_subscription_count() > 0);
    }

    if (delta_has_sub_)
    {
      auto msg = std_msgs::msg::Float32MultiArray();
      const size_t n = cmd_buffer_.size();
      msg.data.resize(n);
      for (size_t k = 0; k < n; ++k)
      {
        msg.data[k] = cmd_buffer_[k] - last_cmd_buffer_[k];
      }
      delta_pub_->publish(msg);
    }
  }

  const uint8_t  mode  = trajectory_mode_.load();
  const uint16_t radio = trajectory_radio_.load();

  if (++debug_write_tick_ >= debug_every_n_)
  {
    debug_write_tick_ = 0;
    char buf_l[256], buf_h[256], buf_r[256];

    auto write_arr = [](char* buf, size_t buflen, const std::vector<float>& v,
                        size_t from, size_t count) {
      size_t pos = 0;
      pos += std::snprintf(buf + pos, buflen - pos, "[");
      for (size_t k = 0; k < count && pos < buflen - 16; ++k)
      {
        pos += std::snprintf(buf + pos, buflen - pos, "%s%.4f", (k ? ", " : ""), v[from + k]);
      }
      std::snprintf(buf + pos, buflen - pos, "]");
    };

    const size_t n_left  = left_indices_.size();
    const size_t n_head  = head_indices_.size();
    const size_t n_right = right_indices_.size();
    const size_t off_left  = 0;
    const size_t off_head  = off_left + n_left;
    const size_t off_right = off_head + n_head;

    write_arr(buf_l, sizeof(buf_l), cmd_buffer_, off_left,  n_left );
    write_arr(buf_h, sizeof(buf_h), cmd_buffer_, off_head,  n_head );
    write_arr(buf_r, sizeof(buf_r), cmd_buffer_, off_right, n_right);

    RCLCPP_INFO(logger_,
      "[%s] write:\n  left:  %s\n  head:  %s\n  right: %s mode: %u radio: %u",
      component_name(), buf_l, buf_h, buf_r,
      static_cast<unsigned>(mode), static_cast<unsigned>(radio));
  }

  const auto t0 = std::chrono::steady_clock::now();
  const RetCode ret = api_->move_joint(Group::ALL, cmd_buffer_, true, mode, radio);
  const auto t1 = std::chrono::steady_clock::now();

  if (ret != RetCode::SUCCESS)
  {
    ++error_streak_;
    if (error_streak_ == 1 || error_streak_ % 100 == 0)
    {
      RCLCPP_WARN(logger_, "[%s] move_joint(ALL) failed: %d (streak=%d)", component_name(), static_cast<int>(ret), error_streak_);
    }
  }
  else
  {
    error_streak_ = 0;
  }

  const double call_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  const double period_ms    = period.seconds() * 1000.0;
  const double threshold_ms = period_ms * 1.5;
  if (timing_pub_)
  {
    if (++timing_sub_check_tick_ >= 100)
    {
      timing_sub_check_tick_ = 0;
      timing_has_sub_ = (timing_pub_->get_subscription_count() > 0);
    }

    if (timing_has_sub_)
    {
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
    if (dt_filled < dt_history.size()) 
      ++dt_filled;

    if (dt_idx >= dt_history.size())
    {
      dt_idx = 0;

      // ---- 计算统计量 ----
      double dt_sum = 0.0, dt_sum2 = 0.0, dt_max = 0.0, dt_min = 1e9;
      double cl_sum = 0.0, cl_max = 0.0;

      const size_t n = dt_filled;

      for (size_t i = 0; i < n; ++i)
      {
        const double d = dt_history[i];
        const double c = call_history[i];

        dt_sum  += d;
        dt_sum2 += d * d;
        if (d > dt_max) dt_max = d;
        if (d < dt_min) dt_min = d;

        cl_sum += c;
        if (c > cl_max) cl_max = c;
      }

      const double dt_mean = dt_sum / n;
      const double dt_var  = std::max(0.0, dt_sum2 / n - dt_mean * dt_mean);
      const double dt_sd   = std::sqrt(dt_var);
      const double cl_mean = cl_sum / n;

      // ---- 判断健康度 ----
      // 均值偏离 period > 15%，或标准差 > 10% period，或 max > 2× period → 告警
      const bool mean_drift = std::abs(dt_mean - period_ms) > (period_ms * 0.15);
      const bool high_sd    = dt_sd > (period_ms * 0.10);
      const bool tail_spike = dt_max > (period_ms * 2.0);

      if (mean_drift || high_sd || tail_spike)
      {
        RCLCPP_WARN(logger_,
          "[%s] write dt stats (N=%zu): "
          "mean=%.2f sd=%.2f min=%.2f max=%.2f ms (period=%.2f) | "
          "call mean=%.2f max=%.2f ms%s%s%s",
          component_name(), n,
          dt_mean, dt_sd, dt_min, dt_max, period_ms,
          cl_mean, cl_max,
          mean_drift ? " [mean_drift]" : "",
          high_sd    ? " [high_sd]" : "",
          tail_spike ? " [tail_spike]" : "");
      }
      else
      {
        RCLCPP_DEBUG(logger_,
          "[%s] write dt stats (N=%zu): "
          "mean=%.2f sd=%.2f min=%.2f max=%.2f ms (period=%.2f) | "
          "call mean=%.2f max=%.2f ms",
          component_name(), n,
          dt_mean, dt_sd, dt_min, dt_max, period_ms,
          cl_mean, cl_max);
      }
    }
  }

  if (csv_is_open_.load(std::memory_order_acquire))
  {
    CsvRow row{};
    row.t_s = std::chrono::duration<double>(t1.time_since_epoch()).count();

    const size_t n_left  = left_indices_.size();
    const size_t n_right = right_indices_.size();
    const size_t off_right = n_left + head_indices_.size();

    if (n_left == 7 && n_right == 7)
    {
      std::memcpy(row.left,  cmd_buffer_.data() + 0,         7 * sizeof(float));
      std::memcpy(row.right, cmd_buffer_.data() + off_right, 7 * sizeof(float));
    }
    else                               // 慢路径（可变关节数）
    {
      for (size_t k = 0; k < 7; ++k)
        row.left[k] = (k < n_left) ? cmd_buffer_[k] : std::numeric_limits<float>::quiet_NaN();
      for (size_t k = 0; k < 7; ++k)
        row.right[k] = (k < n_right) ? cmd_buffer_[off_right + k] : std::numeric_limits<float>::quiet_NaN();
    }

    row.follow = 1;   // follow=true 固定
    row.mode   = mode;
    row.radio  = radio;

    if (!csv_queue_.push(row))
    {
      csv_dropped_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  std::swap(cmd_buffer_, last_cmd_buffer_);

  return hardware_interface::return_type::OK;
}

void DualArmsHardwareInterface::start_tuner_node()
{
  if (tuner_node_) 
    return;

  const std::string node_name = std::string(get_name()) + "_tuner";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + node_name});
  options.parameter_overrides({});
  options.use_global_arguments(true);

  tuner_node_ = std::make_shared<rclcpp::Node>(node_name, options);

  const std::string ns = "/" + std::string(get_name());
  const std::string tuner_topic = ns + "/trajectory_tuner";
  const std::string delta_topic = ns + "/cmd_delta";
  const std::string timing_topic = ns + "/write_timing";
  const std::string csv_topic = ns + "/csv_enable";

  delta_pub_ = tuner_node_->create_publisher<std_msgs::msg::Float32MultiArray>(delta_topic, 1);
  timing_pub_ = tuner_node_->create_publisher<std_msgs::msg::Float32MultiArray>(timing_topic, 1);
  tuner_sub_ = tuner_node_->create_subscription<std_msgs::msg::Int16MultiArray>(
    tuner_topic, 
    1,
    [this](std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
      // [mode, radio]
      if (msg->data.size() < 2)
      {
        RCLCPP_WARN(logger_,
          "[%s] trajectory_tuner: expected [mode, radio], got %zu element(s)",
          component_name(), msg->data.size());
        return;
      }

      // ---- mode: 0..2 ----
      const int16_t raw_mode = msg->data[0];
      if (raw_mode < 0 || raw_mode > 2)
      {
        RCLCPP_WARN(logger_,
          "[%s] trajectory_mode=%d out of range (valid 0,1,2); ignoring",
          component_name(), static_cast<int>(raw_mode));
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
        component_name(),
        static_cast<unsigned>(trajectory_mode_.load()),
        static_cast<unsigned>(trajectory_radio_.load()));
    });

  csv_enable_sub_ = tuner_node_->create_subscription<std_msgs::msg::Bool>(
    csv_topic, 
    1,
    [this](std_msgs::msg::Bool::SharedPtr msg)
    {
      csv_requested_.store(msg->data);
      RCLCPP_INFO(logger_, "[%s] csv_enable -> %s", component_name(), msg->data ? "true" : "false");
    });
 
  RCLCPP_INFO(logger_, "[%s] write_timing publisher on %s (layout: [call_ms, period_ms, threshold_ms])",
    component_name(), timing_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] cmd_delta publisher on %s", component_name(), delta_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] csv_enable subscriber on %s", component_name(), csv_topic.c_str());

  tuner_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  tuner_executor_->add_node(tuner_node_);
  tuner_running_.store(true);

  tuner_thread_ = std::thread([this]() {
    while (rclcpp::ok() && tuner_running_.load())
    {
      tuner_executor_->spin_once(std::chrono::milliseconds(10));
    }
  });

  RCLCPP_INFO(logger_, "[%s] tuner node '%s' started (topic: %s, format: [mode, radio])",
    component_name(), node_name.c_str(), tuner_topic.c_str());
}

void DualArmsHardwareInterface::stop_tuner_node()
{
  if (!tuner_node_) 
    return;

  tuner_running_.store(false);

  if (tuner_executor_) 
  {
    tuner_executor_->cancel();
  }

  if (tuner_thread_.joinable()) 
  {
    tuner_thread_.join();
  }

  timing_pub_.reset();
  delta_pub_.reset();
  tuner_sub_.reset(); 
  tuner_executor_.reset();
  tuner_node_.reset();

  RCLCPP_INFO(logger_, "[%s] tuner node stopped", component_name());
}

// ============================================================
// CSV writer Thread
// ============================================================
void DualArmsHardwareInterface::start_csv_writer()
{
  if (csv_writer_thread_.joinable())
  {
    RCLCPP_WARN(logger_, "[%s] CSV writer already running", component_name());
    return;
  }

  csv_writer_running_.store(true, std::memory_order_release);

  try
  {
    csv_writer_thread_ = std::thread(&DualArmsHardwareInterface::csv_writer_loop, this);
  }
  catch (const std::exception& e)
  {
    csv_writer_running_.store(false, std::memory_order_release);
    RCLCPP_ERROR(logger_, "[%s] failed to start CSV writer thread: %s", component_name(), e.what());
    throw;   // 让 on_init 失败，避免"以为启动了实际没启"的隐性错误
  }

  RCLCPP_INFO(logger_, "[%s] CSV writer thread started (queue capacity=%zu)",
    component_name(), kCsvQueueCapacity);
}

void DualArmsHardwareInterface::stop_csv_writer()
{
  if (!csv_writer_thread_.joinable())
  {
    return;
  }

  csv_writer_running_.store(false, std::memory_order_release);

  csv_writer_thread_.join();

  csv_is_open_.store(false, std::memory_order_release);
  csv_requested_.store(false, std::memory_order_release);

  RCLCPP_INFO(logger_,
              "[%s] CSV writer thread stopped (total dropped=%lu)",
              component_name(),
              static_cast<unsigned long>(csv_dropped_.load()));
}

void DualArmsHardwareInterface::csv_writer_loop()
{
  std::ofstream file;
  bool          file_open = false;
  std::string   path;
  std::size_t   row_idx = 0;
  double        session_start_s = 0.0;
  std::size_t   writes_since_flush = 0;

  constexpr std::size_t kFlushEvery = 500;   // 每 500 行 flush 一次
  constexpr int         kBatchMax   = 200;   // 每轮最多消费 200 行
  constexpr int         kPollMs     = 2;     // 轮询间隔

  CsvRow row{};

  while (csv_writer_running_.load(std::memory_order_acquire))
  {
    try
    {
      const bool want = csv_requested_.load(std::memory_order_acquire);

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

        // 4. 打开文件
        file.open(path, std::ios::out | std::ios::trunc);

        if (!file.is_open())
        {
          RCLCPP_ERROR(logger_, "[%s] cannot open CSV: %s", component_name(), path.c_str());
          csv_requested_.store(false, std::memory_order_release);
        }
        else
        {
          file << "idx,timestamp_s";
          for (int j = 1; j <= 7; ++j) 
            file << ",left_j"  << j;
          for (int j = 1; j <= 7; ++j) 
            file << ",right_j" << j;
          file << ",follow,mode,radio\n";
          file << std::fixed << std::setprecision(6);

          file.flush();

          if (file.fail())
          {
            RCLCPP_ERROR(logger_, "[%s] CSV header flush failed: %s", component_name(), path.c_str());
            file.close();
            csv_requested_.store(false, std::memory_order_release);
          }
          else
          {
            row_idx = 0;
            writes_since_flush = 0;
            session_start_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            file_open = true;

            csv_path_ = path;
            csv_dropped_.store(0, std::memory_order_relaxed);

            csv_is_open_.store(true, std::memory_order_release);

            RCLCPP_INFO(logger_, "[%s] CSV log STARTED: %s", component_name(), path.c_str());
          }
        }
      }

      // ------------------------------------------------------------
      // 关闭会话
      // ------------------------------------------------------------
      else if (!want && file_open)
      {
        csv_is_open_.store(false, std::memory_order_release);

        while (csv_queue_.pop(row))
        {
          const double ts_s = row.t_s - session_start_s;

          file << row_idx++ << ',' << ts_s;
          for (int k = 0; k < 7; ++k) file << ',' << row.left[k];
          for (int k = 0; k < 7; ++k) file << ',' << row.right[k];
          file << ',' << static_cast<int>(row.follow)
               << ',' << static_cast<int>(row.mode)
               << ',' << static_cast<int>(row.radio)
               << '\n';
        }

        file.flush();
        const bool write_ok = !file.fail();
        file.close();
        file_open = false;

        if (!write_ok)
        {
          RCLCPP_ERROR(logger_, "[%s] CSV flush failed on close: %s (%zu rows written)",
            component_name(), path.c_str(), row_idx);
        }

        RCLCPP_INFO(logger_, "[%s] CSV log STOPPED: %s (%zu rows, %lu dropped)",
          component_name(), path.c_str(), row_idx,
          static_cast<unsigned long>(csv_dropped_.load(std::memory_order_relaxed)));
      }

      if (file_open)
      {
        int processed = 0;
        while (processed < kBatchMax && csv_queue_.pop(row))
        {
          const double ts_s = row.t_s - session_start_s;

          file << row_idx++ << ',' << ts_s;
          for (int k = 0; k < 7; ++k) 
            file << ',' << row.left[k];
          for (int k = 0; k < 7; ++k) 
            file << ',' << row.right[k];
          file << ',' << static_cast<int>(row.follow)
               << ',' << static_cast<int>(row.mode)
               << ',' << static_cast<int>(row.radio)
               << '\n';

          ++processed;
          ++writes_since_flush;

          // 定期 flush
          if (writes_since_flush >= kFlushEvery)
          {
            file.flush();
            writes_since_flush = 0;

            if (file.fail())
            {
              RCLCPP_ERROR(logger_, "[%s] CSV write failed after %zu rows; closing session",
                component_name(), row_idx);
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
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "[%s] CSV writer exception: %s", component_name(), e.what());

      csv_is_open_.store(false, std::memory_order_release);
      if (file_open)
      {
        try { file.flush(); } catch (...) {}
        try { file.close(); } catch (...) {}
        file_open = false;
      }
      csv_requested_.store(false, std::memory_order_release);
    }
    catch (...)
    {
      RCLCPP_ERROR(logger_, "[%s] CSV writer unknown exception", component_name());

      csv_is_open_.store(false, std::memory_order_release);
      if (file_open)
      {
        try { file.flush(); } catch (...) {}
        try { file.close(); } catch (...) {}
        file_open = false;
      }
      csv_requested_.store(false, std::memory_order_release);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  try
  {
    if (file_open)
    {
      csv_is_open_.store(false, std::memory_order_release);

      while (csv_queue_.pop(row))
      {
        const double ts_s = row.t_s - session_start_s;

        file << row_idx++ << ',' << ts_s;
        for (int k = 0; k < 7; ++k) 
          file << ',' << row.left[k];
        for (int k = 0; k < 7; ++k) 
          file << ',' << row.right[k];
        file << ',' << static_cast<int>(row.follow)
             << ',' << static_cast<int>(row.mode)
             << ',' << static_cast<int>(row.radio)
             << '\n';
      }

      file.flush();
      const bool write_ok = !file.fail();
      file.close();
      file_open = false;

      if (!write_ok)
      {
        RCLCPP_ERROR(logger_, "[%s] CSV flush failed on shutdown: %s (%zu rows)",
          component_name(), path.c_str(), row_idx);
      }
      else
      {
        RCLCPP_INFO(logger_, "[%s] CSV log closed on shutdown: %s (%zu rows)",
          component_name(), path.c_str(), row_idx);
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown exception: %s", component_name(), e.what());
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown unknown exception", component_name());
  }
}

}  // namespace dual_arm_hardware_interface

// ============================================================
// pluginlib 
// ============================================================
PLUGINLIB_EXPORT_CLASS(
  dual_arm_hardware_interface::DualArmsHardwareInterface,
  hardware_interface::SystemInterface)