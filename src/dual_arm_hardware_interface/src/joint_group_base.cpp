#include "dual_arm_hardware_interface/joint_group_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <thread>

#include "dual_arm_hardware_interface/control_api_manager.hpp"

namespace dual_arm_hardware_interface
{

using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using dual_arm_v2_2_sdk::State;

// ============================================================
// on_init_impl —— 建索引 + 分配缓冲区（必须在 export_*_interfaces 之前）
// ============================================================
CallbackReturn JointGroupBase::on_init_impl()
{
  const size_t n = info_.joints.size();

  arm_indices_.clear();
  arm_indices_.reserve(n);
  joint_indices_.clear();
  joint_indices_.reserve(n);

  hw_position_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_    .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());
  cmd_buffer_          .assign(n, 0.0F);
  enable_states_       .assign(n, 0);
  error_states_        .assign(n, 0);

  for (size_t i = 0; i < n; ++i)
  {
    const auto& joint = info_.joints[i];
    joint_indices_[joint.name] = i;
    arm_indices_.push_back(i);
    RCLCPP_INFO(logger_, "[%s] joint[%zu] = %s", component_name(), i, joint.name.c_str());
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure_impl —— 等待初值，避免激活瞬间跳到 0
// ============================================================
CallbackReturn JointGroupBase::on_configure_impl()
{
  // 轮询 get_joint 直到拿到有效反馈
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(configure_timeout_sec_);

  bool got_state = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    auto [code, positions] = api_->get_joint(group());

    RCLCPP_INFO(logger_, "[%s] get_joint ret=%d, size=%zu (expected=%zu)",
                component_name(), static_cast<int>(code),
                positions.size(), arm_indices_.size());

    if (code == RetCode::SUCCESS && positions.size() == arm_indices_.size())
    {
      for (size_t k = 0; k < positions.size(); ++k)
      {
        const size_t idx = arm_indices_[k];
        hw_position_states_  [idx] = positions[k];
        hw_position_commands_[idx] = positions[k];
      }
      got_state = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!got_state)
  {
    RCLCPP_WARN(logger_,
                "[%s] no initial state within %d s; read() will keep trying",
                component_name(), configure_timeout_sec_);
    // 不返回 ERROR —— 让 lifecycle 继续，read() 里会不断重试
  }
  else
  {
    RCLCPP_INFO(logger_, "[%s] initial state captured", component_name());
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate_impl —— 使能 group
// ============================================================
CallbackReturn JointGroupBase::on_activate_impl()
{
  const RetCode ret = api_->set_group(group(), true);
  if (ret != RetCode::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[%s] set_group(true) failed: %d", component_name(), static_cast<int>(ret));
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate_impl —— 下使能 group
// ============================================================
CallbackReturn JointGroupBase::on_deactivate_impl()
{
  const RetCode ret = api_->set_group(group(), false);
  if (ret != RetCode::SUCCESS)
  {
    RCLCPP_WARN(logger_, "[%s] set_group(false) failed: %d", component_name(), static_cast<int>(ret));
  }
  return CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface>
JointGroupBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(info_.joints.size() * 3);

  for (const auto& joint : info_.joints)
  {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end())
    {
      RCLCPP_ERROR(logger_, "[%s] joint '%s' not registered", component_name(), joint.name.c_str());
      continue;
    }
    const size_t idx = it->second;

    for (const auto& state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &hw_position_states_[idx]);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[idx]);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        out.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[idx]);
      }
    }
  }
  return out;
}

// ============================================================
// export_command_interfaces
// ============================================================
std::vector<hardware_interface::CommandInterface>
JointGroupBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(info_.joints.size());

  for (const auto& joint : info_.joints)
  {
    auto it = joint_indices_.find(joint.name);
    if (it == joint_indices_.end()) continue;
    const size_t idx = it->second;

    for (const auto& cmd_if : joint.command_interfaces)
    {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION)
      {
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                         &hw_position_commands_[idx]);
      }
      // 目前不支持 effort 命令接口。
      // 如需支持，请实现 SDK 侧的力矩 API 后在此添加分支。
    }
  }
  return out;
}

// ============================================================
// read
// ============================================================
hardware_interface::return_type JointGroupBase::read(
    const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!api_)
  {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // ---- position ----
  {
    auto [ret, positions] = api_->get_joint(group());
    if (ret == RetCode::SUCCESS)
    {
      const size_t m = std::min(positions.size(), arm_indices_.size());
      for (size_t k = 0; k < m; ++k)
      {
        hw_position_states_[arm_indices_[k]] = positions[k];
      }
    }

    if (++debug_read_tick_ >= debug_every_n_)
    {
      debug_read_tick_ = 0;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4) << "[";
      for (size_t k = 0; k < positions.size(); ++k)
      {
        oss << positions[k];
        if (k + 1 < positions.size()) oss << ", ";
      }
      oss << "]";

      RCLCPP_INFO(logger_, "[%s] read: ret=%d, size=%zu/%zu, pos=%s",
                  component_name(), static_cast<int>(ret),
                  positions.size(), arm_indices_.size(),
                  oss.str().c_str());
    }
  }

  // ---- velocity ----
  {
    auto [ret, velocities] = api_->get_joint_velocity(group());
    if (ret == RetCode::SUCCESS)
    {
      const size_t m = std::min(velocities.size(), arm_indices_.size());
      for (size_t k = 0; k < m; ++k)
      {
        hw_velocity_states_[arm_indices_[k]] = velocities[k];
      }
    }
  }

  // ---- effort：SDK 暂不提供，统一置 0 ----
  // for (size_t idx : arm_indices_)
  // {
  //   hw_effort_states_[idx] = 0.0;
  // }

  // ---- enable / error 状态（仅内部诊断用） ----
  {
    auto [ret, states] = api_->get_group_state(group());
    if (ret == RetCode::SUCCESS)
    {
      const size_t m = std::min(states.size(), arm_indices_.size());
      for (size_t k = 0; k < m; ++k)
      {
        const size_t idx = arm_indices_[k];
        enable_states_[idx] = (states[k] == State::OPERATIONAL) ? 1 : 0;
        error_states_ [idx] = (states[k] == State::ERROR)       ? 1 : 0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write
// ============================================================
hardware_interface::return_type JointGroupBase::write(
  const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  // if (!api_ || !is_active())
  // {
  //   return hardware_interface::return_type::OK;
  // }

  // std::lock_guard<std::mutex> lock(mutex_);

  // for (size_t k = 0; k < arm_indices_.size(); ++k)
  // {
  //   cmd_buffer_[k] = static_cast<float>(hw_position_commands_[arm_indices_[k]]);
  // }

  // const RetCode ret = api_->move_joint(group(), cmd_buffer_,
  //                                     /*follow=*/true,
  //                                     /*trajectory_mode=*/2,
  //                                     /*radio=*/999);

  // if (++debug_write_tick_ >= debug_every_n_)
  // {
  //   debug_write_tick_ = 0;

  //   std::ostringstream oss;
  //   oss << std::fixed << std::setprecision(4) << "[";
  //   for (size_t k = 0; k < cmd_buffer_.size(); ++k)
  //   {
  //     oss << cmd_buffer_[k];
  //     if (k + 1 < cmd_buffer_.size()) oss << ", ";
  //   }
  //   oss << "]";

  //   RCLCPP_INFO(logger_, "[%s] write: ret=%d, size=%zu/%zu, pos=%s",
  //               component_name(), static_cast<int>(ret),
  //               cmd_buffer_.size(), arm_indices_.size(),
  //               oss.str().c_str());
  // }

  // if (ret != RetCode::SUCCESS)
  // {
  //   ++error_streak_;
  //   if (error_streak_ == 1 || error_streak_ % 100 == 0)
  //   {
  //     RCLCPP_WARN(logger_, "[%s] move_joint failed: %d (streak=%d)",
  //                 component_name(), static_cast<int>(ret), error_streak_);
  //   }
  // }
  // else
  // {
  //   error_streak_ = 0;
  // }

  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_hardware_interface