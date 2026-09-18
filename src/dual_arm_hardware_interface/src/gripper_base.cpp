#include "dual_arm_hardware_interface/gripper_base.hpp"

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

namespace {

// SDK 侧取值范围
constexpr uint16_t kSdkPositionMin = 0;
constexpr uint16_t kSdkPositionMax = 1000;

// 归一化常量
constexpr double kSdkPositionScale = static_cast<double>(kSdkPositionMax);

// hardware_parameters 默认值
constexpr uint16_t kDefaultSpeed = 500;
constexpr uint16_t kDefaultForce = 500;

inline uint16_t clamp_u16(uint16_t value, uint16_t lo, uint16_t hi)
{
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

}  // namespace

// ============================================================
// on_init_impl —— 校验关节数 + 读 speed/force
// ============================================================
CallbackReturn GripperBase::on_init_impl()
{
  if (info_.joints.empty())
  {
    RCLCPP_ERROR(logger_, "[%s] no joints declared", component_name());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() > 1)
    RCLCPP_WARN(logger_,
                "[%s] %zu joints declared; only '%s' will be used",
                component_name(), info_.joints.size(),
                info_.joints[0].name.c_str());

  joint_name_ = info_.joints[0].name;

  auto read_u16 = [&](const char* key, uint16_t def) -> uint16_t {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty())
      return def;
    try {
      const int v = std::stoi(it->second);
      return clamp_u16(static_cast<uint16_t>(std::max(0, v)), 1, 1000);
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "[%s] bad %s='%s': %s",
                  component_name(), key, it->second.c_str(), e.what());
      return def;
    }
  };

  speed_ = read_u16("gripper_speed", kDefaultSpeed);
  force_ = read_u16("gripper_force", kDefaultForce);

  RCLCPP_INFO(logger_, "[%s] joint='%s', speed=%u, force=%u",
              component_name(), joint_name_.c_str(), speed_, force_);
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure_impl —— 只做初值轮询
// ============================================================
CallbackReturn GripperBase::on_configure_impl()
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(configure_timeout_sec_);

  bool got_state = false;
  while (std::chrono::steady_clock::now() < deadline)
  {
    auto [code, state_map] = api_->get_gripper_state(group());
    if (code == RetCode::SUCCESS && state_map.count("position"))
    {
      const uint16_t sdk_pos = state_map.at("position");
      const double normalized = static_cast<double>(sdk_pos) / kSdkPositionScale;
      hw_position_state_   = normalized;
      hw_position_command_ = normalized;
      got_state = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!got_state)
    RCLCPP_WARN(logger_, "[%s] no initial state within %d s",
                component_name(), configure_timeout_sec_);
  else
    RCLCPP_INFO(logger_, "[%s] initial state captured", component_name());

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate_impl —— 应用 speed / force
// ============================================================
CallbackReturn GripperBase::on_activate_impl()
{
  // 注意：gripper 不单独做 set_group(true)。
  // 在 SDK 中，gripper 与它所属的 arm 共用一个 Group。如果这里调
  // set_group(LEFT_ARM, true)，会连带把左臂也一并使能，导致与
  // LeftArmHardwareInterface 的行为冲突。
  //
  // set_gripper_config 会触发 SDK 内部的 ensure_initialized()，
  // 完成一次 settings 刷新，之后 set_gripper_position 就能正常工作。
  const RetCode ret = api_->set_gripper_config(
      group(), speed_, force_, /*block=*/false, /*timeout=*/0);

  if (ret != RetCode::SUCCESS)
  {
    RCLCPP_ERROR(logger_,
                 "[%s] set_gripper_config(speed=%u, force=%u) failed: %d",
                 component_name(), speed_, force_,
                 static_cast<int>(ret));
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_,
              "[%s] gripper configured (speed=%u, force=%u)",
              component_name(), speed_, force_);
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate_impl —— 保持当前位置（不主动松开）
// ============================================================
CallbackReturn GripperBase::on_deactivate_impl()
{
  // 同样不调 set_group(false)：那会连带下使能整个 arm。
  // 这里只把当前命令位置刷新为当前位置，避免下次激活时突变。
  std::lock_guard<std::mutex> lock(mutex_);
  hw_position_command_ = hw_position_state_;

  RCLCPP_INFO(logger_, "[%s] gripper deactivated (holding position %.4f)",
              component_name(), hw_position_state_);
  return CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces
// ============================================================
std::vector<hardware_interface::StateInterface>
GripperBase::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(1);

  const auto& joint = info_.joints[0];
  for (const auto& state_if : joint.state_interfaces)
  {
    if (state_if.name == hardware_interface::HW_IF_POSITION)
    {
      out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                       &hw_position_state_);
    }
    // 夹爪通常不导出 velocity / effort
  }
  return out;
}

// ============================================================
// export_command_interfaces
// ============================================================
std::vector<hardware_interface::CommandInterface>
GripperBase::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(1);

  const auto& joint = info_.joints[0];
  for (const auto& cmd_if : joint.command_interfaces)
  {
    if (cmd_if.name == hardware_interface::HW_IF_POSITION)
    {
      out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                       &hw_position_command_);
    }
  }
  return out;
}

// ============================================================
// read —— 从 SDK 拉取夹爪状态
// ============================================================
hardware_interface::return_type GripperBase::read(
    const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!api_)
  {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto [code, state_map] = api_->get_gripper_state(group());
  if (code != RetCode::SUCCESS)
  {
    // 读失败时保留上一个有效值，不要写 NaN
    return hardware_interface::return_type::OK;
  }

  auto it = state_map.find("position");
  if (it == state_map.end())
  {
    return hardware_interface::return_type::OK;
  }

  const double normalized =
      static_cast<double>(it->second) / kSdkPositionScale;

  hw_position_state_ = normalized;

  return hardware_interface::return_type::OK;
}

// ============================================================
// write —— 下发夹爪位置命令
// ============================================================
hardware_interface::return_type GripperBase::write(
    const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
{
  if (!api_ || !is_active())
  {
    return hardware_interface::return_type::OK;
  }

  uint16_t sdk_position = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 归一化 [0, 1] → SDK [0, 1000]，并做 clamp
    const double clamped = std::clamp(hw_position_command_, 0.0, 1.0);
    sdk_position = static_cast<uint16_t>(
        std::lround(clamped * kSdkPositionScale));

    // 防御：显式 clamp 到 SDK 合法区间
    sdk_position = clamp_u16(sdk_position, kSdkPositionMin, kSdkPositionMax);
  }

  // block=false，绝不阻塞 controller_manager 的 100 Hz 周期
  const RetCode ret = api_->set_gripper_position(
      group(), sdk_position, /*block=*/false, /*timeout=*/0);

  if (ret != RetCode::SUCCESS)
  {
    ++error_streak_;
    if (error_streak_ == 1 || error_streak_ % 100 == 0)
    {
      RCLCPP_WARN(logger_,
                  "[%s] set_gripper_position(%u) failed: %d (streak=%d)",
                  component_name(), sdk_position,
                  static_cast<int>(ret), error_streak_);
    }
  }
  else
  {
    error_streak_ = 0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace dual_arm_hardware_interface