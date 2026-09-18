#ifndef DUAL_ARM_HARDWARE_INTERFACE__JOINT_GROUP_BASE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__JOINT_GROUP_BASE_HPP_

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/hardware_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Base class for hardware components that expose a group of joints
 *        controllable via ControlApi::move_joint() / get_joint().
 *
 * Typical users:
 *   - LeftArmHardwareInterface  (Group::LEFT_ARM)
 *   - RightArmHardwareInterface (Group::RIGHT_ARM)
 *   - HeadHardwareInterface     (Group::HEAD)
 *   - ElevatorHardwareInterface (Group::ELEVATOR)
 *
 * Behaviour:
 *   - on_configure_impl : polls get_joint(group) until a valid frame arrives,
 *                         seeds hw_position_commands_ with the measured
 *                         positions so that the first write() after activation
 *                         never commands a jump to zero.
 *   - on_activate_impl  : calls set_group(group, true)
 *   - on_deactivate_impl: calls set_group(group, false)
 *   - read              : position + velocity + group state from the SDK
 *   - write             : pushes position commands with follow=true so the
 *                         100 Hz controller_manager cycle never blocks.
 *
 * Derived classes only need to implement:
 *   - group()          : which Group this component drives
 *   - component_name() : inherited pure virtual from HardwareBase
 */
class JointGroupBase : public HardwareBase
{
public:
  JointGroupBase() = default;
  ~JointGroupBase() override = default;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

protected:
  // ---- 子类必须实现 ----
  virtual dual_arm_v2_2_sdk::Group group() const = 0;

  // ---- 生命周期钩子 ----
  CallbackReturn on_init_impl()       override;
  CallbackReturn on_configure_impl()  override;
  CallbackReturn on_activate_impl()   override;
  CallbackReturn on_deactivate_impl() override;

  // ---- 关节索引 ----
  std::vector<size_t> arm_indices_;                          // info_.joints 索引
  std::unordered_map<std::string, size_t> joint_indices_;    // name -> info_.joints 索引

  // ---- 状态 / 命令缓冲区 ----
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  std::vector<double> hw_position_commands_;

  // SDK 命令缓冲（float），复用避免每次 write 都分配
  std::vector<float> cmd_buffer_;

  // ---- 诊断用（不导出为 ROS2 state_interface） ----
  std::vector<uint16_t> enable_states_;
  std::vector<uint16_t> error_states_;

  // ---- 并发 / 日志节流 ----
  std::mutex mutex_;
  int error_streak_{0};

  // 子类可在 on_init 之前调整初值等待超时
  int configure_timeout_sec_{5};
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__JOINT_GROUP_BASE_HPP_