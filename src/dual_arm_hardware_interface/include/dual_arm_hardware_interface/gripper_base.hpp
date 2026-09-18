#ifndef DUAL_ARM_HARDWARE_INTERFACE__GRIPPER_BASE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__GRIPPER_BASE_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/hardware_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Base class for gripper hardware components backed by the SDK's
 *        set_gripper_position() / get_gripper_state() API.
 *
 * Typical users:
 *   - LeftGripperHardwareInterface  (Group::LEFT_ARM  → gripperL)
 *   - RightGripperHardwareInterface (Group::RIGHT_ARM → gripperR)
 *
 * Gripper characteristics (see control_api.h):
 *   - The SDK exposes the gripper position as an integer in [0, 1000],
 *     where 0 = open and 1000 = closed.
 *   - For ROS2 we expose the same joint as a float in [0, 1] so that
 *     standard gripper controllers and MoveIt can talk to it naturally.
 *   - Speed and force are "config" parameters that are applied once when the
 *     component is activated. They are read from the hardware_parameters:
 *       <param name="gripper_speed">500</param>   # 1..1000
 *       <param name="gripper_force">500</param>   # 1..1000
 *   - Only a single joint is expected. If the URDF declares more, the first
 *     one is used and the rest are ignored with a warning.
 *
 * Derived classes only need to implement:
 *   - group()          : LEFT_ARM for left gripper, RIGHT_ARM for right
 *   - component_name() : inherited pure virtual from HardwareBase
 */
class GripperBase : public HardwareBase
{
public:
  GripperBase() = default;
  ~GripperBase() override = default;

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

  // ---- 关节（单个） ----
  std::string joint_name_;

  // ---- 归一化位置（ROS2 侧，[0, 1]） ----
  double hw_position_state_{0.0};
  double hw_position_command_{0.0};

  // ---- SDK 侧配置 ----
  uint16_t speed_{500};
  uint16_t force_{500};

  // ---- 并发 / 日志节流 ----
  std::mutex mutex_;
  int error_streak_{0};

  // 子类可在构造时调整初值等待超时
  int configure_timeout_sec_{5};
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__GRIPPER_BASE_HPP_