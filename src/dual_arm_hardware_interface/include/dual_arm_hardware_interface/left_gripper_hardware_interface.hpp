#ifndef DUAL_ARM_HARDWARE_INTERFACE__LEFT_GRIPPER_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__LEFT_GRIPPER_HARDWARE_INTERFACE_HPP_

#include <rclcpp/macros.hpp>

#include "dual_arm_hardware_interface/gripper_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Left gripper hardware interface.
 *
 * Drives the gripper named "gripperL" in the SDK. In the SDK's Group model,
 * grippers belong to the same Group as their arm:
 *
 *   Group::LEFT_ARM  →  gripperL
 *   Group::RIGHT_ARM →  gripperR
 *
 * Therefore this class passes Group::LEFT_ARM to GripperBase, which then
 * calls set_gripper_position(Group::LEFT_ARM, ...) and lets the SDK resolve
 * the actual gripper index by name.
 *
 * Position encoding:
 *   ROS2 side : float in [0.0, 1.0]  (0 = fully open, 1 = fully closed)
 *   SDK side  : uint16 in [0, 1000]
 *
 * Speed and force are configured via hardware_parameters:
 *   <param name="gripper_speed">500</param>   <!-- 1..1000 -->
 *   <param name="gripper_force">500</param>   <!-- 1..1000 -->
 */
class LeftGripperHardwareInterface : public GripperBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeftGripperHardwareInterface)

  LeftGripperHardwareInterface() = default;
  ~LeftGripperHardwareInterface() override = default;

protected:
  const char* component_name() const override
  {
    return "left_gripper";
  }

  dual_arm_v2_2_sdk::Group group() const override
  {
    return dual_arm_v2_2_sdk::Group::LEFT_ARM;
  }
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__LEFT_GRIPPER_HARDWARE_INTERFACE_HPP_