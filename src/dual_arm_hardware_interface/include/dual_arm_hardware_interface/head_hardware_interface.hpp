#ifndef DUAL_ARM_HARDWARE_INTERFACE__HEAD_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__HEAD_HARDWARE_INTERFACE_HPP_

#include <rclcpp/macros.hpp>

#include "dual_arm_hardware_interface/joint_group_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Head (pan/tilt) hardware interface.
 *
 * Drives dual_arm_v2_2_sdk::Group::HEAD. All behaviour is inherited from
 * JointGroupBase; this class only identifies itself and its target Group.
 *
 * The number of joints must match the SDK configuration for the HEAD arm
 * entry in `robot.arm` inside config.yaml. Joints are exported in the order
 * they appear in the URDF's ros2_control block.
 */
class HeadHardwareInterface : public JointGroupBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HeadHardwareInterface)

  HeadHardwareInterface() = default;
  ~HeadHardwareInterface() override = default;

protected:
  const char* component_name() const override
  {
    return "head";
  }

  dual_arm_v2_2_sdk::Group group() const override
  {
    return dual_arm_v2_2_sdk::Group::HEAD;
  }
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__HEAD_HARDWARE_INTERFACE_HPP_