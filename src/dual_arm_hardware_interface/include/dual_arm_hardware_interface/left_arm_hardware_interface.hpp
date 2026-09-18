#ifndef DUAL_ARM_HARDWARE_INTERFACE__LEFT_ARM_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__LEFT_ARM_HARDWARE_INTERFACE_HPP_

#include <rclcpp/macros.hpp>

#include "dual_arm_hardware_interface/joint_group_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Left arm hardware interface.
 *
 * Drives dual_arm_v2_2_sdk::Group::LEFT_ARM. All behaviour is inherited
 * from JointGroupBase; this class only identifies itself and its target
 * Group.
 */
class LeftArmHardwareInterface : public JointGroupBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeftArmHardwareInterface)

  LeftArmHardwareInterface() = default;
  ~LeftArmHardwareInterface() override = default;

protected:
  const char* component_name() const override
  {
    return "left_arm";
  }

  dual_arm_v2_2_sdk::Group group() const override
  {
    return dual_arm_v2_2_sdk::Group::LEFT_ARM;
  }
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__LEFT_ARM_HARDWARE_INTERFACE_HPP_