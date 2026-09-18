#ifndef DUAL_ARM_HARDWARE_INTERFACE__ELEVATOR_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__ELEVATOR_HARDWARE_INTERFACE_HPP_

#include <rclcpp/macros.hpp>

#include "dual_arm_hardware_interface/joint_group_base.hpp"

namespace dual_arm_hardware_interface
{

/**
 * @brief Elevator (vertical lift) hardware interface.
 *
 * Drives dual_arm_v2_2_sdk::Group::ELEVATOR. All behaviour is inherited
 * from JointGroupBase; this class only identifies itself and its target
 * Group.
 *
 * The number of joints must match the SDK configuration for the ELEVATOR
 * entry in `robot.arm` inside config.yaml. Typically this is a single
 * prismatic joint, but multi-joint elevators are supported as long as the
 * SDK exposes them as one Group.
 */
class ElevatorHardwareInterface : public JointGroupBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ElevatorHardwareInterface)

  ElevatorHardwareInterface() = default;
  ~ElevatorHardwareInterface() override = default;

protected:
  const char* component_name() const override
  {
    return "elevator";
  }

  dual_arm_v2_2_sdk::Group group() const override
  {
    return dual_arm_v2_2_sdk::Group::ELEVATOR;
  }
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__ELEVATOR_HARDWARE_INTERFACE_HPP_