#include "dual_arm_hardware_interface/right_gripper_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::RightGripperHardwareInterface,
    hardware_interface::SystemInterface)