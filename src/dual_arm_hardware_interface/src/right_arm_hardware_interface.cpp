#include "dual_arm_hardware_interface/right_arm_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::RightArmHardwareInterface,
    hardware_interface::SystemInterface)