#include "dual_arm_hardware_interface/left_arm_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::LeftArmHardwareInterface,
    hardware_interface::SystemInterface)