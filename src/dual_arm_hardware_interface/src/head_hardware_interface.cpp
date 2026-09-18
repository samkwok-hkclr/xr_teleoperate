#include "dual_arm_hardware_interface/head_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::HeadHardwareInterface,
    hardware_interface::SystemInterface)