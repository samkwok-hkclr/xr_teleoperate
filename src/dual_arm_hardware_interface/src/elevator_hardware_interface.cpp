#include "dual_arm_hardware_interface/elevator_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    dual_arm_hardware_interface::ElevatorHardwareInterface,
    hardware_interface::SystemInterface)