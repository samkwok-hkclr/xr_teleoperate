#ifndef DUAL_ARM_HARDWARE_INTERFACE__HARDWARE_BASE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__HARDWARE_BASE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "control_api.h"

namespace dual_arm_hardware_interface
{

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Common base for every hardware component that talks to the
 *        dual_arm_v2_2 SDK.
 *
 * Responsibilities:
 *   - Acquire the process-wide shared ControlApi via ControlApiManager.
 *   - Parse the required `config_prefix` hardware parameter.
 *   - Provide unified lifecycle plumbing (on_init / on_configure / ...)
 *     so that each concrete component only implements the parts that are
 *     actually different.
 *
 * Derived classes override:
 *   - component_name()          : short identifier used in logs, e.g. "left_arm"
 *   - on_configure_impl()       : optional, wait for initial state, ...
 *   - on_activate_impl()        : optional, e.g. set_group(..., true)
 *   - on_deactivate_impl()      : optional, e.g. set_group(..., false)
 *   - on_cleanup_impl()         : optional
 */
class HardwareBase : public hardware_interface::SystemInterface
{
public:
  HardwareBase();
  ~HardwareBase() override = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup   (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown  (const rclcpp_lifecycle::State& previous_state) override;

protected:
  // ---- 子类必须实现 ----
  virtual const char* component_name() const = 0;

  // ---- 子类可选覆盖的生命周期钩子 ----
  virtual CallbackReturn on_init_impl()       { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_configure_impl()  { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_activate_impl()   { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_deactivate_impl() { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_cleanup_impl()    { return CallbackReturn::SUCCESS; }

  // ---- 共享资源 ----
  std::shared_ptr<dual_arm_v2_2_sdk::ControlApi> api_;
  std::string config_prefix_;
  rclcpp::Logger logger_;

  int debug_tick_{0};
  const int debug_every_n_{100};

  // ---- 辅助 ----
  bool is_active() const;
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__HARDWARE_BASE_HPP_