#ifndef DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/hardware_base.hpp"
#include "dual_arm_hardware_interface/dual_arms_common.hpp"

namespace dual_arm_hardware_interface
{

class DualArmsHardwareInterface : public HardwareBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmsHardwareInterface)

  DualArmsHardwareInterface() = default;
  ~DualArmsHardwareInterface() override = default;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

protected:
  const char* component_name() const override { return "dual_arms"; }

  CallbackReturn on_init_impl()       override;
  CallbackReturn on_configure_impl()  override;
  CallbackReturn on_activate_impl()   override;
  CallbackReturn on_deactivate_impl() override;
  CallbackReturn on_cleanup_impl()    override;
  CallbackReturn on_shutdown_impl()   override;

  std::mutex mutex_;

  // ---- Extracted shared state -------------------------------------------
  JointLayout                       layout_;
  JointBuffers                      bufs_;
  CombinedCmdBuffer                 cmdbuf_;
  std::unique_ptr<CsvLogger>        csv_;
  std::unique_ptr<TrajectoryTuner>  tuner_;
  WriteTimingStats                  timing_stats_;

  // ---- Transport-specific (kept here) -----------------------------------
  std::vector<double> rt_control_states_;
  bool sim_left_{false};
  bool sim_right_{false};

  std::atomic<bool>     stream_restart_requested_{false};
  std::atomic<uint8_t>  pending_mode_{2};
  std::atomic<uint16_t> pending_radio_{500};

  TrajectoryParams trajectory_;

  int debug_read_tick_{0};
  int debug_write_tick_{0};
  int debug_every_n_{500};
  int error_streak_{0};
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_