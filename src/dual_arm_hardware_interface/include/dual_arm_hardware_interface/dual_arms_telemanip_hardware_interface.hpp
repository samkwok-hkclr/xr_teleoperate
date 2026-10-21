#ifndef DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/dual_arms_common.hpp"
#include "dual_arm_hardware_interface/telemanip_session.hpp"

namespace dual_arm_hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DualArmsTelemanipHardwareInterface : public DualArmsLifecycleBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmsTelemanipHardwareInterface)

  DualArmsTelemanipHardwareInterface() = default;
  ~DualArmsTelemanipHardwareInterface() override = default;

  // ---- ros2_control -----------------------------------------------------
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // ---- lifecycle (same shape as the legacy interface) -------------------
  const char* component_name() const override { return "dual_arms_telemanip"; }

  CallbackReturn on_init_impl()       override;
  CallbackReturn on_configure_impl()  override;
  CallbackReturn on_activate_impl()   override;
  CallbackReturn on_deactivate_impl() override;
  CallbackReturn on_cleanup_impl()    override;
  CallbackReturn on_shutdown_impl()   override;

private:
  bool wait_and_seed_all_groups(int timeout_ms = 2000);
  void reset_joint_limiters();

  // ---- Configuration ----------------------------------------------------
  std::string cell_id_;
  std::string zenoh_config_path_;
  uint64_t    period_ms_{10};
  uint64_t    agent_wait_ms_{10000};

  std::unique_ptr<TelemanipSession> session_;
  bool                              is_activated_{false};

  // ---- Extracted shared state -------------------------------------------
  JointLayout                       layout_;
  JointBuffers                      bufs_;
  CombinedCmdBuffer                 cmdbuf_;
  std::unique_ptr<CsvLogger>        csv_;
  std::unique_ptr<TrajectoryTuner>  tuner_;
  WriteTimingStats                  timing_stats_;

  // ---- Transport-specific ------------------------------------------------
  TrajectoryParams trajectory_;

  std::vector<JointLimiter> joint_limiters_;
  double max_joint_vel_rad_s_{0.0};
  double max_joint_acc_rad_s2_{0.0};

  std::mutex mutex_;
  int debug_read_tick_{0};
  int debug_write_tick_{0};
  int debug_every_n_{500};
  int error_streak_{0};
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_