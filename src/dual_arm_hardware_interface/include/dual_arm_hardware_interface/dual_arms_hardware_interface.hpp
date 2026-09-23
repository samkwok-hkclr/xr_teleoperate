#ifndef DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_

#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <boost/lockfree/spsc_queue.hpp>

#include <rclcpp/macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/hardware_base.hpp"

namespace dual_arm_hardware_interface
{

struct CsvRow
{
  double   t_s;         // steady_clock::now().time_since_epoch() in seconds
  float    left[7];     // NaN if not present
  float    right[7];
  uint8_t  follow;      // 0/1
  uint8_t  mode;
  uint16_t radio;
};

/**
 * @brief Combined left-arm + head + right-arm hardware interface.
 *
 * Why this class exists
 * ---------------------
 * The SDK exposes a single 100 Hz UDP command channel. Every
 * ControlApi::move_joint(group, ...) call rewrites the JointCmd array for
 * the WHOLE robot, filling the non-selected arms with their current
 * feedback positions. If left_arm and right_arm are separate
 * SystemInterface instances, controller_manager calls write() twice per
 * cycle, producing two UDP packets. rt_control only consumes one packet
 * per 10 ms, so the second packet either overwrites the first (freezing
 * one arm) or is dropped.
 *
 * The only clean fix is to combine both arms into a single
 * SystemInterface so that write() produces exactly ONE packet per cycle,
 * containing targets for every joint (left + head + right).
 *
 * Joint classification
 * --------------------
 * Joints are classified by their name suffix:
 *   *_l  -> left arm   (SDK arm index 0)
 *   *_r  -> right arm  (SDK arm index 2)
 *   other -> head      (SDK arm index 1, may be empty)
 *
 * This MUST match the order of `robot.arm` entries in config.yaml:
 *   arm[0] = left, arm[1] = head, arm[2] = right
 *
 * If your config only has two entries, add a placeholder head entry with
 * joint_size: 0 to keep the indices aligned.
 */

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

  void start_tuner_node();
  void stop_tuner_node();

protected:
  const char* component_name() const override { return "dual_arms"; }

  CallbackReturn on_init_impl()       override;
  CallbackReturn on_configure_impl()  override;
  CallbackReturn on_activate_impl()   override;
  CallbackReturn on_deactivate_impl() override;
  CallbackReturn on_cleanup_impl()    override;
  CallbackReturn on_shutdown_impl()   override;

  void start_csv_writer();
  void stop_csv_writer();
  void csv_writer_loop();

  // ---- Joint classification (indices into info_.joints) ----
  std::vector<size_t> left_indices_;    // SDK arm[0]
  std::vector<size_t> head_indices_;    // SDK arm[1]
  std::vector<size_t> right_indices_;   // SDK arm[2]

  std::unordered_map<std::string, size_t> joint_indices_;

  // ---- Per-joint buffers (length = info_.joints.size()) ----
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  std::vector<double> hw_position_commands_;
  std::vector<double> rt_control_states_;

  // ---- Combined command buffer in SDK order (left + head + right) ----
  std::vector<float> cmd_buffer_;

  // ---- Optional per-side simulation (for mixed real/sim testing) ----
  // If sim_left  is true, the left  arm is held at its current feedback
  // position and its command interfaces are ignored.
  // If sim_right is true, the same happens for the right arm.
  bool sim_left_{false};
  bool sim_right_{false};

  int debug_read_tick_{0};
  int debug_write_tick_{0};
  int debug_every_n_{500};        // 每 100 次 = 1 Hz

  std::mutex mutex_;
  int error_streak_{0};
  int configure_timeout_sec_{5};

  std::chrono::steady_clock::time_point last_write_time_{};
  std::vector<float>                    last_cmd_buffer_;

  std::atomic<uint8_t>  trajectory_mode_{2};
  std::atomic<uint16_t> trajectory_radio_{500};

  rclcpp::Node::SharedPtr                                      tuner_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr         tuner_executor_;
  std::thread                                                  tuner_thread_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  delta_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  timing_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tuner_sub_;
  std::atomic<bool>                                            tuner_running_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            csv_enable_sub_;

  std::thread           csv_writer_thread_;
  std::atomic<bool>     csv_writer_running_{false};
  std::string           csv_path_;
  std::atomic<bool>     csv_is_open_{false};
  std::atomic<uint64_t> csv_dropped_{0};
  std::atomic<bool>     csv_requested_{false};

  static constexpr std::size_t kCsvQueueCapacity = 4096;

  boost::lockfree::spsc_queue<CsvRow, boost::lockfree::capacity<kCsvQueueCapacity>> csv_queue_;
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_HARDWARE_INTERFACE_HPP_