#ifndef DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_hardware_interface/telemanip_session.hpp"

namespace dual_arm_hardware_interface
{

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ============================================================
// CSV row (identical to the legacy class — same format, same file name scheme)
// ============================================================
struct CsvRow
{
  double   t_s;         // steady_clock::now().time_since_epoch() in seconds
  float    left[7];     // NaN if not present
  float    right[7];
  uint8_t  follow;      // 0/1
  uint8_t  mode;
  uint16_t radio;
};

// ============================================================
// 单关节限幅器：速度 + 加速度二阶限制
// ============================================================
struct JointLimiter
{
  double max_vel{0.0};      // rad/s, 0 = 禁用速度限制
  double max_acc{0.0};      // rad/s^2, 0 = 禁用加速度限制
  double last_cmd{0.0};     // 上一次"已限幅"命令
  double last_vel{0.0};     // 上一次"已限幅"速度
  bool   initialized{false};
  uint64_t clamp_count{0};  // 诊断：被裁剪的次数

  // 输入 desired（rad），输出限幅后的命令（rad）
  // dt：周期，秒
  double limit(double desired, double dt)
  {
    // 首次调用：以当前位置为起点，不做限制
    if (!initialized)
    {
      last_cmd = desired;
      last_vel = 0.0;
      initialized = true;
      return desired;
    }
    if (dt <= 0.0 || max_vel <= 0.0)
    {
      last_cmd = desired;
      return desired;
    }

    // ---- 步骤 1：期望速度 ----
    const double desired_vel = (desired - last_cmd) / dt;
    double vel = desired_vel;
    bool clamped = false;

    // ---- 步骤 2：速度裁剪 ----
    if (std::abs(vel) > max_vel)
    {
      vel = std::copysign(max_vel, vel);
      clamped = true;
    }

    // ---- 步骤 3：加速度裁剪 ----
    if (max_acc > 0.0)
    {
      double desired_acc = (vel - last_vel) / dt;
      if (std::abs(desired_acc) > max_acc)
      {
        desired_acc = std::copysign(max_acc, desired_acc);
        vel = last_vel + desired_acc * dt;

        // 加速度调整后可能又超出速度上限，再夹一次
        if (std::abs(vel) > max_vel)
        {
          vel = std::copysign(max_vel, vel);
        }
        clamped = true;
      }
    }

    if (clamped) 
      ++clamp_count;

    // ---- 步骤 4：计算限幅后的命令 ----
    const double limited = last_cmd + vel * dt;

    // ---- 步骤 5：更新内部状态 ----
    last_cmd = limited;
    last_vel = vel;

    return limited;
  }
};

/**
 * @brief Combined left-arm + head + right-arm hardware interface over the
 *        telemanip cuarm_2_2 C ABI (Zenoh transport).
 *
 * Why a separate class
 * --------------------
 * The legacy DualArmsHardwareInterface talks to the old UDP ControlApi SDK.
 * This new class talks to the telemanip C ABI + telemanip-agent-cuarm_2_2 +
 * zenohd. Both live in the same package and are selected via the URDF's
 * <plugin> tag.
 *
 * Key semantic differences vs the legacy class
 * --------------------------------------------
 * - write() STAGES three joint targets (one per device_id). Staging is pure
 *   memory work: it never blocks and never sends. The SDK's own beat thread
 *   packs the staging table into one Envelope per period_ms and sends it.
 *   "One packet per cycle for the whole robot" is therefore free.
 * - read() reads the three groups separately:
 *     arms  -> zero-alloc tms_cuarm_2_2_session_joints()
 *     head  -> allocated   tms_cuarm_2_2_session_state()
 * - There is no sim_left_ / sim_right_ member. To hold one side, simply stop
 *   staging it — the SDK keeps sending the last staged value. To enable a
 *   side, call group_enable(id, true).
 *
 * Joint classification
 * --------------------
 *   *_l  -> "left"  group (7 joints)
 *   *_r  -> "right" group (7 joints)
 *   other -> "head"  group (2 joints, no gripper)
 *
 * URDF hardware parameters
 * ------------------------
 *   cell_id         (string, required)  e.g. "cell-cuarm1"
 *   period_ms       (int, default 10)   must equal the agent's period_ms
 *   zenoh_config    (string, optional)  absolute path to zenoh.json5; when
 *                                        empty, auto-derived from the package
 *                                        share directory: <share>/config/zenoh.json5
 *   agent_wait_ms   (int, default 10000) max wait for the agent liveliness
 *   trajectory_mode (int, default 1)    0 direct, 1 curve fit, 2 filter
 *   trajectory_radio(int, default 15)   mode-1: 0..100, mode-2: 0..999
 *
 * A runtime tuner node is provided so the (mode, radio) pair can be changed
 * live via /<name>/trajectory_tuner (Int16MultiArray = [mode, radio]).
 */
class DualArmsTelemanipHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DualArmsTelemanipHardwareInterface)

  DualArmsTelemanipHardwareInterface() = default;
  ~DualArmsTelemanipHardwareInterface() override = default;

  // ---- lifecycle ----------------------------------------------------------
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup   (const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown  (const rclcpp_lifecycle::State& previous_state) override;

  // ---- ros2_control -------------------------------------------------------
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // ---- tuner + csv (public so lifecycle hooks can call them) -------------
  void start_tuner_node();
  void stop_tuner_node();

private:
  // ============================================================
  // Helpers
  // ============================================================
  
  /// 等待三组各拿到第一个 state，并写入 hw_position_states_ /
  /// hw_position_commands_ / joint_limiters_。
  /// @return true 表示全部组拿到有效 state。
  bool wait_and_seed_all_groups(int timeout_ms = 2000);
  
  /// 复位所有关节限幅器（下次 limit() 时以当前位置为起点）。
  void reset_joint_limiters();

protected:
  void start_csv_writer();
  void stop_csv_writer();
  void csv_writer_loop();

  bool is_active() const;

  // ============================================================
  // Configuration (parsed in on_init)
  // ============================================================
  std::string cell_id_;
  std::string zenoh_config_path_;
  uint64_t    period_ms_{10};
  uint64_t    agent_wait_ms_{10000};

  // ============================================================
  // Logger + session
  // ============================================================
  rclcpp::Logger logger_{rclcpp::get_logger("dual_arms_telemanip")};
  std::unique_ptr<TelemanipSession> session_;
  bool is_activated_{false};

  // ============================================================
  // Joint classification (indices into info_.joints)
  // ============================================================
  std::vector<size_t> left_indices_;    // -> device_id "left"
  std::vector<size_t> head_indices_;    // -> device_id "head"
  std::vector<size_t> right_indices_;   // -> device_id "right"

  std::unordered_map<std::string, size_t> joint_indices_;

  // ============================================================
  // Per-joint buffers (length = info_.joints.size())
  // ============================================================
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  std::vector<double> hw_position_commands_;

  // ============================================================
  // Per-group command buffers (fed to stage_joint() once per write()).
  // cmd_buffer_ / last_cmd_buffer_ remain in SDK order [left|head|right]
  // because they feed the delta publisher, the CSV writer and the debug
  // prints — exactly like the legacy class.
  // ============================================================
  std::vector<float> left_cmd_buf_;
  std::vector<float> head_cmd_buf_;
  std::vector<float> right_cmd_buf_;

  std::vector<float> cmd_buffer_;        // [left | head | right]
  std::vector<float> last_cmd_buffer_;   // swap partner for zero-copy delta

  // ============================================================
  // Runtime-adjustable trajectory parameters
  // ============================================================
  std::atomic<uint8_t>  trajectory_mode_{2};      // 0 direct, 1 curve fit, 2 filter
  std::atomic<uint16_t> trajectory_radio_{500};    // 0..100 (mode 1), 0..999 (mode 2)

  // ---- 关节限幅器（index 对应 info_.joints 顺序）----
  std::vector<JointLimiter> joint_limiters_;
  double max_joint_vel_rad_s_{0.0};   // 全局默认，来自 URDF
  double max_joint_acc_rad_s2_{0.0};  // 全局默认，来自 URDF

  // ============================================================
  // Diagnostics / timing
  // ============================================================
  std::mutex mutex_;
  int debug_read_tick_{0};
  int debug_write_tick_{0};
  int debug_every_n_{500};      // 500 writes @ 100 Hz = 5 s

  int delta_sub_check_tick_{0};
  bool delta_has_sub_{false};

  int timing_sub_check_tick_{0};
  bool timing_has_sub_{false};

  int error_streak_{0};

  // ============================================================
  // Embedded tuner node
  // ============================================================
  rclcpp::Node::SharedPtr                                          tuner_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr             tuner_executor_;
  std::thread                                                      tuner_thread_;
  std::atomic<bool>                                                tuner_running_{false};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   delta_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   timing_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr  tuner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             csv_enable_sub_;

  // ============================================================
  // Async CSV writer
  // ============================================================
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

#endif  // DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_TELEMANIP_HARDWARE_INTERFACE_HPP_