#ifndef DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_COMMON_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_COMMON_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dual_arm_hardware_interface
{

// ============================================================
// CSV row — identical layout in both interfaces
// ============================================================
struct CsvRow
{
  double   t_s;         // steady_clock seconds
  float    left[7];     // NaN when absent
  float    right[7];
  uint8_t  follow;      // 0/1
  uint8_t  mode;
  uint16_t radio;
};

// ============================================================
// Single-joint velocity + acceleration limiter
// ============================================================
struct JointLimiter
{
  double   max_vel{0.0};      // rad/s,   0 = disabled
  double   max_acc{0.0};      // rad/s^2, 0 = disabled
  double   last_cmd{0.0};
  double   last_vel{0.0};
  bool     initialized{false};
  uint64_t clamp_count{0};

  double limit(double desired, double dt);
};

// ============================================================
// Joint classification by name suffix
//   *_l -> left,  *_r -> right,  else -> head
// ============================================================
enum class JointSide { kLeft, kHead, kRight };
JointSide classify_joint(const std::string& name);

// ============================================================
// Joint layout: index lists + name lookup, sorted by name
// ============================================================
struct JointLayout
{
  std::vector<size_t> left_indices_;
  std::vector<size_t> head_indices_;
  std::vector<size_t> right_indices_;
  std::unordered_map<std::string, size_t> joint_indices_;
  std::vector<std::string> joint_names_;   // snapshot of info_.joints names

  void   build(const hardware_interface::HardwareInfo& info);
  void   log(rclcpp::Logger logger, const char* component) const;
  size_t total() const { return left_indices_.size() + head_indices_.size() + right_indices_.size(); }
};

// ============================================================
// Per-joint buffers (length = info_.joints.size())
// ============================================================
struct JointBuffers
{
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  std::vector<double> hw_position_commands_;

  void allocate(size_t n);

  /// Copy seed values into both state and command buffers for each group.
  void seed(const JointLayout& layout,
            const std::vector<float>& left_now,
            const std::vector<float>& head_now,
            const std::vector<float>& right_now);

  std::vector<hardware_interface::StateInterface>
  export_state(const JointLayout& layout) const;

  std::vector<hardware_interface::CommandInterface>
  export_command(const JointLayout& layout) const;

  static bool all_zero(const std::vector<float>& v, double eps);
};

// ============================================================
// Combined command buffer [left | head | right] in SDK order
// ============================================================
struct CombinedCmdBuffer
{
  std::vector<float> left_cmd_buf_;
  std::vector<float> head_cmd_buf_;
  std::vector<float> right_cmd_buf_;

  std::vector<float> cmd_buffer_;        // [left | head | right]
  std::vector<float> last_cmd_buffer_;   // swap partner for delta

  void allocate(size_t n_left, size_t n_head, size_t n_right);

  /// Snapshot hw_position_commands_ into per-group buffers under a mutex.
  void gather(const JointLayout& layout,
              const JointBuffers& bufs,
              std::mutex& m,
              bool sim_left = false,
              bool sim_right = false,
              const std::vector<double>* rt_control_states = nullptr);

  /// Reassemble cmd_buffer_ from left+head+right buffers.
  void pack();

  /// Apply per-joint limiter to left/head/right in-place.
  void apply_limiters(std::vector<JointLimiter>& limiters,
                      const JointLayout& layout,
                      double dt_s);

  void swap() { std::swap(cmd_buffer_, last_cmd_buffer_); }
};

// ============================================================
// Build a CsvRow from cmd_buffer_ + metadata
// ============================================================
CsvRow make_csv_row(const std::vector<float>& cmd_buffer,
                    const JointLayout& layout,
                    double t_s,
                    uint8_t follow,
                    uint8_t mode,
                    uint16_t radio);

// ============================================================
// Async CSV logger: owns the writer thread + spsc queue
// ============================================================
class CsvLogger
{
public:
  CsvLogger(rclcpp::Logger logger, std::string component);
  ~CsvLogger();

  void start();
  void stop();

  void request(bool on) { requested_.store(on, std::memory_order_release); }
  bool is_open()   const { return open_.load(std::memory_order_acquire); }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

  /// Non-blocking; drops the row if the queue is full.
  void push(const CsvRow& row);

private:
  void loop();

  rclcpp::Logger logger_;
  std::string    component_;

  std::thread           thread_;
  std::atomic<bool>     running_{false};
  std::atomic<bool>     requested_{false};
  std::atomic<bool>     open_{false};
  std::atomic<uint64_t> dropped_{0};
  std::string           path_;

  static constexpr std::size_t kCapacity = 4096;
  boost::lockfree::spsc_queue<CsvRow, boost::lockfree::capacity<kCapacity>> queue_;
};

// ============================================================
// Embedded trajectory tuner node (pubs, subs, spin thread)
//
// Semantics:
//   on_tuner(mode, radio)  — called when /<name>/trajectory_tuner receives
//                            a valid [mode, radio] pair. The interface
//                            decides what to do (store atomics, request
//                            stream restart, ...).
//   on_csv(bool)           — called when /<name>/csv_enable is toggled.
// ============================================================
class TrajectoryTuner
{
public:
  using OnTuner = std::function<void(uint8_t, uint16_t)>;
  using OnCsv   = std::function<void(bool)>;

  TrajectoryTuner(rclcpp::Logger logger,
                  std::string hw_name,
                  OnTuner on_tuner,
                  OnCsv on_csv);
  ~TrajectoryTuner();

  void start();
  void stop();

  /// Throttled publish of cmd_buffer_ - last_cmd_buffer_ as Float32MultiArray.
  void publish_delta(const std::vector<float>& current,
                     const std::vector<float>& last);

  /// Throttled publish of [call_ms, period_ms, threshold_ms, actual_dt_ms].
  void publish_timing(double call_ms,
                      double period_ms,
                      double threshold_ms,
                      double actual_dt_ms);

private:
  rclcpp::Logger logger_;
  std::string    hw_name_;
  OnTuner        on_tuner_;
  OnCsv          on_csv_;

  rclcpp::Node::SharedPtr                                       node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr          exec_;
  std::thread                                                   thread_;
  std::atomic<bool>                                             running_{false};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  delta_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  timing_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr tuner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            csv_sub_;

  int  delta_sub_check_tick_{0};
  bool delta_has_sub_{false};
  int  timing_sub_check_tick_{0};
  bool timing_has_sub_{false};
};

// ============================================================
// Rolling 100-sample write() timing statistics
// ============================================================
class WriteTimingStats
{
public:
  /// Returns true if a warning-level anomaly was detected (mean drift,
  /// high sd, or tail spike). Fills mean/sd/min/max and call mean/max.
  bool update(double actual_dt_ms, double call_ms, double period_ms,
              double& dt_mean, double& dt_sd, double& dt_min, double& dt_max,
              double& cl_mean, double& cl_max, size_t& n);

private:
  std::array<double, 100> dt_history_{};
  std::array<double, 100> call_history_{};
  size_t idx_{0};
  size_t filled_{0};
};

// ============================================================
// URDF <param> reader with typed accessors + defaults
//
// Wraps hardware_interface::HardwareInfo so on_init() code in
// both interfaces looks the same:
//
//     HardwareParams p(info_);
//     sim_left_      = p.get_bool("sim_left", false);
//     period_ms_     = p.get_u64 ("period_ms", 10);
//     cell_id_       = p.get_string("cell_id");   // "" if missing
//     if (!p.has("cell_id")) { error... }
// ============================================================
class HardwareParams
{
public:
  explicit HardwareParams(const hardware_interface::HardwareInfo& info)
    : info_(info) {}

  bool has(const std::string& key) const;
  const std::string* get(const std::string& key) const;   // nullptr if empty/missing

  bool        get_bool  (const std::string& key, bool                def = false) const;
  int         get_int   (const std::string& key, int                 def = 0)     const;
  uint16_t    get_u16   (const std::string& key, uint16_t            def = 0)     const;
  uint32_t    get_u32   (const std::string& key, uint32_t            def = 0)     const;
  uint64_t    get_u64   (const std::string& key, uint64_t            def = 0)     const;
  double      get_double(const std::string& key, double              def = 0.0)   const;
  std::string get_string(const std::string& key, const std::string&  def = {})    const;

private:
  const hardware_interface::HardwareInfo& info_;
};

struct TrajectoryParams
{
  std::atomic<uint8_t>  mode{2};     // 0 direct, 1 curve fit, 2 filter
  std::atomic<uint16_t> radio{500};  // 0..100 (mode 1), 0..999 (mode 2)

  /// Parse from URDF hardware parameters (defaults 2 / 500).
  /// `mode` is clamped to [0, 2]; `radio` is stored as-is.
  void load(const HardwareParams& params);

  void set(uint8_t m, uint16_t r) {
    mode .store(m, std::memory_order_release);
    radio.store(r, std::memory_order_release);
  }
};

// ============================================================
// Lifecycle delegation base
//
// Provides final on_init/on_configure/... overrides that simply
// forward to on_XXX_impl(). Mirrors the pattern already used by
// HardwareBase (legacy UDP path) so both interfaces share the
// same class-level structure:
//
//     CallbackReturn on_init_impl() override;
//     CallbackReturn on_configure_impl() override;
//     ...
//
// The base owns logger_, info_ (from SystemInterface) and
// is_active(); component_name() is virtual with a default.
// ============================================================
class DualArmsLifecycleBase : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) final;
  CallbackReturn on_configure (const rclcpp_lifecycle::State& s) final;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State& s) final;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& s) final;
  CallbackReturn on_cleanup   (const rclcpp_lifecycle::State& s) final;
  CallbackReturn on_shutdown  (const rclcpp_lifecycle::State& s) final;
  CallbackReturn on_error     (const rclcpp_lifecycle::State& s) final;

protected:
  virtual CallbackReturn on_init_impl()       = 0;
  virtual CallbackReturn on_configure_impl()  = 0;
  virtual CallbackReturn on_activate_impl()   = 0;
  virtual CallbackReturn on_deactivate_impl() = 0;
  virtual CallbackReturn on_cleanup_impl()    = 0;
  virtual CallbackReturn on_shutdown_impl()   = 0;
  virtual CallbackReturn on_error_impl()      { return CallbackReturn::SUCCESS; }

  virtual const char* component_name() const { return "dual_arms"; }

  bool is_active() const {
    return get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  rclcpp::Logger logger_{
      rclcpp::get_logger("dual_arms_hardware_interface")};
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__DUAL_ARMS_COMMON_HPP_