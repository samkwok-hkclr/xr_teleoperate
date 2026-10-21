#include "dual_arm_hardware_interface/dual_arms_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>   // getpid

namespace dual_arm_hardware_interface
{

// ============================================================
// JointLimiter
// ============================================================
double JointLimiter::limit(double desired, double dt)
{
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

  const double desired_vel = (desired - last_cmd) / dt;
  double vel = desired_vel;
  bool   clamped = false;

  if (std::abs(vel) > max_vel) {
    vel = std::copysign(max_vel, vel);
    clamped = true;
  }
  if (max_acc > 0.0) 
  {
    double desired_acc = (vel - last_vel) / dt;
    if (std::abs(desired_acc) > max_acc) 
    {
      desired_acc = std::copysign(max_acc, desired_acc);
      vel = last_vel + desired_acc * dt;
      if (std::abs(vel) > max_vel) 
        vel = std::copysign(max_vel, vel);
      clamped = true;
    }
  }

  if (clamped) 
    ++clamp_count;

  const double limited = last_cmd + vel * dt;
  last_cmd = limited;
  last_vel = vel;
  return limited;
}

// ============================================================
// Joint classification
// ============================================================
JointSide classify_joint(const std::string& name)
{
  if (name.size() >= 2) 
  {
    const std::string suffix = name.substr(name.size() - 2);
    if (suffix == "_l") 
      return JointSide::kLeft;
    if (suffix == "_r") 
      return JointSide::kRight;
  }
  return JointSide::kHead;
}

// ============================================================
// JointLayout
// ============================================================
void JointLayout::build(const hardware_interface::HardwareInfo& info)
{
  const size_t n = info.joints.size();

  joint_indices_.clear();
  joint_indices_.reserve(n);
  left_indices_.clear();
  head_indices_.clear();
  right_indices_.clear();
  joint_names_.clear();
  joint_names_.reserve(n);

  for (size_t i = 0; i < n; ++i) 
  {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;
    joint_names_.push_back(joint.name);

    switch (classify_joint(joint.name)) 
    {
      case JointSide::kLeft:  
        left_indices_ .push_back(i); 
        break;
      case JointSide::kHead:  
        head_indices_ .push_back(i); 
        break;
      case JointSide::kRight: 
        right_indices_.push_back(i); 
        break;
    }
  }

  auto sort_by_name = [&](std::vector<size_t>& idx) {
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
      return info.joints[a].name < info.joints[b].name;
    });
  };
  sort_by_name(left_indices_);
  sort_by_name(head_indices_);
  sort_by_name(right_indices_);
}

void JointLayout::log(rclcpp::Logger logger, const char* component) const
{
  RCLCPP_INFO(logger,
              "[%s] joint classification: left=%zu, head=%zu, right=%zu",
              component, left_indices_.size(), head_indices_.size(),
              right_indices_.size());

  auto log_one = [&](const char* label, const std::vector<size_t>& idx) {
    for (size_t i : idx)
      RCLCPP_INFO(logger, "[%s]   %s: %s", component, label, joint_names_[i].c_str());
  };
  log_one("left",  left_indices_);
  log_one("head",  head_indices_);
  log_one("right", right_indices_);
}

// ============================================================
// JointBuffers
// ============================================================
void JointBuffers::allocate(size_t n)
{
  hw_position_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_  .assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_    .assign(n, 0.0);
  hw_position_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());
}

void JointBuffers::seed(const JointLayout& layout,
                        const std::vector<float>& left_now,
                        const std::vector<float>& head_now,
                        const std::vector<float>& right_now)
{
  auto apply = [&](const std::vector<size_t>& idx, const std::vector<float>& vals) {
    for (size_t k = 0; k < idx.size(); ++k) 
    {
      hw_position_states_  [idx[k]] = vals[k];
      hw_position_commands_[idx[k]] = vals[k];
    }
  };

  apply(layout.left_indices_,  left_now);
  apply(layout.head_indices_,  head_now);
  apply(layout.right_indices_, right_now);
}

bool JointBuffers::all_zero(const std::vector<float>& v, double eps)
{
  for (float x : v) 
    if (std::abs(x) > eps) 
      return false;

  return true;
}

std::vector<hardware_interface::StateInterface>
JointBuffers::export_state(const JointLayout& layout) const
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(layout.joint_names_.size() * 3);

  for (const auto& name : layout.joint_names_) 
  {
    auto it = layout.joint_indices_.find(name);
    if (it == layout.joint_indices_.end()) 
      continue;
    const size_t idx = it->second;
    // interface list comes from the URDF; we still use <joint> names
    // and offer pos/vel/effort if the pointers exist (they always do).
    (void)idx;
  }
  // NOTE: the state/command interfaces depend on info_.joints[].state_interfaces
  // so we accept them as parameters here. See the caller in each interface.
  return out;
}

std::vector<hardware_interface::CommandInterface>
JointBuffers::export_command(const JointLayout& layout) const
{
  (void)layout;
  return {};
}

// ============================================================
// CombinedCmdBuffer
// ============================================================
void CombinedCmdBuffer::allocate(size_t n_left, size_t n_head, size_t n_right)
{
  left_cmd_buf_ .assign(n_left,  0.0F);
  head_cmd_buf_ .assign(n_head,  0.0F);
  right_cmd_buf_.assign(n_right, 0.0F);
  cmd_buffer_      .assign(n_left + n_head + n_right, 0.0F);
  last_cmd_buffer_ .assign(n_left + n_head + n_right, 0.0F);
}

void CombinedCmdBuffer::gather(const JointLayout& layout,
                               const JointBuffers& bufs,
                               std::mutex& m,
                               bool sim_left,
                               bool sim_right,
                               const std::vector<double>* rt_control_states)
{
  std::lock_guard<std::mutex> lock(m);

  for (size_t k = 0; k < layout.left_indices_.size(); ++k) 
  {
    const size_t i = layout.left_indices_[k];
    left_cmd_buf_[k] = (sim_left && rt_control_states)
      ? static_cast<float>((*rt_control_states)[i])
      : static_cast<float>(bufs.hw_position_commands_[i]);
  }
  for (size_t k = 0; k < layout.head_indices_.size(); ++k) 
  {
    const size_t i = layout.head_indices_[k];
    head_cmd_buf_[k] = static_cast<float>(bufs.hw_position_commands_[i]);
  }
  for (size_t k = 0; k < layout.right_indices_.size(); ++k) 
  {
    const size_t i = layout.right_indices_[k];
    right_cmd_buf_[k] = (sim_right && rt_control_states)
      ? static_cast<float>((*rt_control_states)[i])
      : static_cast<float>(bufs.hw_position_commands_[i]);
  }
}

void CombinedCmdBuffer::pack()
{
  size_t k = 0;
  for (float v : left_cmd_buf_ ) 
    cmd_buffer_[k++] = v;
  for (float v : head_cmd_buf_ ) 
    cmd_buffer_[k++] = v;
  for (float v : right_cmd_buf_) 
    cmd_buffer_[k++] = v;
}

void CombinedCmdBuffer::apply_limiters(std::vector<JointLimiter>& limiters,
                                       const JointLayout& layout,
                                       double dt_s)
{
  for (size_t k = 0; k < layout.left_indices_.size(); ++k) 
  {
    const size_t gi = layout.left_indices_[k];
    left_cmd_buf_[k] = static_cast<float>(limiters[gi].limit(left_cmd_buf_[k], dt_s));
  }
  for (size_t k = 0; k < layout.head_indices_.size(); ++k) 
  {
    const size_t gi = layout.head_indices_[k];
    head_cmd_buf_[k] = static_cast<float>(limiters[gi].limit(head_cmd_buf_[k], dt_s));
  }
  for (size_t k = 0; k < layout.right_indices_.size(); ++k) 
  {
    const size_t gi = layout.right_indices_[k];
    right_cmd_buf_[k] = static_cast<float>(limiters[gi].limit(right_cmd_buf_[k], dt_s));
  }
}

// ============================================================
// make_csv_row
// ============================================================
CsvRow make_csv_row(const std::vector<float>& cmd_buffer,
                    const JointLayout& layout,
                    double t_s,
                    uint8_t follow,
                    uint8_t mode,
                    uint16_t radio)
{
  CsvRow row{};
  row.t_s    = t_s;
  row.follow = follow;
  row.mode   = mode;
  row.radio  = radio;

  const size_t nl = layout.left_indices_.size();
  const size_t nh = layout.head_indices_.size();
  const size_t nr = layout.right_indices_.size();
  const size_t off_right = nl + nh;

  if (nl == 7 && nr == 7) 
  {
    std::memcpy(row.left,  cmd_buffer.data() + 0,         7 * sizeof(float));
    std::memcpy(row.right, cmd_buffer.data() + off_right, 7 * sizeof(float));
  } 
  else 
  {
    for (size_t k = 0; k < 7; ++k)
      row.left[k]  = (k < nl) ? cmd_buffer[k]
                              : std::numeric_limits<float>::quiet_NaN();
    for (size_t k = 0; k < 7; ++k)
      row.right[k] = (k < nr) ? cmd_buffer[off_right + k]
                              : std::numeric_limits<float>::quiet_NaN();
  }
  return row;
}

// ============================================================
// CsvLogger
// ============================================================
CsvLogger::CsvLogger(rclcpp::Logger logger, std::string component)
  : logger_(logger), component_(std::move(component))
{}

CsvLogger::~CsvLogger() { stop(); }

void CsvLogger::start()
{
  if (thread_.joinable()) 
  {
    RCLCPP_WARN(logger_, "[%s] CSV writer already running", component_.c_str());
    return;
  }
  running_.store(true, std::memory_order_release);
  try 
  {
    thread_ = std::thread(&CsvLogger::loop, this);
  } 
  catch (const std::exception& e) 
  {
    running_.store(false, std::memory_order_release);
    RCLCPP_ERROR(logger_, "[%s] failed to start CSV writer thread: %s",
                 component_.c_str(), e.what());
    throw;
  }
  RCLCPP_INFO(logger_, "[%s] CSV writer thread started (queue capacity=%zu)",
              component_.c_str(), kCapacity);
}

void CsvLogger::stop()
{
  if (!thread_.joinable()) 
    return;
  running_.store(false, std::memory_order_release);
  thread_.join();
  open_.store(false, std::memory_order_release);
  requested_.store(false, std::memory_order_release);
  RCLCPP_INFO(logger_,
              "[%s] CSV writer thread stopped (total dropped=%lu)",
              component_.c_str(),
              static_cast<unsigned long>(dropped_.load()));
}

void CsvLogger::push(const CsvRow& row)
{
  if (!queue_.push(row)) 
  {
    dropped_.fetch_add(1, std::memory_order_relaxed);
  }
}

void CsvLogger::loop()
{
  std::ofstream file;
  bool          file_open = false;
  std::string   path;
  std::size_t   row_idx = 0;
  double        session_start_s = 0.0;
  std::size_t   writes_since_flush = 0;

  constexpr std::size_t kFlushEvery = 500;
  constexpr int         kBatchMax   = 200;
  constexpr int         kPollMs     = 2;

  CsvRow row{};

  auto write_row = [&](const CsvRow& r, double ts_offset) 
  {
    const double ts_s = r.t_s - ts_offset;
    file << row_idx++ << ',' << ts_s;
    for (int k = 0; k < 7; ++k) 
      file << ',' << r.left[k];
    for (int k = 0; k < 7; ++k) 
      file << ',' << r.right[k];
    file << ',' << static_cast<int>(r.follow)
         << ',' << static_cast<int>(r.mode)
         << ',' << static_cast<int>(r.radio) << '\n';
  };

  while (running_.load(std::memory_order_acquire))
  {
    try
    {
      const bool want = requested_.load(std::memory_order_acquire);

      // -------- open --------
      if (want && !file_open)
      {
        open_.store(false, std::memory_order_release);
        queue_.consume_all([](const CsvRow&){});

        const auto sys_now = std::chrono::system_clock::now();
        const std::time_t tt = std::chrono::system_clock::to_time_t(sys_now);
        std::tm tm_buf{};
        localtime_r(&tt, &tm_buf);

        std::ostringstream name;
        name << "/tmp/test_"
             << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
             << "_" << ::getpid() << ".csv";
        path = name.str();

        file.open(path, std::ios::out | std::ios::trunc);
        if (!file.is_open()) 
        {
          RCLCPP_ERROR(logger_, "[%s] cannot open CSV: %s",
                       component_.c_str(), path.c_str());
          requested_.store(false, std::memory_order_release);
        } 
        else 
        {
          file << "idx,timestamp_s";
          for (int j = 1; j <= 7; ++j) 
            file << ",left_j"  << j;
          for (int j = 1; j <= 7; ++j) 
            file << ",right_j" << j;
          file << ",follow,mode,radio\n";
          file << std::fixed << std::setprecision(6);
          file.flush();

          if (file.fail()) 
          {
            RCLCPP_ERROR(logger_, "[%s] CSV header flush failed: %s",
                         component_.c_str(), path.c_str());
            file.close();
            requested_.store(false, std::memory_order_release);
          } 
          else 
          {
            row_idx = 0;
            writes_since_flush = 0;
            session_start_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            file_open = true;
            path_ = path;
            dropped_.store(0, std::memory_order_relaxed);
            open_.store(true, std::memory_order_release);
            RCLCPP_INFO(logger_, "[%s] CSV log STARTED: %s", component_.c_str(), path.c_str());
          }
        }
      }
      // -------- close --------
      else if (!want && file_open)
      {
        open_.store(false, std::memory_order_release);
        while (queue_.pop(row)) write_row(row, session_start_s);
        file.flush();
        const bool write_ok = !file.fail();
        file.close();
        file_open = false;
        if (!write_ok) {
          RCLCPP_ERROR(logger_, "[%s] CSV flush failed on close: %s (%zu rows)",
                       component_.c_str(), path.c_str(), row_idx);
        }
        RCLCPP_INFO(logger_, "[%s] CSV log STOPPED: %s (%zu rows, %lu dropped)",
                    component_.c_str(), path.c_str(), row_idx,
                    static_cast<unsigned long>(dropped_.load(std::memory_order_relaxed)));
      }

      // -------- batch consume --------
      if (file_open) {
        int processed = 0;
        while (processed < kBatchMax && queue_.pop(row)) {
          write_row(row, session_start_s);
          ++processed;
          ++writes_since_flush;

          if (writes_since_flush >= kFlushEvery) {
            file.flush();
            writes_since_flush = 0;
            if (file.fail()) {
              RCLCPP_ERROR(logger_, "[%s] CSV write failed after %zu rows",
                           component_.c_str(), row_idx);
              open_.store(false, std::memory_order_release);
              file.close();
              file_open = false;
              requested_.store(false, std::memory_order_release);
              break;
            }
          }
        }
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "[%s] CSV writer exception: %s",
                   component_.c_str(), e.what());
      open_.store(false, std::memory_order_release);
      if (file_open) { try { file.flush(); } catch (...) {} try { file.close(); } catch (...) {} file_open = false; }
      requested_.store(false, std::memory_order_release);
    }
    catch (...) {
      RCLCPP_ERROR(logger_, "[%s] CSV writer unknown exception", component_.c_str());
      open_.store(false, std::memory_order_release);
      if (file_open) { try { file.flush(); } catch (...) {} try { file.close(); } catch (...) {} file_open = false; }
      requested_.store(false, std::memory_order_release);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }

  // -------- shutdown flush --------
  try {
    if (file_open) {
      open_.store(false, std::memory_order_release);
      while (queue_.pop(row)) write_row(row, session_start_s);
      file.flush();
      const bool write_ok = !file.fail();
      file.close();
      file_open = false;
      if (!write_ok)
        RCLCPP_ERROR(logger_, "[%s] CSV flush failed on shutdown: %s",
                     component_.c_str(), path.c_str());
      else
        RCLCPP_INFO(logger_, "[%s] CSV log closed on shutdown: %s (%zu rows)",
                    component_.c_str(), path.c_str(), row_idx);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown exception: %s",
                 component_.c_str(), e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "[%s] CSV writer shutdown unknown exception",
                 component_.c_str());
  }
}

// ============================================================
// TrajectoryTuner
// ============================================================
TrajectoryTuner::TrajectoryTuner(rclcpp::Logger logger,
                                 std::string hw_name,
                                 OnTuner on_tuner,
                                 OnCsv on_csv)
  : logger_(logger),
    hw_name_(std::move(hw_name)),
    on_tuner_(std::move(on_tuner)),
    on_csv_(std::move(on_csv))
{}

TrajectoryTuner::~TrajectoryTuner() { stop(); }

void TrajectoryTuner::start()
{
  if (node_) return;

  const std::string node_name = hw_name_ + "_tuner";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=" + node_name});
  options.parameter_overrides({});
  options.use_global_arguments(true);

  node_ = std::make_shared<rclcpp::Node>(node_name, options);

  const std::string ns            = "/" + hw_name_;
  const std::string tuner_topic   = ns + "/trajectory_tuner";
  const std::string delta_topic   = ns + "/cmd_delta";
  const std::string timing_topic  = ns + "/write_timing";
  const std::string csv_topic     = ns + "/csv_enable";

  delta_pub_  = node_->create_publisher<std_msgs::msg::Float32MultiArray>(delta_topic,  1);
  timing_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(timing_topic, 1);

  tuner_sub_ = node_->create_subscription<std_msgs::msg::Int16MultiArray>(
    tuner_topic, 1,
    [this](std_msgs::msg::Int16MultiArray::SharedPtr msg) {
      if (msg->data.size() < 2) {
        RCLCPP_WARN(logger_,
          "[%s] trajectory_tuner: expected [mode, radio], got %zu element(s)",
          hw_name_.c_str(), msg->data.size());
        return;
      }
      const int16_t raw_mode = msg->data[0];
      if (raw_mode < 0 || raw_mode > 2) {
        RCLCPP_WARN(logger_,
          "[%s] trajectory_mode=%d out of range (valid 0,1,2); ignoring",
          hw_name_.c_str(), static_cast<int>(raw_mode));
        return;
      }
      const int32_t raw_radio = msg->data[1];
      uint16_t clamped;
      if (raw_mode == 1)
        clamped = static_cast<uint16_t>(std::clamp<int32_t>(raw_radio, 0, 100));
      else
        clamped = static_cast<uint16_t>(std::clamp<int32_t>(raw_radio, 0, 999));

      if (on_tuner_) on_tuner_(static_cast<uint8_t>(raw_mode), clamped);
    });

  csv_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    csv_topic, 1,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      RCLCPP_INFO(logger_, "[%s] csv_enable -> %s",
                  hw_name_.c_str(), msg->data ? "true" : "false");
      if (on_csv_) on_csv_(msg->data);
    });

  RCLCPP_INFO(logger_, "[%s] write_timing publisher on %s",
              hw_name_.c_str(), timing_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] cmd_delta publisher on %s",
              hw_name_.c_str(), delta_topic.c_str());
  RCLCPP_INFO(logger_, "[%s] csv_enable subscriber on %s",
              hw_name_.c_str(), csv_topic.c_str());

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  running_.store(true);
  thread_ = std::thread([this]() {
    while (rclcpp::ok() && running_.load())
      exec_->spin_once(std::chrono::milliseconds(10));
  });

  RCLCPP_INFO(logger_, "[%s] tuner node '%s' started (topic: %s)",
              hw_name_.c_str(), node_name.c_str(), tuner_topic.c_str());
}

void TrajectoryTuner::stop()
{
  if (!node_) return;
  running_.store(false);
  if (exec_) exec_->cancel();
  if (thread_.joinable()) thread_.join();

  timing_pub_.reset();
  delta_pub_.reset();
  tuner_sub_.reset();
  csv_sub_.reset();
  exec_.reset();
  node_.reset();

  RCLCPP_INFO(logger_, "[%s] tuner node stopped", hw_name_.c_str());
}

void TrajectoryTuner::publish_delta(const std::vector<float>& current,
                                    const std::vector<float>& last)
{
  if (!delta_pub_) return;
  if (++delta_sub_check_tick_ >= 100) {
    delta_sub_check_tick_ = 0;
    delta_has_sub_ = (delta_pub_->get_subscription_count() > 0);
  }
  if (!delta_has_sub_ || current.size() != last.size()) return;

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(current.size());
  for (size_t k = 0; k < current.size(); ++k)
    msg.data[k] = current[k] - last[k];
  delta_pub_->publish(msg);
}

void TrajectoryTuner::publish_timing(double call_ms, double period_ms,
                                     double threshold_ms, double actual_dt_ms)
{
  if (!timing_pub_) return;
  if (++timing_sub_check_tick_ >= 100) {
    timing_sub_check_tick_ = 0;
    timing_has_sub_ = (timing_pub_->get_subscription_count() > 0);
  }
  if (!timing_has_sub_) return;

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = static_cast<float>(call_ms);
  msg.data[1] = static_cast<float>(period_ms);
  msg.data[2] = static_cast<float>(threshold_ms);
  msg.data[3] = static_cast<float>(actual_dt_ms);
  timing_pub_->publish(msg);
}

// ============================================================
// WriteTimingStats
// ============================================================
bool WriteTimingStats::update(double actual_dt_ms, double call_ms,
                              double period_ms,
                              double& dt_mean, double& dt_sd,
                              double& dt_min,  double& dt_max,
                              double& cl_mean, double& cl_max,
                              size_t& n)
{
  dt_history_  [idx_] = actual_dt_ms;
  call_history_[idx_] = call_ms;

  ++idx_;
  if (filled_ < dt_history_.size()) ++filled_;

  if (idx_ < dt_history_.size()) return false;
  idx_ = 0;

  double dt_sum = 0.0, dt_sum2 = 0.0;
  double cl_sum = 0.0;
  dt_max = 0.0; dt_min = 1e9; cl_max = 0.0;
  n = filled_;

  for (size_t i = 0; i < n; ++i) {
    const double d = dt_history_[i];
    const double c = call_history_[i];
    dt_sum  += d;
    dt_sum2 += d * d;
    if (d > dt_max) dt_max = d;
    if (d < dt_min) dt_min = d;
    cl_sum += c;
    if (c > cl_max) cl_max = c;
  }

  dt_mean = dt_sum / n;
  const double dt_var = std::max(0.0, dt_sum2 / n - dt_mean * dt_mean);
  dt_sd   = std::sqrt(dt_var);
  cl_mean = cl_sum / n;

  const bool mean_drift = std::abs(dt_mean - period_ms) > (period_ms * 0.15);
  const bool high_sd    = dt_sd > (period_ms * 0.10);
  const bool tail_spike = dt_max > (period_ms * 2.0);

  return mean_drift || high_sd || tail_spike;
}

// ============================================================
// HardwareParams
// ============================================================
bool HardwareParams::has(const std::string& key) const
{
  auto it = info_.hardware_parameters.find(key);
  return it != info_.hardware_parameters.end() && !it->second.empty();
}

const std::string* HardwareParams::get(const std::string& key) const
{
  auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end() || it->second.empty()) 
    return nullptr;
  return &it->second;
}

bool HardwareParams::get_bool(const std::string& key, bool def) const
{
  const auto* v = get(key);
  if (!v) 
    return def;
  return (*v == "true" || *v == "True" || *v == "TRUE" || *v == "1");
}

int HardwareParams::get_int(const std::string& key, int def) const
{
  const auto* v = get(key);
  if (!v) 
    return def;
  try 
  { 
    return std::stoi(*v); 
  } 
  catch (...) 
  { 
    return def; 
  }
}

uint16_t HardwareParams::get_u16(const std::string& key, uint16_t def) const
{
  return static_cast<uint16_t>(get_int(key, static_cast<int>(def)));
}

uint32_t HardwareParams::get_u32(const std::string& key, uint32_t def) const
{
  const auto* v = get(key);
  if (!v) 
    return def;
  try 
  { 
    return static_cast<uint32_t>(std::stoul(*v)); 
  } 
  catch (...) 
  { 
    return def; 
  }
}

uint64_t HardwareParams::get_u64(const std::string& key, uint64_t def) const
{
  const auto* v = get(key);
  if (!v) 
    return def;
  try 
  { 
    return static_cast<uint64_t>(std::stoull(*v)); 
  } 
  catch (...) 
  { 
    return def; 
  }
}

double HardwareParams::get_double(const std::string& key, double def) const
{
  const auto* v = get(key);
  if (!v) 
    return def;
  try 
  { 
    return std::stod(*v); 
  } 
  catch (...) 
  { 
    return def; 
  }
}

std::string HardwareParams::get_string(const std::string& key, const std::string& def) const
{
  const auto* v = get(key);
  return v ? *v : def;
}

void TrajectoryParams::load(const HardwareParams& params)
{
  const uint16_t raw_mode  = params.get_u16("trajectory_mode",  2);
  const uint16_t raw_radio = params.get_u16("trajectory_radio", 500);

  mode.store(static_cast<uint8_t>(std::min<uint16_t>(raw_mode, 2)),
             std::memory_order_release);
  radio.store(raw_radio, std::memory_order_release);
}

// ============================================================
// DualArmsLifecycleBase
// ============================================================
DualArmsLifecycleBase::CallbackReturn
DualArmsLifecycleBase::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  logger_ = rclcpp::get_logger(std::string(get_name()) + "_hardware_interface");
  return on_init_impl();
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_configure(const rclcpp_lifecycle::State&)
{ 
  return on_configure_impl(); 
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_activate(const rclcpp_lifecycle::State&)
{ 
  return on_activate_impl(); 
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_deactivate(const rclcpp_lifecycle::State&)
{ 
  return on_deactivate_impl(); 
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_cleanup(const rclcpp_lifecycle::State&)
{ 
  return on_cleanup_impl(); 
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_shutdown(const rclcpp_lifecycle::State&)
{ 
  return on_shutdown_impl(); 
}

DualArmsLifecycleBase::CallbackReturn DualArmsLifecycleBase::on_error(const rclcpp_lifecycle::State&)
{ 
  return on_error_impl();
}

}  // namespace dual_arm_hardware_interface