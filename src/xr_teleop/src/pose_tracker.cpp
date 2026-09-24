#include <thread>
#include <atomic>
#include <string>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>

#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>

#include "xr_teleop/arm_types.hpp"

namespace {
class ScopedTimer
{
public:
  ScopedTimer(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
              const char * name, double warn_ms = 0.0)
  : logger_(logger), clock_(clock), name_(name),
    start_(std::chrono::steady_clock::now()),
    warn_ms_(warn_ms) {}

  double elapsed_ms() const
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_).count();
  }

  ~ScopedTimer()
  {
    const double ms = elapsed_ms();
    if (warn_ms_ > 0.0 && ms > warn_ms_)
    {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "[timing] %s took %.2f ms (threshold %.2f)", name_, ms, warn_ms_);
    }
  }

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  const char * name_;
  std::chrono::steady_clock::time_point start_;
  double warn_ms_;
};
} // namespace

namespace pose_tracker
{

class PoseTrackerWrapper : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PoseTrackerWrapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("pose_tracker", options),
    options_(options),
    ready_(false),
    stop_tracking_(false),
    last_status_(0),
    singularity_halt_(false)
  {
    declare_parameter<std::string>("side", "");
    declare_parameter<double>("lin_tol_x", 0.01);
    declare_parameter<double>("lin_tol_y", 0.01);
    declare_parameter<double>("lin_tol_z", 0.01);
    declare_parameter<double>("rot_tol", 0.05);
    declare_parameter<double>("target_pose_timeout", 0.004);
    declare_parameter<double>("loop_rate", 100.0);

    RCLCPP_INFO(this->get_logger(), "PoseTracker component instantiated.");
  }

  ~PoseTrackerWrapper()
  {
    shutdown_threads();
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const std::string side = get_parameter("side").as_string();
    if (side.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "Could not get arm side parameter!");
      return CallbackReturn::FAILURE;
    }

    // 1. Create the MoveIt helper node (passes parameter options down to MoveIt)
    rclcpp::NodeOptions helper_options = options_;
    helper_options.arguments(std::vector<std::string>{});

    const std::string helper_name = std::string(this->get_name()) + "_helper";
    const std::string helper_ns   = std::string(this->get_namespace());

    moveit_node_ = std::make_shared<rclcpp::Node>(helper_name, helper_ns, helper_options);

    // Spin the helper node in the background for MoveIt TF and Joint States
    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);
    moveit_spin_thread_ = std::thread(&PoseTrackerWrapper::moveit_spin_loop, this);
    
    // 2. Initialize MoveIt dependencies using the helper node
    servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(moveit_node_);
    if (!servo_parameters_)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not get servo parameters! Check YAML parameter namespaces.");
      return CallbackReturn::FAILURE;
    }

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(moveit_node_, "robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(this->get_logger(), "Error setting up the PlanningSceneMonitor.");
      return CallbackReturn::FAILURE;
    }

    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor(servo_parameters_->joint_topic);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    if (!planning_scene_monitor_->waitForCurrentRobotState(moveit_node_->get_clock()->now(), 10.0))
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for current robot state.");
      return CallbackReturn::FAILURE;
    }

    tracker_ = std::make_unique<moveit_servo::PoseTracking>(moveit_node_, servo_parameters_, planning_scene_monitor_);
    tracker_->resetTargetPose();

    // 3. Initialize Pubs/Subs
    status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/" + side + "_arm/servo_pose_tracking_helper/status", 
      1,
      std::bind(&PoseTrackerWrapper::status_cb, this, std::placeholders::_1));

    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/" + side + "_arm/ready", 
      1,
      std::bind(&PoseTrackerWrapper::ready_cb, this, std::placeholders::_1));

    warn_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + side + "_arm/warn", 1);
    error_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + side + "_arm/error", 1);

    // CHANGED: the tracking thread is NOT started here anymore. It is started
    // in on_activate and stopped in on_deactivate, so it can never be running
    // while the node is INACTIVE and holding the TF buffer lock during a
    // lifecycle transition on another node (e.g. xr_teleop writing /tf_static).

    RCLCPP_INFO(this->get_logger(), "PoseTracker configured successfully.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    warn_pub_->on_activate();
    error_pub_->on_activate();
    tracker_->resetTargetPose();

    stop_tracking_.store(false);
    stop_moveit_spin_.store(false);
    worker_active_.store(false);
    worker_exit_.store(false);

    if (!tracking_thread_.joinable())
    {
      tracking_thread_ = std::thread(&PoseTrackerWrapper::tracking_loop, this);
    }
    // if (!moveit_spin_thread_.joinable())
    // {
    //   moveit_spin_thread_ = std::thread(&PoseTrackerWrapper::moveit_spin_loop, this);
    // }

    RCLCPP_INFO(this->get_logger(), "PoseTracker activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_tracking_.store(true);

    if (tracker_)
    {
      tracker_->stopMotion();
    }

    if (tracking_thread_.joinable())
    {
      tracking_thread_.join();
    }

    warn_pub_->on_deactivate();
    error_pub_->on_deactivate();

    if (tracker_)
    {
      tracker_->stopMotion();
    }

    ready_.store(false);

    RCLCPP_INFO(this->get_logger(), "PoseTracker deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    shutdown_threads();

    status_sub_.reset();
    ready_sub_.reset();
    warn_pub_.reset();
    error_pub_.reset();

    tracker_.reset();
    planning_scene_monitor_.reset();
    servo_parameters_.reset();
    moveit_node_.reset();

    RCLCPP_INFO(this->get_logger(), "PoseTracker cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    shutdown_threads();
    return CallbackReturn::SUCCESS;
  }

private:
  bool is_active()
  {
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void shutdown_threads()
  {
    // ---- 停 supervisor ----
    stop_tracking_.store(true);
    if (tracker_) 
      tracker_->stopMotion();
    if (tracking_thread_.joinable())
    {
      tracking_thread_.join();
    }

    // ---- 停 moveit spin ----
    stop_moveit_spin_.store(true);
    if (moveit_executor_)
    {
      moveit_executor_->cancel();
    }
    if (moveit_spin_thread_.joinable())
    {
      moveit_spin_thread_.join();
    }
  }

  void status_cb(const std_msgs::msg::Int8::SharedPtr msg)
  {
    last_status_.store(msg->data);
    if (!is_active()) 
      return;

    using moveit_servo::StatusCode;
    std_msgs::msg::Bool warn_msg;
    const int64_t now_ns = this->now().nanoseconds();

    switch (static_cast<StatusCode>(msg->data))
    {
      case StatusCode::NO_WARNING:
        warn_msg.data = false;
        if (singularity_halt_.load()) 
        {
          if (singular_exit_ns_ < 0) 
          {
            singular_exit_ns_ = now_ns;         
          } 
          else if ((now_ns - singular_exit_ns_) > 500'000'000LL) // 0.5s cooldown
          {  
            singularity_halt_.store(false);
            singular_exit_ns_ = -1;
            RCLCPP_INFO(this->get_logger(), "Singularity cleared (auto), resuming pose tracking.");
          }
        } 
        else 
        {
          singular_exit_ns_ = -1;
        }
        break;

      case StatusCode::HALT_FOR_SINGULARITY:
        warn_msg.data = true;
        singular_exit_ns_ = -1;
        if (!singularity_halt_.exchange(true))
        {
          RCLCPP_WARN(this->get_logger(), "HALT_FOR_SINGULARITY — pausing pose tracking.");
          if (tracker_) 
            tracker_->stopMotion();
          // supervisor 会在下一个 tick 把 worker_active_ 置 false
        }
        break;

      case StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY:
      case StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY:
      case StatusCode::DECELERATE_FOR_COLLISION:
      case StatusCode::HALT_FOR_COLLISION:
      case StatusCode::JOINT_BOUND:
        warn_msg.data = true;
        break;

      default:
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unknown status code: %d", msg->data);
        warn_msg.data = true;
        break;
    }

    warn_pub_->publish(warn_msg);
  }

  void ready_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool now_ready = msg->data;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "now_ready: %s", now_ready ? "true" : "false");

    if (!now_ready && prev_ready_)
    {
      // 立即打断正在跑的 moveToPose
      if (tracker_) 
        tracker_->stopMotion();
      RCLCPP_INFO(this->get_logger(), "stopMotion() issued from ready_cb");
      // supervisor 会在下一个 tick 把 worker_active_ 置 false 并通知 cv
    }
    else if (now_ready && !prev_ready_)
    {
      if (singularity_halt_.exchange(false))
      {
        RCLCPP_INFO(this->get_logger(), "Singularity halt cleared by ready press");
      }
      singular_exit_ns_ = -1;
      // supervisor 会在下一个 tick 把 worker_active_ 置 true
    }

    prev_ready_ = now_ready;
    ready_.store(now_ready);
  }

  void moveit_spin_loop()
  {
    RCLCPP_INFO(this->get_logger(), "moveit_spin_loop started.");
    const auto spin_timeout = std::chrono::milliseconds(10);
    constexpr double warn_threshold_ms = 30.0;   // 30ms 以上才算异常

    while (rclcpp::ok() && !stop_moveit_spin_.load())
    {
      const auto before = std::chrono::steady_clock::now();

      try
      {
        moveit_executor_->spin_once(spin_timeout);
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "moveit_spin_loop exception: %s", e.what());
      }

      const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - before).count();

      if (elapsed_ms > warn_threshold_ms)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "spin callback took %.1f ms", elapsed_ms);
      }
    }
    RCLCPP_INFO(this->get_logger(), "moveit_spin_loop exiting.");
  }
  
  void tracking_loop()
  {
    const double rate = get_parameter("loop_rate").as_double();
    rclcpp::Rate loop_rate(rate);
    const double period_ms = 1000.0 / rate;

    RCLCPP_INFO(this->get_logger(), "Tracking supervisor started (rate=%.1f Hz)", rate);

    // ---- 启动 worker 线程（一次） ----
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_active_.store(false);
      worker_exit_.store(false);
    }
    tracking_worker_ = std::thread(&PoseTrackerWrapper::worker_loop, this);

    // ---- Supervisor 主循环 ----
    auto last_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !stop_tracking_.load())
    {
      const auto loop_start = std::chrono::steady_clock::now();
      const double elapsed_ms = std::chrono::duration<double, std::milli>(loop_start - last_time).count();
      last_time = loop_start;

      if (elapsed_ms > period_ms * 2.0)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[supervisor] period overrun: %.1f ms (target %.1f ms)",
          elapsed_ms, period_ms);
      }

      if (!is_active())
      {
        loop_rate.sleep();
        continue;
      }

      const bool should_track = ready_.load() && !singularity_halt_.load();
      const bool is_tracking  = worker_active_.load();

      if (should_track && !is_tracking)
      {
        // ---- 激活 worker ----
        {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          worker_active_.store(true);
        }
        worker_cv_.notify_all();
        RCLCPP_INFO(this->get_logger(), "Tracking worker activated");
      }
      else if (!should_track && is_tracking)
      {
        // ---- 停掉 worker + 打断 moveToPose ----
        {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          worker_active_.store(false);
        }
        if (tracker_) 
          tracker_->stopMotion();

        worker_cv_.notify_all();
        RCLCPP_INFO(this->get_logger(), "Tracking worker deactivated");
      }
      else if (!should_track)
      {
        // 未激活：打一条低频提示
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "Waiting to activate. Tracker status: [%d]",
          static_cast<int>(last_status_.load()));
      }

      loop_rate.sleep();
    }

    // ---- Supervisor 退出：先停 worker，再 join ----
    RCLCPP_INFO(this->get_logger(), "Tracking supervisor shutting down...");
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_active_.store(false);
      worker_exit_.store(true);
    }

    if (tracker_) 
      tracker_->stopMotion();
    worker_cv_.notify_all();

    if (tracking_worker_.joinable())
    {
      tracking_worker_.join();
    }

    RCLCPP_INFO(this->get_logger(), "tracking_loop end");
  }

  void worker_loop()
  {
    // ---- 参数只读一次 ----
    const Eigen::Vector3d lin_tol{
      get_parameter("lin_tol_x").as_double(),
      get_parameter("lin_tol_y").as_double(),
      get_parameter("lin_tol_z").as_double()};
    const double rot_tol             = get_parameter("rot_tol").as_double();
    const double target_pose_timeout = get_parameter("target_pose_timeout").as_double();

    RCLCPP_INFO(this->get_logger(), "Tracking worker started");

    while (rclcpp::ok() && !worker_exit_.load())
    {
      // ---- 等待 supervisor 激活 ----
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_cv_.wait(lock, [&] {
          return worker_active_.load() || worker_exit_.load();
        });
      }

      if (worker_exit_.load()) 
        break;

      // ---- activate moveToPose ----
      while (worker_active_.load() && !worker_exit_.load() && rclcpp::ok())
      {
        const auto t0 = std::chrono::steady_clock::now();

        moveit_servo::PoseTrackingStatusCode code =
          moveit_servo::PoseTrackingStatusCode::INVALID;
        try
        {
          code = tracker_->moveToPose(lin_tol, rot_tol, target_pose_timeout);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "moveToPose threw: %s", e.what());
          break;
        }
        catch (...)
        {
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "moveToPose unknown exception");
          break;
        }

        const double dur_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0).count();

        switch (code)
        {
          case moveit_servo::PoseTrackingStatusCode::SUCCESS:
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "moveToPose SUCCESS (%.1f ms)", dur_ms);
            break;
          case moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED:
            // stopMotion 被调用 → 正常退出
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "moveToPose STOP_REQUESTED (%.1f ms)", dur_ms);
            break;
          case moveit_servo::PoseTrackingStatusCode::NO_RECENT_TARGET_POSE:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "moveToPose NO_RECENT_TARGET_POSE (%.1f ms)", dur_ms);
            break;
          case moveit_servo::PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "moveToPose NO_RECENT_END_EFFECTOR_POSE (%.1f ms)", dur_ms);
            break;
          case moveit_servo::PoseTrackingStatusCode::INVALID:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
              "moveToPose INVALID (%.1f ms)", dur_ms);
            break;
        }

        // STOP_REQUESTED 后立即退出内循环；否则再看 worker_active_
        if (code == moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED) 
          break;
      }

      // 每次会话结束后重置目标
      if (!worker_exit_.load() && tracker_)
      {
        tracker_->resetTargetPose();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Tracking worker exiting");
  }

  rclcpp::NodeOptions options_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr warn_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr error_pub_;

  // MoveIt Helper Node & Executor
  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
  std::thread moveit_spin_thread_;

  std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::PoseTracking> tracker_;

  std::thread             tracking_thread_;

  std::thread             tracking_worker_;
  std::mutex              worker_mutex_;
  std::condition_variable worker_cv_;
  std::atomic<bool>       worker_active_{false};   // 是否要跑 moveToPose
  std::atomic<bool>       worker_exit_{false};     // 是否要退出 worker 线程
  
  std::atomic<bool> ready_;
  std::atomic<bool> stop_tracking_;
  std::atomic<int8_t> last_status_;
  std::atomic<bool> singularity_halt_;
  std::atomic<int64_t> singular_exit_ns_{-1};
  std::atomic<bool> stop_moveit_spin_{false};
  bool prev_ready_{false};
};

} // namespace pose_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(pose_tracker::PoseTrackerWrapper)