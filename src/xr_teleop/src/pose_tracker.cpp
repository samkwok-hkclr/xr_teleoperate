#include <thread>
#include <atomic>
#include <string>
#include <cstdint>

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
    declare_parameter<double>("lin_tol_x", 0.08);
    declare_parameter<double>("lin_tol_y", 0.08);
    declare_parameter<double>("lin_tol_z", 0.08);
    declare_parameter<double>("rot_tol", 0.05);
    declare_parameter<double>("tracker_timeout", 0.05);
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
    // moveit_spin_thread_ = std::thread{ moveit_executor_->spin(); };

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
      "/" + side + "_arm/servo_pose_tracking_helper/status", 1,
      std::bind(&PoseTrackerWrapper::status_cb, this, std::placeholders::_1));

    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/" + side + "_arm/ready", 1,
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

    if (!tracking_thread_.joinable())
    {
      tracking_thread_ = std::thread(&PoseTrackerWrapper::tracking_loop, this);
    }
    if (!moveit_spin_thread_.joinable())
    {
      moveit_spin_thread_ = std::thread(&PoseTrackerWrapper::moveit_spin_loop, this);
    }

    RCLCPP_INFO(this->get_logger(), "PoseTracker activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop_tracking_.store(true);
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
    stop_tracking_.store(true);
    if (tracking_thread_.joinable())
    {
      tracking_thread_.join();
    }

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
          tracker_->stopMotion();
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

    if (!now_ready) 
    {
      tracker_->stopMotion();
    } 
    else if (!prev_ready_) 
    {
      if (singularity_halt_.exchange(false)) 
      {
        RCLCPP_INFO(this->get_logger(), "Singularity halt manually cleared by ready press.");
      }
      singular_exit_ns_ = -1;
    }

    prev_ready_ = now_ready;
    ready_.store(now_ready);
  }

  void moveit_spin_loop()
  {
    RCLCPP_INFO(this->get_logger(), "moveit_spin_loop started.");

    const auto spin_timeout = std::chrono::milliseconds(10);

    while (rclcpp::ok() && !stop_moveit_spin_.load())
    {
      moveit_executor_->spin_once(spin_timeout);
    }

    RCLCPP_INFO(this->get_logger(), "moveit_spin_loop exiting.");
  }
  

  void tracking_loop()
  {
    // Tolerances for reaching the target
    const Eigen::Vector3d lin_tol{
      get_parameter("lin_tol_x").as_double(),
      get_parameter("lin_tol_y").as_double(), 
      get_parameter("lin_tol_z").as_double()};
    const double rot_tol = get_parameter("rot_tol").as_double();
    const double timeout = get_parameter("tracker_timeout").as_double(); // second

    const double rate = get_parameter("loop_rate").as_double();
    auto last_time = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / rate);
    const auto warn_threshold = period * 1.5;

    rclcpp::Rate loop_rate(rate);
    while (rclcpp::ok())
    {
      const auto loop_start = std::chrono::steady_clock::now();
      const auto elapsed_last = loop_start - last_time;
      last_time = loop_start;

      if (elapsed_last > warn_threshold)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "tracking_loop period overrun: %.1f ms (target %.1f ms)",
          std::chrono::duration<double, std::milli>(elapsed_last).count(),
          period.count() * 1000.0);
      }

      if (stop_tracking_.load())
      {
        tracker_->stopMotion();
        break;
      }

      // CHANGED: this thread only runs while active now, so the
      // !is_active() branch is a safety net rather than the normal path.
      // It also keeps resetTargetPose() from running at 100 Hz while
      // the node is INACTIVE, which is what allowed it to hold the TF
      // buffer lock across a lifecycle transition.
      if (!is_active())
      {
        loop_rate.sleep();
        continue;
      }

      if (ready_.load() && !singularity_halt_.load())
      {
        moveit_servo::PoseTrackingStatusCode code = tracker_->moveToPose(lin_tol, rot_tol, timeout);
        switch (code)
        {
          case moveit_servo::PoseTrackingStatusCode::INVALID:
            RCLCPP_ERROR(this->get_logger(), "INVALID");
            break;
          case moveit_servo::PoseTrackingStatusCode::SUCCESS:
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "SUCCESS");
            break;
          case moveit_servo::PoseTrackingStatusCode::NO_RECENT_TARGET_POSE:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NO_RECENT_TARGET_POSE");
            break;
          case moveit_servo::PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NO_RECENT_END_EFFECTOR_POSE");
            break;
          case moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "STOP_REQUESTED");
            break;
        }
      }
      else if (!ready_.load())
      {
        tracker_->resetTargetPose();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "Wait to activate. Tracker status: [%d]", static_cast<int>(last_status_.load()));
      }
      else
      {
        // singularity_halt_ == true 且 ready_ == true → 保留目标位姿，等恢复
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Singularity halt active, holding target pose.");
      }
      
      loop_rate.sleep();
    }
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

  std::thread tracking_thread_;
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