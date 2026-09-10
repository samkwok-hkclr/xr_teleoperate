#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>

#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>

#include "xr_teleop/arm_types.hpp"

class PoseTrackerWrapper : public rclcpp::Node
{
public:
  PoseTrackerWrapper(const rclcpp::Node::SharedPtr node, const std::string& node_name)
    : Node(node_name), node_(node), ready_(false), stop_tracking_(false)
  {
    declare_parameter<std::string>("side", "");

    if (get_parameter("side").as_string().empty())
    {
      RCLCPP_FATAL(this->get_logger(), "Could not get arm side");
      rclcpp::shutdown();
    }
    const std::string side = get_parameter("side").as_string();

    status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/" + side + "_arm/servo_pose_tracking/status", 
      1,
      std::bind(&PoseTrackerWrapper::status_cb, this, std::placeholders::_1));

    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/" + side + "_arm/ready", 
      1, 
      std::bind(&PoseTrackerWrapper::ready_cb, this, std::placeholders::_1));

    servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
    if (!servo_parameters_)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not get servo parameters!");
      rclcpp::shutdown();
    }

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(this->get_logger(), "Error setting up the PlanningSceneMonitor.");
      rclcpp::shutdown();
    }

    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor(servo_parameters_->joint_topic);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->get_clock()->now(), 10.0))
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for current robot state.");
      rclcpp::shutdown();
    }

    warn_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + side + "_arm/warn", 1);
    error_pub_ = this->create_publisher<std_msgs::msg::Bool>("/" + side + "_arm/error", 1);

    tracker_ = std::make_unique<moveit_servo::PoseTracking>(node_, servo_parameters_, planning_scene_monitor_);
    tracker_->resetTargetPose();

    tracking_thread_ = std::thread(&PoseTrackerWrapper::tracking_loop, this);
    RCLCPP_INFO(this->get_logger(), "Tracker is up");
  }

  ~PoseTrackerWrapper()
  {
    stop_tracking_.store(true); 

    if (tracking_thread_.joinable())
    {
      tracking_thread_.join();
    }
  }

  void status_cb(const std_msgs::msg::Int8::SharedPtr msg)
  {
    using moveit_servo::StatusCode;
    std_msgs::msg::Bool warn_msg;

    switch (static_cast<StatusCode>(msg->data))
    {
      case StatusCode::NO_WARNING:
        warn_msg.data = false;
        break;
      case StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY:
      case StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY:
      case StatusCode::HALT_FOR_SINGULARITY:
      case StatusCode::DECELERATE_FOR_COLLISION:
      case StatusCode::HALT_FOR_COLLISION:
      case StatusCode::JOINT_BOUND:
        warn_msg.data = true;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown status code: %d", msg->data);
        warn_msg.data = true;
        break;
    }

    warn_pub_->publish(warn_msg);
  }

  void ready_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    ready_.store(msg->data);
  }

private:
  void tracking_loop()
  {
    // Tolerances for reaching the target
    const Eigen::Vector3d lin_tol{0.01, 0.01, 0.01};
    const double rot_tol = 0.05; // degree
    const double timeout = 0.02; // second

    RCLCPP_INFO(this->get_logger(), "Tracker active! Listening for target poses...");
    
    rclcpp::Rate loop_rate(50.0);
    while (rclcpp::ok())
    {
      if (stop_tracking_.load())
      {
        tracker_->stopMotion();
        break;
      }

      if (ready_.load())
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
            RCLCPP_ERROR(this->get_logger(), "NO_RECENT_TARGET_POSE");
            break;
          case moveit_servo::PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE:
            RCLCPP_ERROR(this->get_logger(), "NO_RECENT_END_EFFECTOR_POSE");
            break;
          case moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED:
            RCLCPP_WARN(this->get_logger(), "STOP_REQUESTED");
            break;
        }
      }
      else
      {
        tracker_->resetTargetPose();
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Not ready");
      }

      loop_rate.sleep();
    }


    RCLCPP_INFO(this->get_logger(), "Tracking loop stopped.");
  }

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::PoseTracking> tracker_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr warn_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;

  std::thread tracking_thread_;

  rclcpp::Time ready_time_;

  std::atomic<bool> ready_;
  std::atomic<bool> stop_tracking_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::string helper_name = "pose_tracker_helper";
  std::string tracker_name = "pose_tracker";

  auto node = std::make_shared<rclcpp::Node>(helper_name);
  auto tracker = std::make_shared<PoseTrackerWrapper>(node, tracker_name);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());
  exec->add_node(tracker->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
  return 0;
}