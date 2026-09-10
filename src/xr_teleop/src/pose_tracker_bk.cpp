
#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <atomic>

class PoseTrackerWrapper
{
public:
  PoseTrackerWrapper(const rclcpp::Node::SharedPtr& node)
    : node_(node), stop_tracking_(false)
  {
  }

  // Destructor ensures the background thread is safely closed on shutdown
  ~PoseTrackerWrapper()
  {
    stopTracking();
  }

  bool init()
  {
    // 1. Load Parameters
    servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
    if (!servo_parameters_)
    {
      RCLCPP_FATAL(node_->get_logger(), "Could not get servo parameters!");
      return false;
    }

    // 2. Setup Planning Scene Monitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(node_->get_logger(), "Error setting up the PlanningSceneMonitor.");
      return false;
    }

    planning_scene_monitor_->providePlanningSceneService();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
    planning_scene_monitor_->startStateMonitor(servo_parameters_->joint_topic);
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5.0))
    {
      RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for current robot state.");
      return false;
    }

    // 3. Create the Pose Tracker
    tracker_ = std::make_unique<moveit_servo::PoseTracking>(node_, servo_parameters_, planning_scene_monitor_);
    tracker_->resetTargetPose();

    return true;
  }

  void startTracking()
  {
    stop_tracking_ = false;
    // Launch the tracking loop in a detached background thread
    tracking_thread_ = std::thread(&PoseTrackerWrapper::trackingLoop, this);
  }

  void stopTracking()
  {
    stop_tracking_ = true; // Signal the loop to stop
    if (tracking_thread_.joinable())
    {
      tracking_thread_.join();
    }
  }

private:
  void trackingLoop()
  {
    // Tolerances for reaching the target
    Eigen::Vector3d lin_tol{0.005, 0.005, 0.005}; // 5mm
    double rot_tol = 0.05; // ~2.8 degrees

    RCLCPP_INFO(node_->get_logger(), "Tracker active! Listening for target poses...");

    // The loop keeps running until ROS shuts down OR we manually trigger stop_tracking_
    while (rclcpp::ok() && !stop_tracking_)
    {
      tracker_->moveToPose(lin_tol, rot_tol, 0.02 /* timeout */);
    }

    RCLCPP_INFO(node_->get_logger(), "Tracking loop stopped.");
  }

  // Member variables
  rclcpp::Node::SharedPtr node_;
//   std::shared_ptr<moveit_servo::ServoParameters> servo_parameters_;
  std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::unique_ptr<moveit_servo::PoseTracking> tracker_;
  
  std::thread tracking_thread_;
  std::atomic<bool> stop_tracking_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("left_pose_tracker");

  PoseTrackerWrapper tracker_node(node);

  // Initialize it (this handles TF and parameters)
  if (!tracker_node.init())
  {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Start the background tracking thread
  tracker_node.startTracking();

  // Use a MultiThreadedExecutor to process incoming ROS callbacks (TF, Pose updates) 
  // while our tracking thread handles the move commands.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Clean shutdown
  tracker_node.stopTracking();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}