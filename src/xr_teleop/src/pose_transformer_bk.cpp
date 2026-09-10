#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <xr_teleop_msgs/msg/button.hpp>

#include <cmath>

class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer()
  : Node("xr_teleop_node")
  {
    xr_to_ros_rot = {
      0.0,  0.0, -1.0,
      -1.0,  0.0,  0.0,
      0.0,  1.0,  0.0
    };
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    left_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/left_arm/ready", 10);
    right_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/right_arm/ready", 10);

    // Create subscribers
    head_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr/raw_head_pose", 50, std::bind(&PoseTransformer::head_pose_cb, this, std::placeholders::_1));
      
    left_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr/raw_left_arm", 50, std::bind(&PoseTransformer::left_pose_cb, this, std::placeholders::_1));
      
    right_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr/raw_right_arm", 50, std::bind(&PoseTransformer::right_pose_cb, this, std::placeholders::_1));

    left_btn_a_sub_ = this->create_subscription<xr_teleop_msgs::msg::Button>(
      "/xr/left_a_button", 10, std::bind(&PoseTransformer::left_a_btn_cb, this, std::placeholders::_1));

    right_btn_a_sub_ = this->create_subscription<xr_teleop_msgs::msg::Button>(
      "/xr/right_a_button", 10, std::bind(&PoseTransformer::right_a_btn_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node initialized. Broadcasting TF...");
  }

private:
  void head_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Define a 90 degree (pi/2) rotation around the Z-axis
    tf2::Quaternion q_rot;
    q_rot.setRPY(0.0, M_PI, M_PI); 

    // Extract the original orientation
    tf2::Quaternion q_orig;
    tf2::fromMsg(msg->pose.orientation, q_orig);

    // Apply the rotation (q_orig * q_rot applies rotation in the local frame)
    // If you need it rotated in the global frame, use: q_new = q_rot * q_orig;
    tf2::Quaternion q_new = q_orig * q_rot;
    q_new.normalize(); // Always normalize after multiplication

    // Update the message orientation
    msg->pose.orientation = tf2::toMsg(q_new);

    broadcast_tf_head(msg, "base_link", "head_frame");
  }

  void left_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    broadcast_tf_hand(msg, "head_frame", "left_hand_frame");
  }

  void right_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    broadcast_tf_hand(msg, "head_frame", "right_hand_frame");
  }

  void left_a_btn_cb(const xr_teleop_msgs::msg::Button::SharedPtr msg)
  {
    const bool is_pressed = msg->pressed;

    if (is_pressed && !left_was_pressed_) 
    {
      // Button just got pressed → record the start time
      left_press_start_time_ = this->now();
    }
    else if (is_pressed && left_was_pressed_) 
    {
      // Still holding → check duration
      double held_sec = (this->now() - left_press_start_time_).seconds();
      if (held_sec >= BUTTON_HOLD_DURATION) 
      {
        left_enable_ = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Left arm ENABLED");
      }
    }
    else if (!is_pressed && left_was_pressed_) 
    {
      // Button released → reset
      left_enable_ = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Left arm DISABLED");
    }

    left_was_pressed_ = is_pressed;

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = left_enable_;
    left_ready_pub_->publish(ready_msg);
  }

  void right_a_btn_cb(const xr_teleop_msgs::msg::Button::SharedPtr msg)
  {
    const bool is_pressed = msg->pressed;

    if (is_pressed && !right_was_pressed_) 
    {
      right_press_start_time_ = this->now();
    }
    else if (is_pressed && right_was_pressed_) 
    {
      double held_sec = (this->now() - right_press_start_time_).seconds();
      if (held_sec >= BUTTON_HOLD_DURATION) 
      {
        right_enable_ = true;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Right arm ENABLED");
      }
    }
    else if (!is_pressed && right_was_pressed_) 
    {
      right_enable_ = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Right arm DISABLED");
    }

    right_was_pressed_ = is_pressed;
    
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = right_enable_;
    right_ready_pub_->publish(ready_msg);
  }

  void broadcast_tf_head(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg, 
    const std::string& parent_frame_id,
    const std::string& child_frame_id)
  {
    geometry_msgs::msg::TransformStamped t;

    // Use the timestamp and parent frame from the incoming PoseStamped
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_id;
    t.child_frame_id = child_frame_id;

    // Copy Translation
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;

    // Copy Rotation
    t.transform.rotation = msg->pose.orientation;

    // Broadcast
    tf_broadcaster_->sendTransform(t);
  }

  void broadcast_tf_hand(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg, 
    const std::string& parent_frame_id,
    const std::string& child_frame_id)
  {
    tf2::Transform raw_pose;
    tf2::fromMsg(msg->pose, raw_pose);

    tf2::Transform xr_to_ros;
    xr_to_ros.setBasis(xr_to_ros_rot);
    xr_to_ros.setOrigin(tf2::Vector3(0, 0, 0));

    tf2::Transform ros_pose = xr_to_ros * raw_pose * xr_to_ros.inverse();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_id;
    t.child_frame_id = child_frame_id;

    t.transform.translation.x = ros_pose.getOrigin().x();
    t.transform.translation.y = ros_pose.getOrigin().y();
    t.transform.translation.z = ros_pose.getOrigin().z();

    t.transform.rotation = tf2::toMsg(ros_pose.getRotation());

    tf_broadcaster_->sendTransform(t);
  }

  // Member variables

  bool left_enable_;
  bool right_enable_;

  rclcpp::Time left_press_start_time_;
  rclcpp::Time right_press_start_time_;
  bool left_was_pressed_ = false;
  bool right_was_pressed_ = false;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_ready_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_sub_;
  rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr left_btn_a_sub_;
  rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr right_btn_a_sub_;

  // Define the rotation matrix to map OpenXR axes to ROS axes
  // ROS X (Forward) = -XR Z
  // ROS Y (Left)    = -XR X
  // ROS Z (Up)      =  XR Y
  tf2::Matrix3x3 xr_to_ros_rot;

  static constexpr double BUTTON_HOLD_DURATION = 1.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}