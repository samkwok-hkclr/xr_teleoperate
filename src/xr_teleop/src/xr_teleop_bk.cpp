#include <memory>
#include <chrono>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "xr_teleop_msgs/msg/button.hpp"
#include "xr_teleop/arm_types.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class XrTeleop : public rclcpp::Node
{
public:
  XrTeleop()
  : Node("xr_teleop")
  {
    declare_parameter<double>("x_scale_factor", 1.2);
    declare_parameter<double>("y_scale_factor", 1.2);
    declare_parameter<double>("z_scale_factor", 1.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    head_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr/raw_head_pose", 
      rclcpp::SensorDataQoS(), 
      std::bind(&XrTeleop::head_pose_cb, this, std::placeholders::_1));

    for (uint8_t i = static_cast<uint8_t>(Arm::LEFT); i < static_cast<uint8_t>(Arm::LAST); ++i)
    {
      const Arm arm = static_cast<Arm>(i);
      const std::string side = arm_to_str(arm);

      is_initial_recorded_[arm] = false;
      press_start_time_[arm] = this->now();
      was_pressed_[arm] = false;
      enabled_[arm] = false;

      target_pose_pub_[arm] = this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + side + "_arm/target_pose", rclcpp::SensorDataQoS());
      ready_pub_[arm] = this->create_publisher<std_msgs::msg::Bool>("/" + side + "_arm/ready", 1);

      std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> raw_pose_callback = 
        std::bind(&XrTeleop::arm_pose_cb, this, _1, arm, side + "_hand_frame");
      arm_pose_sub_[arm] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/xr/raw_" + side + "_arm", 
        rclcpp::SensorDataQoS(), 
        raw_pose_callback);

      std::function<void(const xr_teleop_msgs::msg::Button::SharedPtr)> btn_a_callback =
        std::bind(&XrTeleop::btn_a_cb, this, _1, arm);
      btn_a_sub_[arm] = this->create_subscription<xr_teleop_msgs::msg::Button>(
        "/xr/" + side + "_a_button", 
        rclcpp::SensorDataQoS(), 
        btn_a_callback);
    }

    // Create a 50Hz timer (20 milliseconds)
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), 
      std::bind(&XrTeleop::tf_timer_cb, this));

    broadcast_static_tf();

    RCLCPP_INFO(this->get_logger(), "XrTeleop node initialized, listening to TF tree at 50Hz...");
  }

  void tf_timer_cb()
  {
    auto left_tf_opt = get_tf("base_link", "left_hand_frame");
    if (left_tf_opt.has_value())
    {
      const geometry_msgs::msg::TransformStamped& left_tf = left_tf_opt.value();
      
      geometry_msgs::msg::PoseStamped left_pose;
      left_pose.header.stamp = this->get_clock()->now();
      left_pose.header.frame_id = "base_link";
      left_pose.pose.position.x = left_tf.transform.translation.x;
      left_pose.pose.position.y = left_tf.transform.translation.y;
      left_pose.pose.position.z = left_tf.transform.translation.z;
      left_pose.pose.orientation = left_tf.transform.rotation;
      target_pose_pub_[Arm::LEFT]->publish(left_pose);
    }

    auto right_tf_opt = get_tf("base_link", "right_hand_frame");
    if (right_tf_opt.has_value())
    {
      const geometry_msgs::msg::TransformStamped& right_tf = right_tf_opt.value();

      geometry_msgs::msg::PoseStamped right_pose;
      right_pose.header.stamp = this->get_clock()->now();
      right_pose.header.frame_id = "base_link";
      right_pose.pose.position.x = right_tf.transform.translation.x;
      right_pose.pose.position.y = right_tf.transform.translation.y;
      right_pose.pose.position.z = right_tf.transform.translation.z;
      right_pose.pose.orientation = right_tf.transform.rotation;
      target_pose_pub_[Arm::RIGHT]->publish(right_pose);
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped> get_tf(
    const std::string& to_frame, 
    const std::string& from_frame)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;

    try 
    {
      tf_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } 
    catch (tf2::TransformException & ex) 
    {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Could not transform %s to %s: %s", to_frame.c_str(), from_frame.c_str(), ex.what());
      return std::nullopt;
    }
    catch (...) 
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Unknown exception in transform %s to %s", to_frame.c_str(), from_frame.c_str());
      return std::nullopt;
    }

    return tf_stamped;
  }

private:
  void head_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {

  }

  void arm_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const Arm arm, const std::string& frame)
  {
    if (!enabled_[arm]) 
    {
      broadcast_tf(msg, "head_frame", frame + "_raw");
      return;
    }

    tf2::Transform t_end;
    tf2::fromMsg(msg->pose, t_end);

    if (!is_initial_recorded_[arm]) 
    {
      t_begin_[arm] = t_end;

      const std::string ee_frame = tcp.at(arm); 
      auto t_zero_opt = get_tf("base_link", ee_frame);

      if (t_zero_opt.has_value()) 
      {
        tf2::fromMsg(t_zero_opt.value().transform, t_zero_[arm]);
        is_initial_recorded_[arm] = true;
        RCLCPP_INFO(this->get_logger(), "Recoeded %s arm T_begin and T_zero", arm_to_str(arm).c_str());
      } 
      else 
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
          "Waiting for %s arm current pose for T_zero...", arm_to_str(arm).c_str());
        return;
      }
    }

    tf2::Transform t_delta = t_begin_[arm].inverse() * t_end;
    tf2::Transform t_result = t_zero_[arm] * t_delta;

    geometry_msgs::msg::PoseStamped result_msg;
    result_msg.header.stamp = msg->header.stamp;
    result_msg.header.frame_id = "base_link"; 
    tf2::toMsg(t_result, result_msg.pose);

    broadcast_tf(std::make_shared<geometry_msgs::msg::PoseStamped>(result_msg), "base_link", frame);
  }

  void btn_a_cb(const xr_teleop_msgs::msg::Button::SharedPtr msg, const Arm arm)
  {
    const bool is_pressed = msg->pressed;

    if (is_pressed && !was_pressed_[arm]) 
    {
      // Button just got pressed → record the start time
      press_start_time_[arm] = this->now();
    }
    else if (is_pressed && was_pressed_[arm]) 
    {
      // Still holding → check duration
      double held_sec = (this->now() - press_start_time_[arm]).seconds();
      if (held_sec >= BUTTON_HOLD_DURATION) 
      {
        if (!enabled_[arm]) 
        {
          enabled_[arm] = true;
          is_initial_recorded_[arm] = false;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "%s arm ENABLED", arm_to_str(arm).c_str());
        }
      }
    }
    else if (!is_pressed && was_pressed_[arm]) 
    {
      // Button released → reset
      enabled_[arm] = false;
      is_initial_recorded_[arm] = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "%s arm DISABLED", arm_to_str(arm).c_str());
    }

    was_pressed_[arm] = is_pressed;

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = enabled_[arm];
    ready_pub_[arm]->publish(ready_msg);
  }

  void broadcast_tf(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg, 
    const std::string& parent_frame_id,
    const std::string& child_frame_id)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_id;
    t.child_frame_id = child_frame_id;

    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  void broadcast_static_tf()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "arm_base_link_l";
    t.child_frame_id = "xr_base_link";

    t.transform.translation.x = 0.07;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.16;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s", 
      t.header.frame_id.c_str(), t.child_frame_id.c_str());

    geometry_msgs::msg::TransformStamped tf_xr_head;
    tf_xr_head.header.stamp = this->get_clock()->now();
    tf_xr_head.header.frame_id = "xr_base_link";
    tf_xr_head.child_frame_id = "head_frame";
    tf_static_broadcaster_->sendTransform(tf_xr_head);
    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s", 
      tf_xr_head.header.frame_id.c_str(), tf_xr_head.child_frame_id.c_str());
  }

  // Member variables
  std::unordered_map<Arm, bool> is_initial_recorded_;
  std::unordered_map<Arm, tf2::Transform> t_begin_;
  std::unordered_map<Arm, tf2::Transform> t_zero_;

  std::unordered_map<Arm, rclcpp::Time> press_start_time_;
  std::unordered_map<Arm, bool> was_pressed_;
  std::unordered_map<Arm, bool> enabled_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::unordered_map<Arm, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> ready_pub_;
  std::unordered_map<Arm, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> arm_pose_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr> btn_a_sub_;

  const std::unordered_map<Arm, std::string> tcp = { 
    { Arm::LEFT , "tcp_l" }, 
    { Arm::RIGHT , "tcp_r"} 
  };
  static constexpr double BUTTON_HOLD_DURATION = 0.5;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XrTeleop>());
  rclcpp::shutdown();
  return 0;
}