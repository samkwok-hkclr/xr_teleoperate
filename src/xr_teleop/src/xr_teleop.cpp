#include <memory>
#include <chrono>
#include <unordered_map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rcpputils/join.hpp>

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

namespace xr_teleop
{

using std::placeholders::_1;
using std::placeholders::_2;

class XrTeleop : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit XrTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("xr_teleop", options)
  {
    declare_parameter<double>("x_scale_factor", 1.0);
    declare_parameter<double>("y_scale_factor", 1.0);
    declare_parameter<double>("z_scale_factor", 1.0);
    declare_parameter<double>("target_pose_max_age_s", 0.5);
    declare_parameter<std::vector<std::string>>("xr_reference_data", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("tcp", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("k_sources", std::vector<std::string>{});
    
    RCLCPP_INFO(this->get_logger(), "XrTeleop node instantiated.");
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    max_age_s_ = get_parameter("target_pose_max_age_s").as_double();

    // Pull every parameter up front so all failures can report actual values.
    const auto tcp           = get_parameter("tcp").as_string_array();
    const auto k_sources     = get_parameter("k_sources").as_string_array();
    const auto xr_ref_data   = get_parameter("xr_reference_data").as_string_array();

    // --- tcp / k_sources: non-empty ---
    if (tcp.empty())
    {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'tcp' is empty. Expected at least 2 entries (one per arm). "
        "Got %zu.", tcp.size());
      return CallbackReturn::FAILURE;
    }
    if (k_sources.empty())
    {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'k_sources' is empty. Expected at least 2 entries (one per arm). "
        "Got %zu.", k_sources.size());
      return CallbackReturn::FAILURE;
    }

    // --- tcp / k_sources: exactly 2 ---
    if (tcp.size() < 2)
    {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'tcp' has %zu entries but this node manages a dual-arm setup "
        "and requires 2. Values: [%s].",
        tcp.size(), rcpputils::join(tcp, ", ").c_str());
      return CallbackReturn::FAILURE;
    }
    if (k_sources.size() < 2)
    {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'k_sources' has %zu entries but this node manages a dual-arm setup "
        "and requires 2. Values: [%s].",
        k_sources.size(), rcpputils::join(k_sources, ", ").c_str());
      return CallbackReturn::FAILURE;
    }
    if (xr_ref_data.size() < 8)
    {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'xr_reference_data' has %zu entries but requires at least 8 "
        "(parent_frame, child_frame, x, y, z, roll, pitch, yaw). Values: [%s].",
        xr_ref_data.size(), rcpputils::join(xr_ref_data, ", ").c_str());
      return CallbackReturn::FAILURE;
    }

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

      tcp_[arm] = tcp[i - 1];
      k_sources_[arm] = k_sources[i - 1];

      target_pose_pub_[arm] = this->create_publisher<geometry_msgs::msg::PoseStamped>("/" + side + "_cartesian_motion_controller/target_frame", 1);
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

    // Create a 100Hz timer, but keep it cancelled until activated
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), 
      std::bind(&XrTeleop::tf_timer_cb, this));
    tf_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "XrTeleop configured.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Activate Lifecycle Publishers
    for (auto& [arm, pub] : target_pose_pub_) 
      pub->on_activate();
    for (auto& [arm, pub] : ready_pub_) 
      pub->on_activate();

    broadcast_static_tf();
    
    // Start listening and publishing TF
    tf_timer_->reset();

    RCLCPP_INFO(this->get_logger(), "XrTeleop activated.");
    return CallbackReturn::SUCCESS;
  }

  // I cannot call ros2 lifecycle set /xr_teleop deactivate. why?
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // Pause operations
    tf_timer_->cancel();

    for (auto& [arm, is_enabled] : enabled_) 
      is_enabled = false;
    for (auto& [arm, pressed] : was_pressed_) 
      pressed = false;
    for (auto& [arm, init] : is_initial_recorded_)
      init = false;

    // Deactivate Lifecycle Publishers
    for (auto& [arm, pub] : target_pose_pub_) 
      pub->on_deactivate();
    for (auto& [arm, pub] : ready_pub_) 
      pub->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "XrTeleop deactivated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    // Free resources
    tf_timer_.reset();
    head_sub_.reset();
    arm_pose_sub_.clear();
    btn_a_sub_.clear();
    target_pose_pub_.clear();
    ready_pub_.clear();

    // Clear state tracking maps
    is_initial_recorded_.clear();
    t_begin_.clear();
    t_zero_.clear();
    press_start_time_.clear();
    was_pressed_.clear();
    enabled_.clear();

    tf_static_broadcaster_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    RCLCPP_INFO(this->get_logger(), "XrTeleop cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(this->get_logger(), "XrTeleop shutting down.");
    return CallbackReturn::SUCCESS;
  }

private:
  bool is_active()
  {
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void tf_timer_cb()
  {
    if (!is_active())
      return;

    const rclcpp::Time now = this->get_clock()->now();
    
    for (const auto & [arm, frame] : k_sources_)
    {
      if (frame.empty())
      {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "traget frame is empty");
        continue;
      }
      auto tf_opt = get_tf("base_link", frame);

      if (!tf_opt.has_value())
        continue;

      const auto & tf = tf_opt.value();

      // Freshness
      const double age_s = (now - tf.header.stamp).seconds();
      if (age_s < 0.0 || age_s > max_age_s_)
      {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Dropping %s target: TF age %.3f s", frame.c_str(), age_s);
        continue;
      }

      // Validity
      // if (!is_transform_valid(tf.transform))
      // {
      //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      //     "Dropping %s target: invalid transform", frame.c_str());
      //   continue;
      // }

      // Build and publish (unique_ptr -> move overload -> IPC-friendly)
      auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
      msg->header.stamp    = tf.header.stamp;
      msg->header.frame_id = "base_link";
      msg->pose.position.x = tf.transform.translation.x;
      msg->pose.position.y = tf.transform.translation.y;
      msg->pose.position.z = tf.transform.translation.z;
      msg->pose.orientation = tf.transform.rotation;

      target_pose_pub_[arm]->publish(std::move(msg));
    }

    // auto left_tf_opt = get_tf("base_link", "left_hand_frame");
    // if (left_tf_opt.has_value())
    // {
    //   const geometry_msgs::msg::TransformStamped& left_tf = left_tf_opt.value();
      
    //   auto left_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    //   left_pose->header.stamp = this->get_clock()->now();
    //   left_pose->header.frame_id = "base_link";
    //   left_pose->pose.position.x = left_tf.transform.translation.x;
    //   left_pose->pose.position.y = left_tf.transform.translation.y;
    //   left_pose->pose.position.z = left_tf.transform.translation.z;
    //   left_pose->pose.orientation = left_tf.transform.rotation;
    //   target_pose_pub_[Arm::LEFT]->publish(std::move(left_pose));
    // }

    // auto right_tf_opt = get_tf("base_link", "right_hand_frame");
    // if (right_tf_opt.has_value())
    // {
    //   const geometry_msgs::msg::TransformStamped& right_tf = right_tf_opt.value();

    //   auto right_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    //   right_pose->header.stamp = this->get_clock()->now();
    //   right_pose->header.frame_id = "base_link";
    //   right_pose->pose.position.x = right_tf.transform.translation.x;
    //   right_pose->pose.position.y = right_tf.transform.translation.y;
    //   right_pose->pose.position.z = right_tf.transform.translation.z;
    //   right_pose->pose.orientation = right_tf.transform.rotation;
    //   target_pose_pub_[Arm::RIGHT]->publish(right_pose);
    // }
  }

  std::optional<geometry_msgs::msg::TransformStamped> get_tf(
    const std::string& to_frame, 
    const std::string& from_frame)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;

    try 
    {
      tf_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
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

  void head_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    // if (!is_active()) 
    //   return;
  }

  void arm_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const Arm arm, const std::string& frame)
  {
    if (!is_active()) 
      return;

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

      const std::string ee_frame = tcp_.at(arm); 
      auto t_zero_opt = get_tf("base_link", ee_frame);

      if (t_zero_opt.has_value()) 
      {
        tf2::fromMsg(t_zero_opt.value().transform, t_zero_[arm]);
        is_initial_recorded_[arm] = true;
        RCLCPP_INFO(this->get_logger(), "Recorded %s arm T_begin and T_zero", arm_to_str(arm).c_str());
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
    if (!is_active()) 
      return;

    const bool is_pressed = msg->pressed;

    if (is_pressed && !was_pressed_[arm]) 
    {
      press_start_time_[arm] = this->now();
    }
    else if (is_pressed && was_pressed_[arm]) 
    {
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
    std::vector<std::string> xr_reference_data = this->get_parameter("xr_reference_data").as_string_array();

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = xr_reference_data[0];
    t.child_frame_id = xr_reference_data[1];

    t.transform.translation.x = std::stof(xr_reference_data[2]);
    t.transform.translation.y = std::stof(xr_reference_data[3]);
    t.transform.translation.z = std::stof(xr_reference_data[4]);

    // Convert string to float for RPY and convert to quaternion
    double roll = std::stod(xr_reference_data[5]);
    double pitch = std::stod(xr_reference_data[6]);
    double yaw = std::stod(xr_reference_data[7]);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

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
  double max_age_s_;
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

  std::unordered_map<Arm, rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr> ready_pub_;
  std::unordered_map<Arm, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pose_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> arm_pose_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr> btn_a_sub_;

  std::unordered_map<Arm, std::string> tcp_;
  std::unordered_map<Arm, std::string> k_sources_;
  static constexpr double BUTTON_HOLD_DURATION = 0.5;
};

} // namespace xr_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(xr_teleop::XrTeleop)