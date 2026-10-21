#include <memory>
#include <chrono>
#include <optional>
#include <unordered_map>
#include <string>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rcpputils/join.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "xr_teleop_msgs/msg/button.hpp"
#include "xr_teleop/arm_types.hpp"

#include "robot_controller_msgs/srv/execute_joints.hpp"

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
    declare_parameter<double>("a_button_hold_duration", 0.5);
    declare_parameter<double>("b_button_hold_duration", 3.0);
    declare_parameter<std::string>("reference_frame", "base_link");
    declare_parameter<std::vector<std::string>>("xr_reference_data", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("tcp", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("k_sources", std::vector<std::string>{});

    declare_parameter<double>("b_short_press_min_s", 0.1);
    declare_parameter<double>("b_short_press_max_s", 0.4);

    declare_parameter<std::vector<std::string>>("b_short_press_left_joints",
      std::vector<std::string>{
        "arm_J1_l","arm_J2_l","arm_J3_l","arm_J4_l","arm_J5_l","arm_J6_l","arm_J7_l"});
    declare_parameter<std::vector<double>>("b_short_press_left_positions",
      std::vector<double>{0.69, 0.41, 0.0, 1.15, 0.0, 0.0, 0.0});

    declare_parameter<std::vector<std::string>>("b_short_press_right_joints",
      std::vector<std::string>{
        "arm_J1_r","arm_J2_r","arm_J3_r","arm_J4_r","arm_J5_r","arm_J6_r","arm_J7_r"});
    declare_parameter<std::vector<double>>("b_short_press_right_positions",
      std::vector<double>{0.69, 0.41, 0.0, 1.15, 0.0, 0.0, 0.0});

    RCLCPP_INFO(this->get_logger(), "XrTeleop node instantiated.");
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    max_age_s_ = get_parameter("target_pose_max_age_s").as_double();
    a_button_hold_duration_ = get_parameter("a_button_hold_duration").as_double();
    b_button_hold_duration_ = get_parameter("b_button_hold_duration").as_double();
    ref_frame_ = get_parameter("reference_frame").as_string();

    // Pull every parameter up front so all failures can report actual values.
    const auto tcp         = get_parameter("tcp").as_string_array();
    const auto k_sources   = get_parameter("k_sources").as_string_array();
    const auto xr_ref_data = get_parameter("xr_reference_data").as_string_array();

    b_short_press_min_s_ = get_parameter("b_short_press_min_s").as_double();
    b_short_press_max_s_ = get_parameter("b_short_press_max_s").as_double();
    
    if (b_short_press_min_s_ >= b_short_press_max_s_)
    {
      RCLCPP_FATAL(get_logger(),
        "b_short_press_min_s (%.3f) must be < b_short_press_max_s (%.3f)",
        b_short_press_min_s_, b_short_press_max_s_);
      return CallbackReturn::FAILURE;
    }

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

    // ---- Robot start/shutdown service clients (B-button toggle) ----
    start_robot_cli_    = this->create_client<std_srvs::srv::Trigger>("/start_robot");
    shutdown_robot_cli_ = this->create_client<std_srvs::srv::Trigger>("/shutdown_robot");

    // Latched subscription: the lifecycle manager publishes transient_local,
    // so we get the last known state right away.
    robot_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot_hardware_status",
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&XrTeleop::robot_status_cb, this, _1));

    head_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr/raw_head_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&XrTeleop::head_pose_cb, this, std::placeholders::_1));

    for (uint8_t i = static_cast<uint8_t>(Arm::LEFT); i < static_cast<uint8_t>(Arm::LAST); ++i)
    {
      const Arm arm = static_cast<Arm>(i);
      const std::string side = arm_to_str(arm);

      is_initial_recorded_[arm] = false;

      // A-button state
      a_press_start_time_[arm] = this->now();
      a_was_pressed_[arm]      = false;

      // B-button state
      b_press_start_time_[arm] = this->now();
      b_was_pressed_[arm]      = false;
      b_triggered_[arm]        = false;

      enabled_[arm] = false;

      tcp_[arm] = tcp[i - 1];
      k_sources_[arm] = k_sources[i - 1];

      target_pose_pub_[arm] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + side + "_cartesian_motion_controller/target_frame", 1);
      ready_pub_[arm] = this->create_publisher<std_msgs::msg::Bool>(
        "/" + side + "_arm/ready", 1);

      std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> raw_pose_callback =
        std::bind(&XrTeleop::arm_pose_cb, this, _1, arm, side + "_hand_frame");
      arm_pose_sub_[arm] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/xr/raw_" + side + "_arm",
        rclcpp::SensorDataQoS(),
        raw_pose_callback);

      // ---- A button ----
      std::function<void(const xr_teleop_msgs::msg::Button::SharedPtr)> btn_a_callback =
        std::bind(&XrTeleop::btn_a_cb, this, _1, arm);
      btn_a_sub_[arm] = this->create_subscription<xr_teleop_msgs::msg::Button>(
        "/xr/" + side + "_a_button",
        rclcpp::SensorDataQoS(),
        btn_a_callback);

      // ---- B button ----
      std::function<void(const xr_teleop_msgs::msg::Button::SharedPtr)> btn_b_callback =
        std::bind(&XrTeleop::btn_b_cb, this, _1, arm);
      btn_b_sub_[arm] = this->create_subscription<xr_teleop_msgs::msg::Button>(
        "/xr/" + side + "_b_button",
        rclcpp::SensorDataQoS(),
        btn_b_callback);

      b_short_press_joint_positions_[arm] = get_parameter("b_short_press_" + side + "_positions").as_double_array();
      b_short_press_joint_names_[arm] = get_parameter("b_short_press_" + side + "_joints").as_string_array();

      execute_cli_[arm]  = this->create_client<robot_controller_msgs::srv::ExecuteJoints>(
        "/" + side + "_arm/execute_joints");
    }

    // sanity: names.size() == positions.size() per arm
    for (const auto & [arm, names] : b_short_press_joint_names_)
    {
      const auto & pos = b_short_press_joint_positions_[arm];
      if (names.size() != pos.size())
      {
        RCLCPP_FATAL(get_logger(),
          "b_short_press_*_joints and _positions size mismatch for %s (%zu vs %zu)",
          arm_to_str(arm).c_str(), names.size(), pos.size());
        return CallbackReturn::FAILURE;
      }
    }

    // Create a 50 Hz timer, but keep it cancelled until activated
    tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
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

    // Reset per-arm button state so no stale press carries over
    for (auto& [arm, t] : a_press_start_time_) t = this->now();
    for (auto& [arm, p] : a_was_pressed_)      p = false;
    for (auto& [arm, t] : b_press_start_time_) t = this->now();
    for (auto& [arm, p] : b_was_pressed_)      p = false;
    for (auto& [arm, f] : b_triggered_)        f = false;

    toggle_request_in_flight_.store(false);

    // Start listening and publishing TF
    tf_timer_->reset();

    RCLCPP_INFO(this->get_logger(), "XrTeleop activated.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // Pause operations
    tf_timer_->cancel();

    for (auto& [arm, is_enabled] : enabled_)
      is_enabled = false;
    for (auto& [arm, pressed] : a_was_pressed_)
      pressed = false;
    for (auto& [arm, pressed] : b_was_pressed_)
      pressed = false;
    for (auto& [arm, fired] : b_triggered_)
      fired = false;
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
    btn_b_sub_.clear();
    target_pose_pub_.clear();
    ready_pub_.clear();

    execute_cli_.clear();
    b_short_press_joint_names_.clear();
    b_short_press_joint_positions_.clear();

    robot_status_sub_.reset();
    start_robot_cli_.reset();
    shutdown_robot_cli_.reset();

    // Clear state tracking maps
    is_initial_recorded_.clear();
    tf_begin_.clear();
    tf_zero_.clear();

    a_press_start_time_.clear();
    a_was_pressed_.clear();
    b_press_start_time_.clear();
    b_was_pressed_.clear();
    b_triggered_.clear();
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

  // ------------------------------------------------------------------
  // TF timer (50 Hz)
  // ------------------------------------------------------------------
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
      auto tf_opt = get_tf(ref_frame_, frame);

      if (!tf_opt.has_value())
        continue;

      const auto & tf = tf_opt.value();

      // Freshness
      const double age_s = (now - tf.header.stamp).seconds();
      if (age_s < 0.0 || age_s > max_age_s_)
      {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
          "Dropping %s target: TF age %.3f s", frame.c_str(), age_s);
        continue;
      }

      // Build and publish (unique_ptr -> move overload -> IPC-friendly)
      auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
      msg->header.stamp    = tf.header.stamp;
      msg->header.frame_id = ref_frame_;
      msg->pose.position.x = tf.transform.translation.x;
      msg->pose.position.y = tf.transform.translation.y;
      msg->pose.position.z = tf.transform.translation.z;
      msg->pose.orientation = tf.transform.rotation;

      target_pose_pub_[arm]->publish(std::move(msg));
    }
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

    tf2::Transform tf_end;
    tf2::fromMsg(msg->pose, tf_end);

    if (!is_initial_recorded_[arm])
    {
      tf_begin_[arm] = tf_end;

      const std::string ee_frame = tcp_.at(arm);
      auto tf_zero_opt = get_tf(ref_frame_, ee_frame);

      if (tf_zero_opt.has_value())
      {
        tf2::fromMsg(tf_zero_opt.value().transform, tf_zero_[arm]);
        is_initial_recorded_[arm] = true;
        RCLCPP_INFO(this->get_logger(), "Recorded %s arm tf_begin and tf_zero", arm_to_str(arm).c_str());
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000,
          "Waiting for %s arm current pose for tf_zero...", arm_to_str(arm).c_str());
        return;
      }
    }

    tf2::Transform tf_delta = tf_begin_[arm].inverse() * tf_end;
    tf2::Transform tf_result = tf_zero_[arm] * tf_delta;

    geometry_msgs::msg::PoseStamped result_msg;
    result_msg.header.stamp = msg->header.stamp;
    result_msg.header.frame_id = ref_frame_;
    tf2::toMsg(tf_result, result_msg.pose);

    broadcast_tf(std::make_shared<geometry_msgs::msg::PoseStamped>(result_msg), ref_frame_, frame);
  }

  // ------------------------------------------------------------------
  // A button: hold to enable teleop for that arm
  // ------------------------------------------------------------------
  void btn_a_cb(const xr_teleop_msgs::msg::Button::SharedPtr msg, const Arm arm)
  {
    if (!is_active())
      return;

    const bool is_pressed = msg->pressed;

    if (is_pressed && !a_was_pressed_[arm])
    {
      a_press_start_time_[arm] = this->now();
    }
    else if (is_pressed && a_was_pressed_[arm])
    {
      double held_sec = (this->now() - a_press_start_time_[arm]).seconds();
      if (held_sec >= a_button_hold_duration_)
      {
        if (!enabled_[arm])
        {
          enabled_[arm] = true;
          is_initial_recorded_[arm] = false;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
            "%s arm ENABLED", arm_to_str(arm).c_str());
        }
      }
    }
    else if (!is_pressed && a_was_pressed_[arm])
    {
      enabled_[arm] = false;
      is_initial_recorded_[arm] = false;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
        "%s arm DISABLED", arm_to_str(arm).c_str());
    }

    a_was_pressed_[arm] = is_pressed;

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = enabled_[arm];
    ready_pub_[arm]->publish(ready_msg);
  }

  // ------------------------------------------------------------------
  // B button: hold >= b_button_hold_duration (3 s) to toggle start/shutdown
  // ------------------------------------------------------------------
  void robot_status_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_running_.store(msg->data);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "robot_hardware_status = %s", robot_running_.load() ? "true" : "false");
  }

  void btn_b_cb(const xr_teleop_msgs::msg::Button::SharedPtr msg, const Arm arm)
  {
    if (!is_active())
      return;

    const bool is_pressed = msg->pressed;

    if (is_pressed && !b_was_pressed_[arm])
    {
      // Press edge: start timing
      b_press_start_time_[arm] = this->now();
      b_triggered_[arm]        = false;
    }
    else if (is_pressed && b_was_pressed_[arm])
    {
      // Still held: fire once per press cycle at the threshold
      if (!b_triggered_[arm])
      {
        const double held = (this->now() - b_press_start_time_[arm]).seconds();
        if (held >= b_button_hold_duration_)
        {
          b_triggered_[arm] = true;
          RCLCPP_INFO(get_logger(),
            "%s B-button held %.2f s -> toggling robot state",
            arm_to_str(arm).c_str(), held);
          toggle_robot_state();
        }
      }
    }
    else if (!is_pressed && b_was_pressed_[arm])
    {
      // Release edge: decide between short-press action and re-arming
      if (!b_triggered_[arm])
      {
        const double held = (this->now() - b_press_start_time_[arm]).seconds();
        if (held >= b_short_press_min_s_ && held <= b_short_press_max_s_)
        {
          RCLCPP_INFO(get_logger(),
            "%s B-button short press (%.3f s) -> execute_joints",
            arm_to_str(arm).c_str(), held);
          execute_joints_for_arm(arm);
        }
        else
        {
          RCLCPP_DEBUG(get_logger(),
            "%s B-button released after %.3f s (no action)",
            arm_to_str(arm).c_str(), held);
        }
      }
      b_triggered_[arm] = false;   // re-arm for the next press
    }

    b_was_pressed_[arm] = is_pressed;
  }

  void toggle_robot_state()
  {
    if (toggle_request_in_flight_.load())
    {
      RCLCPP_WARN(get_logger(),
        "Toggle requested, but a start/shutdown request is already in flight");
      return;
    }

    const bool want_start = !robot_running_.load();
    auto client = want_start ? start_robot_cli_ : shutdown_robot_cli_;
    const char * name = want_start ? "start_robot" : "shutdown_robot";

    if (!client || !client->service_is_ready())
    {
      RCLCPP_WARN(get_logger(),
        "B-button toggle: service '%s' not available", name);
      return;
    }

    toggle_request_in_flight_.store(true);

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    RCLCPP_INFO(get_logger(),
      "B-button toggle -> calling /%s (robot_running_=%s)",
      name, robot_running_.load() ? "true" : "false");

    client->async_send_request(
      req,
      [this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        toggle_request_in_flight_.store(false);
        try
        {
          const auto resp = future.get();
          if (resp->success)
            RCLCPP_INFO(get_logger(), "/%s OK: %s", name, resp->message.c_str());
          else
            RCLCPP_ERROR(get_logger(), "/%s FAILED: %s", name, resp->message.c_str());
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(get_logger(), "/%s exception: %s", name, e.what());
        }
      });
  }

  void execute_joints_for_arm(const Arm arm)
  {
    const std::string side = arm_to_str(arm);

    if (execute_cli_.find(arm) == execute_cli_.end())
    {
      RCLCPP_WARN(get_logger(), "/%s_arm/execute_joints client not found; dropping request", side.c_str());
      return;
    }

    auto client = execute_cli_[arm];
  
    if (!client || !client->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "/%s_arm/execute_joints not available; dropping request", side.c_str());
      return;
    }

    auto req = std::make_shared<robot_controller_msgs::srv::ExecuteJoints::Request>();
    req->joint_names     = b_short_press_joint_names_[arm];
    req->joint_positions = b_short_press_joint_positions_[arm];
    req->async_execute   = true;

    RCLCPP_INFO(get_logger(),
      "Calling /%s_arm/execute_joints (%zu joints, async)",
      side.c_str(), req->joint_names.size());

    client->async_send_request(
      req,
      [this, side](rclcpp::Client<robot_controller_msgs::srv::ExecuteJoints>::SharedFuture future)
      {
        try
        {
          const auto resp = future.get();
          // Adjust based on your actual srv response fields.
          // If it's `bool success, string message`, use:
          //   if (!resp->success) RCLCPP_ERROR(...);
          RCLCPP_INFO(get_logger(),
            "/%s_arm/execute_joints -> success=%d", side.c_str(), resp->success);
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(get_logger(), "/%s_arm/execute_joints exception: %s", side.c_str(), e.what());
        }
      });
  }

  // ------------------------------------------------------------------
  // TF helpers
  // ------------------------------------------------------------------
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

  void publish_static_tf(
    const std::string& frame_id,
    const std::string& child_frame_id,
    double x, double y, double z,
    double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = frame_id;
    t.child_frame_id  = child_frame_id;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s",
      frame_id.c_str(), child_frame_id.c_str());
  }

  void broadcast_static_tf()
  {
    std::vector<std::string> xr = this->get_parameter("xr_reference_data").as_string_array();
    if (xr.size() == 8)
    {
      publish_static_tf(
        xr[0], xr[1],
        std::stod(xr[2]), std::stod(xr[3]), std::stod(xr[4]),
        std::stod(xr[5]), std::stod(xr[6]), std::stod(xr[7]));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "xr_reference_data must have == 8 elements, got %zu", xr.size());
    }

    publish_static_tf("xr_base_link", "head_frame",
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0);
  }

  // ------------------------------------------------------------------
  // Member variables
  // ------------------------------------------------------------------
  double max_age_s_{0.5};
  double a_button_hold_duration_{0.5};
  double b_button_hold_duration_{3.0};
  double b_short_press_min_s_{0.1};
  double b_short_press_max_s_{0.4};
  std::string ref_frame_;

  // Robot start/shutdown toggle
  std::atomic<bool> robot_running_{false};
  std::atomic<bool> toggle_request_in_flight_{false};

  // Per-arm teleop state
  std::unordered_map<Arm, bool> is_initial_recorded_;
  std::unordered_map<Arm, tf2::Transform> tf_begin_;
  std::unordered_map<Arm, tf2::Transform> tf_zero_;
  std::unordered_map<Arm, bool> enabled_;

  // A-button per-arm state
  std::unordered_map<Arm, rclcpp::Time> a_press_start_time_;
  std::unordered_map<Arm, bool>         a_was_pressed_;

  // B-button per-arm state
  std::unordered_map<Arm, rclcpp::Time> b_press_start_time_;
  std::unordered_map<Arm, bool>         b_was_pressed_;
  std::unordered_map<Arm, bool>         b_triggered_;

  std::unordered_map<Arm, std::vector<std::string>> b_short_press_joint_names_;
  std::unordered_map<Arm, std::vector<double>>      b_short_press_joint_positions_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::TimerBase::SharedPtr tf_timer_;

  // Publishers
  std::unordered_map<Arm, rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr> ready_pub_;
  std::unordered_map<Arm, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pose_pub_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr head_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> arm_pose_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr> btn_a_sub_;
  std::unordered_map<Arm, rclcpp::Subscription<xr_teleop_msgs::msg::Button>::SharedPtr> btn_b_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_status_sub_;

  // Service clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_robot_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_robot_cli_;
  std::unordered_map<Arm, rclcpp::Client<robot_controller_msgs::srv::ExecuteJoints>::SharedPtr> execute_cli_;

  // Frame names
  std::unordered_map<Arm, std::string> tcp_;
  std::unordered_map<Arm, std::string> k_sources_;
};

} // namespace xr_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(xr_teleop::XrTeleop)