
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/bool.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"

namespace xr_teleop
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using ListControllers  = controller_manager_msgs::srv::ListControllers;

class PoseTrackingWrapper : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PoseTrackingWrapper(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("pose_tracking_wrapper", options)
  {
    // ---------------- parameters ----------------
    this->declare_parameter<std::string>("side", "");
    this->declare_parameter<std::string>("controller_manager", "/controller_manager");
    this->declare_parameter<double>("poll_rate_hz", 20.0);
    this->declare_parameter<bool>("publish_clear_on_switch_back", true);
    this->declare_parameter<bool>("skip_initial_message", true);
    this->declare_parameter<double>("warn_pulse_sec", 1.0);

    side_               = this->get_parameter("side").as_string();
    controller_manager_ = this->get_parameter("controller_manager").as_string();
    poll_rate_hz_       = this->get_parameter("poll_rate_hz").as_double();
    publish_clear_on_switch_back_ = this->get_parameter("publish_clear_on_switch_back").as_bool();
    skip_initial_message_         = this->get_parameter("skip_initial_message").as_bool();
    warn_pulse_sec_ = this->get_parameter("warn_pulse_sec").as_double();

    if (side_.empty()) 
    {
      RCLCPP_FATAL(get_logger(), "'side' parameter must be set (e.g. 'left' or 'right')");
    }

    cartesian_ctrl_ = side_ + "_cartesian_motion_controller";
    arm_ctrl_       = side_ + "_arm_controller";
    ready_topic_    = "/" + side_ + "_arm/ready";
    warn_topic_     = "/" + side_ + "_arm/warn";
  }

protected:
  // ---------------------------------------------------------------
  // Lifecycle transitions
  // ---------------------------------------------------------------
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      ready_topic_,
      sub_qos,
      std::bind(&PoseTrackingWrapper::ready_cb, this, std::placeholders::_1));

    auto warn_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    warn_pub_ = this->create_publisher<std_msgs::msg::Bool>(warn_topic_, warn_qos);

    switch_client_ = this->create_client<SwitchController>(controller_manager_ + "/switch_controller");
    list_client_ = this->create_client<ListControllers>(controller_manager_ + "/list_controllers");

    // 20 Hz state-machine timer
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / poll_rate_hz_));
    timer_ = this->create_wall_timer(period_ns, std::bind(&PoseTrackingWrapper::process_pending, this));

    warn_off_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(warn_pulse_sec_)),
      std::bind(&PoseTrackingWrapper::warn_off_cb, this));
    warn_off_timer_->cancel();  // start disarmed; only fires when armed

    RCLCPP_INFO(get_logger(),
      "on_configure: side='%s', ready='%s', warn='%s', cartesian='%s', arm='%s'",
      side_.c_str(), ready_topic_.c_str(), warn_topic_.c_str(),
      cartesian_ctrl_.c_str(), arm_ctrl_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    warn_pub_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (warn_off_timer_) 
    {
      warn_off_timer_->cancel();   // drop any pending auto-off
    }
    warn_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    ready_sub_.reset();
    warn_pub_.reset();
    switch_client_.reset();
    list_client_.reset();
    timer_.reset();
    if (warn_off_timer_) 
    {
      warn_off_timer_->cancel();
      warn_off_timer_.reset();
    }
    RCLCPP_INFO(get_logger(), "on_cleanup");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "on_shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  enum class Action { None, ToCartesianMotion, ToJointTrajectory };

  // ---------------- Subscription callback ----------------
  // Convention: ready == true  -> track Cartesian pose
  //             ready == false -> hand back to joint trajectory controller
  void ready_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool now_ready = msg->data;

    // First message: only sync state, don't trigger a switch.
    if (skip_initial_message_ && !initialized_.exchange(true)) 
    {
      prev_ready_.store(now_ready);
      RCLCPP_INFO(get_logger(), "ready_cb: initial state on '%s' = %s (no action taken)",
        ready_topic_.c_str(), now_ready ? "true" : "false");
      return;
    }
    initialized_.store(true);

    const bool prev = prev_ready_.exchange(now_ready);
    if (now_ready == prev) 
    {
      return;  // no edge
    }

    const Action action = now_ready ? Action::ToCartesianMotion : Action::ToJointTrajectory;
    pending_action_.store(action);

    RCLCPP_INFO(get_logger(),
      "ready edge on '%s': %s -> %s, queuing '%s'",
      ready_topic_.c_str(),
      prev ? "true" : "false",
      now_ready ? "true" : "false",
      action == Action::ToCartesianMotion
        ? "ToCartesianMotion (activate cartesian, deactivate arm)"
        : "ToJointTrajectory (activate arm, deactivate cartesian)");
  }

  void warn_off_cb()
  {
    // one-shot: cancel first so it doesn't repeat
    if (warn_off_timer_) 
    {
      warn_off_timer_->cancel();
    }
    publish_warn(false);
  }

  // ---------------- Timer callback (20 Hz) ----------------
  void process_pending()
  {
    const Action a = pending_action_.exchange(Action::None);
    if (a == Action::None) 
    {
      return;
    }

    if (a == Action::ToCartesianMotion) 
    {
      // ready rose (false -> true): enter Cartesian pose tracking
      RCLCPP_INFO(get_logger(), "ready rose -> activate cartesian motion, deactivate joint trajectory");
      send_switch_request(
        /*activate=*/{cartesian_ctrl_},
        /*deactivate=*/{arm_ctrl_},
        "enter Cartesian pose tracking",
        /*on_success=*/[this]() { 
          publish_warn(true); 
        });
    }
    else if (a == Action::ToJointTrajectory) 
    {
      // ready fell (true -> false): hand back to joint trajectory
      RCLCPP_INFO(get_logger(), "ready fell -> activate joint trajectory, deactivate cartesian motion");
      send_switch_request(
        /*activate=*/{arm_ctrl_},
        /*deactivate=*/{cartesian_ctrl_},
        "hand back to joint trajectory controller",
        /*on_success=*/[this]() {
          if (publish_clear_on_switch_back_) 
          {
            publish_warn(true);
          }
        });
    }
  }

  // ---------------- Helpers ----------------
  using SuccessCb = std::function<void()>;

  void publish_warn(bool value)
  {
    if (!warn_pub_ || !warn_pub_->is_activated()) 
    {
      RCLCPP_WARN(get_logger(), "warn publisher not activated; dropping publish");
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = value;
    warn_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published %s on %s", value ? "true" : "false", warn_topic_.c_str());

    // Pulse behaviour: whenever we vibrate (true), schedule a stop (false)
    if (value && warn_off_timer_) 
    {
      warn_off_timer_->reset();   // un-cancels + sets next fire to now + 500 ms
    }
  }

  // Filter activate/deactivate lists to only controllers that need changing.
  void send_switch_request(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate,
    const std::string & reason,
    SuccessCb on_success = nullptr)
  {
    if (!switch_client_->service_is_ready()) 
    {
      RCLCPP_WARN(get_logger(), "switch_controller not available; dropping action: %s", reason.c_str());
      return;
    }

    // If list_controllers is available, filter out no-op entries so we don't
    // spam the log with "can not be deactivated since it is not active".
    if (list_client_->service_is_ready()) 
    {
      auto list_req = std::make_shared<ListControllers::Request>();
      list_client_->async_send_request(
        list_req,
        [this, activate, deactivate, reason, on_success](rclcpp::Client<ListControllers>::SharedFuture future)
        {
          std::map<std::string, std::string> state;
          for (const auto & c : future.get()->controller) 
          {
            state[c.name] = c.state;
          }

          std::vector<std::string> a, d;
          for (const auto & n : activate) 
          {
            auto it = state.find(n);
            if (it == state.end()) 
            {
              RCLCPP_WARN(get_logger(), "'%s' not found; skipping", n.c_str());
              continue;
            }
            if (it->second != "active") 
              a.push_back(n);
          }
          for (const auto & n : deactivate) 
          {
            auto it = state.find(n);
            if (it == state.end()) 
            {
              RCLCPP_WARN(get_logger(), "'%s' not found; skipping", n.c_str());
              continue;
            }
            if (it->second == "active") 
              d.push_back(n);
          }

          if (a.empty() && d.empty()) 
          {
            RCLCPP_INFO(get_logger(), "nothing to switch for '%s'; already in desired state", reason.c_str());
            if (on_success) 
              on_success();
            return;
          }
          do_switch(a, d, reason, on_success);
        });
    } 
    else 
    {
      do_switch(activate, deactivate, reason, on_success);
    }
  }

  void do_switch(
    const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate,
    const std::string & reason,
    SuccessCb on_success)
  {
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers   = activate;
    req->deactivate_controllers = deactivate;
    req->strictness             = SwitchController::Request::BEST_EFFORT;
    req->activate_asap          = true;
    req->timeout.sec            = 1;
    req->timeout.nanosec        = 0;

    RCLCPP_INFO(get_logger(), "Sending switch: %s", reason.c_str());

    switch_client_->async_send_request(
      req,
      [this, reason, on_success](rclcpp::Client<SwitchController>::SharedFuture future) {
        const bool ok = future.get()->ok;
        if (ok) 
        {
          RCLCPP_INFO(get_logger(), "Switch OK: %s", reason.c_str());
          if (on_success) 
            on_success();
        } 
        else 
        {
          RCLCPP_ERROR(get_logger(), "Switch FAILED: %s", reason.c_str());
        }
      });
  }

  // ---------------- Parameters ----------------
  std::string side_;
  std::string controller_manager_;
  double      poll_rate_hz_{20.0};
  bool        publish_clear_on_switch_back_{true};
  bool        skip_initial_message_{true};
  double      warn_pulse_sec_{0.5};

  // ---------------- Derived names ----------------
  std::string ready_topic_;
  std::string warn_topic_;
  std::string cartesian_ctrl_;
  std::string arm_ctrl_;

  // ---------------- ROS entities ----------------
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr warn_pub_;
  rclcpp::Client<SwitchController>::SharedPtr          switch_client_;
  rclcpp::Client<ListControllers>::SharedPtr           list_client_;
  rclcpp::TimerBase::SharedPtr                         timer_;
  rclcpp::TimerBase::SharedPtr                         warn_off_timer_;

  // ---------------- State ----------------
  std::atomic<bool>   initialized_{false};
  std::atomic<bool>   prev_ready_{false};
  std::atomic<Action> pending_action_{Action::None};
};

}  // namespace xr_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(xr_teleop::PoseTrackingWrapper)