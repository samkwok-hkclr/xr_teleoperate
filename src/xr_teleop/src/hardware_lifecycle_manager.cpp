#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace xr_teleop
{

using CallbackReturn   = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using SetHardwareState = controller_manager_msgs::srv::SetHardwareComponentState;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using Trigger          = std_srvs::srv::Trigger;
using LifecycleState   = lifecycle_msgs::msg::State;

class HardwareLifecycleManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HardwareLifecycleManager(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("hardware_lifecycle_manager", options)
  {
    this->declare_parameter<std::vector<std::string>>("components", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("controllers", std::vector<std::string>{});
    this->declare_parameter<std::string>("controller_manager", "/controller_manager");
    this->declare_parameter<double>("service_timeout_sec", 30.0);
    this->declare_parameter<std::string>("start_service", "/start_robot");
    this->declare_parameter<std::string>("shutdown_service", "/shutdown_robot");
    this->declare_parameter<std::string>("status_topic", "/robot_hardware_status");

    components_          = this->get_parameter("components").as_string_array();
    controllers_         = this->get_parameter("controllers").as_string_array();
    controller_manager_  = this->get_parameter("controller_manager").as_string();
    service_timeout_sec_ = this->get_parameter("service_timeout_sec").as_double();
    start_service_       = this->get_parameter("start_service").as_string();
    shutdown_service_    = this->get_parameter("shutdown_service").as_string();
    status_topic_        = this->get_parameter("status_topic").as_string();

    if (components_.empty())
    {
      RCLCPP_FATAL(get_logger(),
        "'components' parameter must be a non-empty list of hardware component names");
    }
    if (controllers_.empty())
    {
      RCLCPP_WARN(get_logger(),
        "'controllers' parameter is empty; no controllers will be switched");
    }
  }

protected:
  // ---------------------------------------------------------------
  // Lifecycle transitions
  // ---------------------------------------------------------------
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // Service callbacks block while waiting for the sequence; the state
    // client callbacks must be able to run concurrently, otherwise we
    // deadlock until service_timeout_sec_ elapses.
    service_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_  = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(status_topic_, status_qos);

    state_client_ = this->create_client<SetHardwareState>(
      controller_manager_ + "/set_hardware_component_state",
      rmw_qos_profile_services_default,
      client_cb_group_);

    switch_client_ = this->create_client<SwitchController>(
      controller_manager_ + "/switch_controller",
      rmw_qos_profile_services_default,
      client_cb_group_);

    start_srv_ = this->create_service<Trigger>(
      start_service_,
      std::bind(&HardwareLifecycleManager::start_service_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_cb_group_);

    shutdown_srv_ = this->create_service<Trigger>(
      shutdown_service_,
      std::bind(&HardwareLifecycleManager::shutdown_service_cb, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_cb_group_);

    RCLCPP_INFO(get_logger(),
      "on_configure: start='%s', shutdown='%s', status='%s', cm='%s'",
      start_service_.c_str(), shutdown_service_.c_str(), status_topic_.c_str(),
      controller_manager_.c_str());
    RCLCPP_INFO(get_logger(), "  components (%zu):", components_.size());
    for (const auto & c : components_)   RCLCPP_INFO(get_logger(), "    - '%s'", c.c_str());
    RCLCPP_INFO(get_logger(), "  controllers (%zu):", controllers_.size());
    for (const auto & c : controllers_)  RCLCPP_INFO(get_logger(), "    - '%s'", c.c_str());

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    status_pub_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    status_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    start_srv_.reset();
    shutdown_srv_.reset();
    status_pub_.reset();
    state_client_.reset();
    switch_client_.reset();
    service_cb_group_.reset();
    client_cb_group_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "on_shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  enum class Action { None, Start, Shutdown };
  enum class Phase
  {
    None,
    // start path
    ConfigureComponents,     // unconfigured -> inactive, one-by-one
    ActivateComponents,      // inactive     -> active,   one-by-one
    ActivateControllers,     // single switch_controller call (activate)
    // shutdown path
    DeactivateControllers,   // single switch_controller call (deactivate)
    DeactivateComponents,    // active       -> inactive, one-by-one
    CleanupComponents,       // inactive     -> unconfigured, one-by-one
  };

  // ---------------- Service callbacks ----------------
  void start_service_cb(
    const std::shared_ptr<Trigger::Request> /*req*/,
    std::shared_ptr<Trigger::Response> resp)
  {
    handle_request(Action::Start, resp);
  }

  void shutdown_service_cb(
    const std::shared_ptr<Trigger::Request> /*req*/,
    std::shared_ptr<Trigger::Response> resp)
  {
    handle_request(Action::Shutdown, resp);
  }

  void handle_request(Action action, std::shared_ptr<Trigger::Response> resp)
  {
    if (this->get_current_state().id() != LifecycleState::PRIMARY_STATE_ACTIVE)
    {
      resp->success = false;
      resp->message = "hardware_lifecycle_manager is not active";
      RCLCPP_WARN(get_logger(), "Rejecting request: node is not active");
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    if (sequence_active_)
    {
      resp->success = false;
      resp->message = "another start/shutdown sequence is already in progress";
      RCLCPP_WARN(get_logger(), "Rejecting request: sequence already active");
      return;
    }

    sequence_active_   = true;
    sequence_done_     = false;
    sequence_is_start_ = (action == Action::Start);
    sequence_success_  = false;
    sequence_message_.clear();
    sequence_idx_      = 0;

    if (sequence_is_start_)
    {
      phase_ = Phase::ConfigureComponents;
      RCLCPP_INFO(get_logger(), "Start sequence begin");
    }
    else
    {
      phase_ = Phase::DeactivateControllers;
      RCLCPP_INFO(get_logger(), "Shutdown sequence begin");
    }

    lock.unlock();
    advance_sequence();
    lock.lock();

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(service_timeout_sec_));

    if (!cv_.wait_until(lock, deadline, [this] { return sequence_done_; }))
    {
      resp->success = false;
      resp->message = "timeout waiting for sequence to complete";
      RCLCPP_ERROR(get_logger(),
        "Timeout after %.1f s waiting for sequence", service_timeout_sec_);
      return;
    }

    resp->success = sequence_success_;
    resp->message = sequence_message_;
    RCLCPP_INFO(get_logger(),
      "Responding: success=%s, message='%s'",
      resp->success ? "true" : "false", resp->message.c_str());
  }

  // ---------------- Sequence driver ----------------
  void advance_sequence()
  {
    switch (phase_)
    {
      case Phase::ConfigureComponents:
        if (advance_component_phase(
              LifecycleState::PRIMARY_STATE_INACTIVE, "inactive")) return;
        RCLCPP_INFO(get_logger(), "All components configured; activating");
        phase_ = Phase::ActivateComponents;
        sequence_idx_ = 0;
        advance_sequence();
        return;

      case Phase::ActivateComponents:
        if (advance_component_phase(
              LifecycleState::PRIMARY_STATE_ACTIVE, "active")) return;
        RCLCPP_INFO(get_logger(), "All components activated; activating controllers");
        phase_ = Phase::ActivateControllers;
        advance_sequence();
        return;

      case Phase::ActivateControllers:
        send_switch_request(
          /*activate=*/controllers_, /*deactivate=*/{},
          "activate robot controllers",
          [this]()
          {
            RCLCPP_INFO(get_logger(), "Start sequence complete");
            publish_status(true);
            complete_sequence(true, "all components active, controllers active");
          });
        return;

      case Phase::DeactivateControllers:
        send_switch_request(
          /*activate=*/{}, /*deactivate=*/controllers_,
          "deactivate robot controllers",
          [this]()
          {
            RCLCPP_INFO(get_logger(), "Controllers deactivated; deactivating components");
            phase_ = Phase::DeactivateComponents;
            sequence_idx_ = 0;
            advance_sequence();
          });
        return;

      case Phase::DeactivateComponents:
        if (advance_component_phase(
              LifecycleState::PRIMARY_STATE_INACTIVE, "inactive")) return;
        RCLCPP_INFO(get_logger(), "All components deactivated; cleaning up");
        phase_ = Phase::CleanupComponents;
        sequence_idx_ = 0;
        advance_sequence();
        return;

      case Phase::CleanupComponents:
        if (advance_component_phase(
              LifecycleState::PRIMARY_STATE_UNCONFIGURED, "unconfigured")) return;
        RCLCPP_INFO(get_logger(), "Shutdown sequence complete");
        publish_status(false);
        complete_sequence(true, "all components unconfigured");
        return;

      case Phase::None:
      default:
        RCLCPP_ERROR(get_logger(), "advance_sequence called with no active phase");
        fail_sequence("internal error: no active phase");
        return;
    }
  }

  // Dispatches one component transition for the current phase.
  // Returns true if a request was dispatched (still in progress),
  // false if all components in this phase have been processed.
  bool advance_component_phase(std::uint8_t target_id, const std::string & target_label)
  {
    if (sequence_idx_ >= components_.size()) return false;
    const std::string name = components_[sequence_idx_];
    ++sequence_idx_;
    send_state_request(name, target_id, target_label,
      [this]() { advance_sequence(); });
    return true;
  }

  void complete_sequence(bool success, const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sequence_success_ = success;
      sequence_message_ = message;
      sequence_done_    = true;
      sequence_active_  = false;
    }
    cv_.notify_all();
  }

  void fail_sequence(const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sequence_success_ = false;
      sequence_message_ = message;
      sequence_done_    = true;
      sequence_active_  = false;
    }
    cv_.notify_all();
  }

  // ---------------- Helpers ----------------
  using SuccessCb = std::function<void()>;

  // ---- Hardware component state transition (one component) ----
  void send_state_request(
    const std::string & name,
    std::uint8_t target_id,
    const std::string & target_label,
    SuccessCb on_success)
  {
    if (!state_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(),
        "set_hardware_component_state not available; aborting sequence");
      fail_sequence("set_hardware_component_state service not available");
      return;
    }

    auto req = std::make_shared<SetHardwareState::Request>();
    req->name               = name;
    req->target_state.id    = target_id;
    req->target_state.label = target_label;

    RCLCPP_INFO(get_logger(), "Requesting '%s' -> '%s'", name.c_str(), target_label.c_str());

    state_client_->async_send_request(
      req,
      [this, name, target_label, on_success](
        rclcpp::Client<SetHardwareState>::SharedFuture future)
      {
        std::shared_ptr<SetHardwareState::Response> resp;
        try { resp = future.get(); }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(get_logger(),
            "Service call failed for '%s': %s", name.c_str(), e.what());
          fail_sequence(std::string("service call failed for '") + name + "': " + e.what());
          return;
        }

        if (resp->ok)
        {
          RCLCPP_INFO(get_logger(),
            "OK: '%s' -> '%s' (now '%s')",
            name.c_str(), target_label.c_str(), resp->state.label.c_str());
          if (on_success) on_success();
        }
        else
        {
          RCLCPP_ERROR(get_logger(),
            "FAILED: '%s' -> '%s'", name.c_str(), target_label.c_str());
          fail_sequence(std::string("'") + name + "' -> '" + target_label + "' failed");
        }
      });
  }

  // ---- Controller activation / deactivation (batch) ----
  void send_switch_request(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate,
    const std::string & reason,
    SuccessCb on_success)
  {
    // If nothing to do, skip the call but still run the callback.
    if (activate.empty() && deactivate.empty())
    {
      RCLCPP_INFO(get_logger(), "No controllers to switch for '%s'; skipping", reason.c_str());
      if (on_success) on_success();
      return;
    }

    if (!switch_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(),
        "switch_controller not available; aborting sequence");
      fail_sequence("switch_controller service not available");
      return;
    }

    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers   = activate;
    req->deactivate_controllers = deactivate;
    req->strictness             = SwitchController::Request::BEST_EFFORT;
    req->activate_asap          = true;
    req->timeout.sec            = 5;
    req->timeout.nanosec        = 0;

    RCLCPP_INFO(get_logger(),
      "switch_controller: %s (activate=%zu, deactivate=%zu)",
      reason.c_str(), activate.size(), deactivate.size());
    for (const auto & c : activate)   RCLCPP_INFO(get_logger(), "  activate:   '%s'", c.c_str());
    for (const auto & c : deactivate) RCLCPP_INFO(get_logger(), "  deactivate: '%s'", c.c_str());

    switch_client_->async_send_request(
      req,
      [this, reason, on_success](rclcpp::Client<SwitchController>::SharedFuture future)
      {
        std::shared_ptr<SwitchController::Response> resp;
        try { resp = future.get(); }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(get_logger(), "switch_controller exception: %s", e.what());
          fail_sequence(std::string("switch_controller failed: ") + e.what());
          return;
        }

        if (resp->ok)
        {
          RCLCPP_INFO(get_logger(), "switch_controller OK: %s", reason.c_str());
          if (on_success) on_success();
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "switch_controller FAILED: %s", reason.c_str());
          fail_sequence(std::string("switch_controller failed: ") + reason);
        }
      });
  }

  void publish_status(bool value)
  {
    if (!status_pub_ || !status_pub_->is_activated())
    {
      RCLCPP_WARN(get_logger(), "status publisher not activated; dropping publish");
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = value;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
      "Published %s on %s", value ? "true" : "false", status_topic_.c_str());
  }

  // ---------------- Parameters ----------------
  std::vector<std::string> components_;
  std::vector<std::string> controllers_;
  std::string controller_manager_;
  double      service_timeout_sec_{30.0};
  std::string start_service_;
  std::string shutdown_service_;
  std::string status_topic_;

  // ---------------- ROS entities ----------------
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Client<SetHardwareState>::SharedPtr                           state_client_;
  rclcpp::Client<SwitchController>::SharedPtr                           switch_client_;
  rclcpp::Service<Trigger>::SharedPtr                                   start_srv_;
  rclcpp::Service<Trigger>::SharedPtr                                   shutdown_srv_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // ---------------- Sequence state ----------------
  std::mutex              mutex_;
  std::condition_variable cv_;
  bool                    sequence_active_{false};
  bool                    sequence_done_{false};
  bool                    sequence_is_start_{true};
  bool                    sequence_success_{false};
  std::string             sequence_message_;
  size_t                  sequence_idx_{0};
  Phase                   phase_{Phase::None};
};

}  // namespace xr_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(xr_teleop::HardwareLifecycleManager)