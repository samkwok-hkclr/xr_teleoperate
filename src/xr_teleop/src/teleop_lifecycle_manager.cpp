#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "xr_teleop_msgs/action/start_teleop.hpp"
#include "xr_teleop_msgs/action/stop_teleop.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

const char* state_name(uint8_t id)
{
  switch (id)
  {
    case State::PRIMARY_STATE_UNKNOWN:      return "UNKNOWN";
    case State::PRIMARY_STATE_UNCONFIGURED: return "UNCONFIGURED";
    case State::PRIMARY_STATE_INACTIVE:     return "INACTIVE";
    case State::PRIMARY_STATE_ACTIVE:       return "ACTIVE";
    case State::PRIMARY_STATE_FINALIZED:    return "FINALIZED";
    default:                                return "?";
  }
}

const char* transition_name(uint8_t id)
{
  switch (id)
  {
    case Transition::TRANSITION_CONFIGURE:  return "CONFIGURE";
    case Transition::TRANSITION_CLEANUP:    return "CLEANUP";
    case Transition::TRANSITION_ACTIVATE:   return "ACTIVATE";
    case Transition::TRANSITION_DEACTIVATE: return "DEACTIVATE";
    default:                                return "?";
  }
}
}  // namespace

class TeleopLifecycleManager : public rclcpp::Node
{
  using StartTeleop       = xr_teleop_msgs::action::StartTeleop;
  using StopTeleop        = xr_teleop_msgs::action::StopTeleop;
  using StartTeleopHandle = rclcpp_action::ServerGoalHandle<StartTeleop>;
  using StopTeleopHandle  = rclcpp_action::ServerGoalHandle<StopTeleop>;
  using GetState          = lifecycle_msgs::srv::GetState;
  using ChangeState       = lifecycle_msgs::srv::ChangeState;

  using FeedbackFn = std::function<void(int state)>;
  using CancelFn   = std::function<bool()>;

  struct NodeClients
  {
    rclcpp::Client<GetState>::SharedPtr    get_state;
    rclcpp::Client<ChangeState>::SharedPtr change_state;
  };

public:
  TeleopLifecycleManager()
  : Node("lifecycle_manager")
  {
    declare_parameter<std::vector<std::string>>("managed_nodes", std::vector<std::string>{});
    declare_parameter<bool>("auto_start", false);
    declare_parameter<double>("service_timeout_s", 3.0);
    declare_parameter<double>("request_timeout_s", 30.0);

    service_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(get_parameter("service_timeout_s").as_double()));
    request_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(get_parameter("request_timeout_s").as_double()));

    start_srv_ = rclcpp_action::create_server<StartTeleop>(
      this, "start_teleop",
      std::bind(&TeleopLifecycleManager::start_teleop_goal_cb,     this, _1, _2),
      std::bind(&TeleopLifecycleManager::start_teleop_cancel_cb,   this, _1),
      std::bind(&TeleopLifecycleManager::start_teleop_accepted_cb, this, _1));

    stop_srv_ = rclcpp_action::create_server<StopTeleop>(
      this, "stop_teleop",
      std::bind(&TeleopLifecycleManager::stop_teleop_goal_cb,      this, _1, _2),
      std::bind(&TeleopLifecycleManager::stop_teleop_cancel_cb,    this, _1),
      std::bind(&TeleopLifecycleManager::stop_teleop_accepted_cb,  this, _1));

    std::vector<std::string> nodes = get_parameter("managed_nodes").as_string_array();
    if (nodes.empty())
    {
      RCLCPP_WARN(get_logger(), "No nodes for Lifecycle Manager");
    }
    else
    {
      for (const auto& name : nodes)
      {
        RCLCPP_INFO(get_logger(), "[%s] node will be start automatically", name.c_str());
        clients_[name] = {
          create_client<GetState>(name + "/get_state"),
          create_client<ChangeState>(name + "/change_state"),
        };
      }
    }

    if (get_parameter("auto_start").as_bool())
    {
      RCLCPP_INFO(get_logger(), "Auto-start enabled. Triggering sequence in 1.5 s...");
      auto_start_timer_ = create_wall_timer(3000ms, [this]()
      {
        auto_start_timer_->cancel();

        // Run off the executor thread — run_start_sequence() blocks on
        // sleep_for and on service waits, and we don't want to stall spin().
        std::thread([this]()
        {
          RCLCPP_INFO(get_logger(), "auto_start: running start sequence");

          std::string error;
          const bool ok = run_start_sequence(
            [this](int state)
            {
              RCLCPP_INFO(get_logger(), "auto_start feedback state=%d", state);
            },
            []() { return !rclcpp::ok(); },   // treat shutdown as cancel
            error);

          if (ok)
          {
            RCLCPP_INFO(get_logger(), "auto_start: start sequence completed");
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "auto_start failed: %s", error.c_str());
          }
        }).detach();
      });
    }

    RCLCPP_INFO(get_logger(), "Lifecycle Manager ready ('start_teleop' / 'stop_teleop' actions).");
  }

private:
  // ------------------------------------------------------------------
  // Action callbacks
  // ------------------------------------------------------------------
  rclcpp_action::GoalResponse start_teleop_goal_cb(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const StartTeleop::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received start_teleop goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse start_teleop_cancel_cb(const std::shared_ptr<StartTeleopHandle>)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request for start_teleop");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void start_teleop_accepted_cb(const std::shared_ptr<StartTeleopHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { start_teleop_execution(goal_handle); }).detach();
  }

  rclcpp_action::GoalResponse stop_teleop_goal_cb(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const StopTeleop::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received stop_teleop goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse stop_teleop_cancel_cb(const std::shared_ptr<StopTeleopHandle>)
  {
    RCLCPP_INFO(get_logger(), "Received cancel request for stop_teleop");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void stop_teleop_accepted_cb(const std::shared_ptr<StopTeleopHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { stop_teleop_execution(goal_handle); }).detach();
  }

  // ------------------------------------------------------------------
  // Core sequences — shared between action servers and auto_start
  // ------------------------------------------------------------------
  bool run_start_sequence(
    const FeedbackFn & publish_fb,
    const CancelFn & is_canceling,
    std::string & error_out)
  {
    std::unique_lock<std::mutex> lock(transition_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      error_out = "Another transition is already in progress";
      RCLCPP_WARN(get_logger(), "%s", error_out.c_str());
      return false;
    }

    auto fb_state = std::make_shared<int>(0);
    rclcpp::TimerBase::SharedPtr fb_timer;
    if (publish_fb)
    {
      fb_timer = create_wall_timer(1s, [publish_fb, fb_state]()
      {
        publish_fb(++(*fb_state));
      });
    }
    auto cleanup = [&]() { if (fb_timer) fb_timer->cancel(); };

    // Step 1: CONFIGURE
    if (!apply_transition_if_state_not_in(
          Transition::TRANSITION_CONFIGURE,
          {State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_ACTIVE},
          is_canceling, error_out))
    {
      cleanup();
      return false;
    }

    std::this_thread::sleep_for(1s);

    // Step 2: ACTIVATE
    if (!apply_transition_if_state_not_in(
          Transition::TRANSITION_ACTIVATE,
          {State::PRIMARY_STATE_ACTIVE},
          is_canceling, error_out))
    {
      cleanup();
      return false;
    }

    cleanup();
    return true;
  }

  bool run_stop_sequence(
    const FeedbackFn & publish_fb,
    const CancelFn & is_canceling,
    std::string & error_out)
  {
    std::unique_lock<std::mutex> lock(transition_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      error_out = "Another transition is already in progress";
      RCLCPP_WARN(get_logger(), "%s", error_out.c_str());
      return false;
    }

    auto fb_state = std::make_shared<int>(0);
    rclcpp::TimerBase::SharedPtr fb_timer;
    if (publish_fb)
    {
      fb_timer = create_wall_timer(1s, [publish_fb, fb_state]()
      {
        publish_fb(++(*fb_state));
      });
    }
    auto cleanup = [&]() { if (fb_timer) fb_timer->cancel(); };

    // Step 1: DEACTIVATE
    if (!apply_transition_if_state_not_in(
          Transition::TRANSITION_DEACTIVATE,
          {State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_UNCONFIGURED},
          is_canceling, error_out))
    {
      cleanup();
      return false;
    }

    std::this_thread::sleep_for(1s);

    // Step 2: CLEANUP
    if (!apply_transition_if_state_not_in(
          Transition::TRANSITION_CLEANUP,
          {State::PRIMARY_STATE_UNCONFIGURED},
          is_canceling, error_out))
    {
      cleanup();
      return false;
    }

    cleanup();
    return true;
  }

  // ------------------------------------------------------------------
  // Action executors — thin adapters over the sequences above
  // ------------------------------------------------------------------
  void start_teleop_execution(const std::shared_ptr<StartTeleopHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing start_teleop");
    auto result = std::make_shared<StartTeleop::Result>();
    std::string error;

    const bool ok = run_start_sequence(
      [this, goal_handle](int state)
      {
        auto fb = std::make_shared<StartTeleop::Feedback>();
        fb->state = state;
        RCLCPP_INFO(get_logger(), "start_teleop feedback state=%d", state);
        goal_handle->publish_feedback(fb);
      },
      [goal_handle]() { return goal_handle->is_canceling(); },
      error);

    result->message = error;

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "start_teleop canceled");
    }
    else if (ok && rclcpp::ok())
    {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "start_teleop succeeded");
    }
    else
    {
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "start_teleop aborted: %s", error.c_str());
    }
  }

  void stop_teleop_execution(const std::shared_ptr<StopTeleopHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing stop_teleop");
    auto result = std::make_shared<StopTeleop::Result>();
    std::string error;

    const bool ok = run_stop_sequence(
      [this, goal_handle](int state)
      {
        auto fb = std::make_shared<StopTeleop::Feedback>();
        fb->state = state;
        RCLCPP_INFO(get_logger(), "stop_teleop feedback state=%d", state);
        goal_handle->publish_feedback(fb);
      },
      [goal_handle]() { return goal_handle->is_canceling(); },
      error);

    result->message = error;

    if (goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "stop_teleop canceled");
    }
    else if (ok && rclcpp::ok())
    {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "stop_teleop succeeded");
    }
    else
    {
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "stop_teleop aborted: %s", error.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Shared transition helper
  //
  // For every managed node whose current state is NOT in `skip_states`,
  // issue `transition_id`. Returns false on the first failure or cancel.
  // ------------------------------------------------------------------
  bool apply_transition_if_state_not_in(
    uint8_t transition_id,
    std::initializer_list<uint8_t> skip_states,
    const CancelFn & is_canceling,
    std::string & error_out)
  {
    const std::vector<std::string> nodes =
      get_parameter("managed_nodes").as_string_array();

    for (const auto & node_name : nodes)
    {
      if (is_canceling && is_canceling())
      {
        error_out = "Cancelled";
        return false;
      }

      const uint8_t current_state = get_node_state(node_name);
      bool skip = false;
      for (uint8_t s : skip_states)
      {
        if (current_state == s)
        {
          skip = true;
          break;
        }
      }

      if (skip)
      {
        RCLCPP_INFO(get_logger(),
          "Node %s is already in %s. Skipping %s.",
          node_name.c_str(), state_name(current_state), transition_name(transition_id));
        continue;
      }

      RCLCPP_INFO(get_logger(),
        "Node %s (%s) -> %s",
        node_name.c_str(), state_name(current_state), transition_name(transition_id));

      if (!change_state(node_name, transition_id))
      {
        error_out = std::string("Failed to ") + transition_name(transition_id)
                  + " " + node_name;
        RCLCPP_ERROR(get_logger(), "%s", error_out.c_str());
        return false;
      }
    }
    return true;
  }

  // ------------------------------------------------------------------
  // Service helpers
  // ------------------------------------------------------------------
  uint8_t get_node_state(const std::string & node_name)
  {
    auto it = clients_.find(node_name);
    if (it == clients_.end())
    {
      RCLCPP_WARN(get_logger(), "No client for %s", node_name.c_str());
      return State::PRIMARY_STATE_UNKNOWN;
    }
    auto & client = it->second.get_state;

    if (!client->wait_for_service(service_timeout_))
    {
      RCLCPP_WARN(get_logger(), "Service %s not available", client->get_service_name());
      return State::PRIMARY_STATE_UNKNOWN;
    }

    auto future = client->async_send_request(std::make_shared<GetState::Request>());
    if (future.wait_for(request_timeout_) != std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(), "Timeout on %s", client->get_service_name());
      return State::PRIMARY_STATE_UNKNOWN;
    }

    auto response = future.get();
    return response ? response->current_state.id : State::PRIMARY_STATE_UNKNOWN;
  }

  bool change_state(const std::string & node_name, uint8_t transition_id)
  {
    auto it = clients_.find(node_name);
    if (it == clients_.end())
    {
      RCLCPP_WARN(get_logger(), "No client for %s", node_name.c_str());
      return false;
    }
    auto & client = it->second.change_state;

    if (!client->wait_for_service(service_timeout_))
    {
      RCLCPP_WARN(get_logger(), "Service %s not available", client->get_service_name());
      return false;
    }

    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = client->async_send_request(request);
    if (future.wait_for(request_timeout_) != std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(), "Timeout on %s", client->get_service_name());
      return false;
    }

    auto response = future.get();
    return response && response->success;
  }

  // ------------------------------------------------------------------
  // Members
  // ------------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr auto_start_timer_;

  rclcpp_action::Server<StartTeleop>::SharedPtr start_srv_;
  rclcpp_action::Server<StopTeleop>::SharedPtr  stop_srv_;

  std::unordered_map<std::string, NodeClients> clients_;

  std::mutex transition_mutex_;   // serialises start / stop / auto_start

  std::chrono::milliseconds service_timeout_{3000};
  std::chrono::milliseconds request_timeout_{30000};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopLifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}