#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class CustomLifecycleManager : public rclcpp::Node
{
public:
  CustomLifecycleManager()
  : Node("custom_lifecycle_manager")
  {
    this->declare_parameter<std::vector<std::string>>(
      "managed_nodes",
      {"/left_arm/servo_pose_tracking", "/right_arm/servo_pose_tracking", "/xr_teleop"}
    );
    this->declare_parameter<bool>("auto_start", false);

    start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/start",
      std::bind(&CustomLifecycleManager::start_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&CustomLifecycleManager::stop_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    if (this->get_parameter("auto_start").as_bool())
    {
      RCLCPP_INFO(this->get_logger(), "Auto-start enabled. Scheduling startup sequence in 1.5 seconds...");
      auto_start_timer_ = this->create_wall_timer(
        1500ms, [this]() {
          auto_start_timer_->cancel();
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
          this->start_callback(req, res);
        });
    }

    RCLCPP_INFO(this->get_logger(), "C++ Lifecycle Manager ready. Use services '~/start' and '~/stop'.");
  }

private:
  void start_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Start request received. Checking and updating node states...");
    auto nodes = this->get_parameter("managed_nodes").as_string_array();

    // Step 1: Configure nodes if they aren't already configured or active
    for (const auto & node_name : nodes)
    {
      std::uint8_t current_state = get_node_state(node_name);

      if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
          current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Node %s is already configured. Skipping configuration.", node_name.c_str());
        continue;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Configuring node: %s", node_name.c_str());
        if (!change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
          response->success = false;
          response->message = "Failed to configure " + node_name;
          RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
          return;
        }
      }
    }

    // Step 2: Activate nodes if they aren't already active
    for (const auto & node_name : nodes)
    {
      std::uint8_t current_state = get_node_state(node_name);

      if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Node %s is already active. Skipping activation.", node_name.c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Activating node: %s", node_name.c_str());
        if (!change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
          response->success = false;
          response->message = "Failed to activate " + node_name;
          RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
          return;
        }
      }
    }

    response->success = true;
    response->message = "All managed nodes successfully started.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Stop request received. Deactivating and cleaning up nodes...");
    auto nodes = this->get_parameter("managed_nodes").as_string_array();

    // Stop in reverse order
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it)
    {
      const auto & node_name = *it;
      
      std::uint8_t current_state = get_node_state(node_name);
      if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Deactivating node: %s", node_name.c_str());
        change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }

      current_state = get_node_state(node_name);
      if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        RCLCPP_INFO(this->get_logger(), "Cleaning up node: %s", node_name.c_str());
        if (!change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
        {
          RCLCPP_WARN(this->get_logger(), "Cleanup warning or failure for %s", node_name.c_str());
        }
      }
    }

    response->success = true;
    response->message = "All managed nodes successfully stopped.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  std::uint8_t get_node_state(const std::string & node_name)
  {
    std::string service_name = node_name + "/get_state";
    auto client = this->create_client<lifecycle_msgs::srv::GetState>(service_name);

    if (!client->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Service %s not available.", service_name.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    std::future_status future_status = future.wait_for(CLI_REQ_TIMEOUT);

    switch (future_status)
    {
    case std::future_status::ready:
      break;
    case std::future_status::deferred:
      RCLCPP_WARN(get_logger(), "Failed to call service %s, status: %s", client->get_service_name(), "deferred");
      return false;
    case std::future_status::timeout:
      RCLCPP_WARN(get_logger(), "Failed to call service %s, status: %s", client->get_service_name(), "timeout");
      return false;
    }

    auto response = future.get();

    if (!response)
    {
      RCLCPP_INFO(get_logger(), "Service %s call failed with error: {%s}", client->get_service_name(), "NULL");
      return response->current_state.id;
    }
    
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  bool change_state(const std::string & node_name, std::uint8_t transition_id)
  {
    std::string service_name = node_name + "/change_state";
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

    if (!client->wait_for_service(2s))
    {
      RCLCPP_WARN(this->get_logger(), "Service %s not available.", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = client->async_send_request(request);
    std::future_status future_status = future.wait_for(CLI_REQ_TIMEOUT);

    switch (future_status)
    {
    case std::future_status::ready:
      break;
    case std::future_status::deferred:
      RCLCPP_WARN(get_logger(), "Failed to call service %s, status: %s", client->get_service_name(), "deferred");
      return false;
    case std::future_status::timeout:
      RCLCPP_WARN(get_logger(), "Failed to call service %s, status: %s", client->get_service_name(), "timeout");
      return false;
    }

    auto response = future.get();

    if (!response)
    {
      RCLCPP_INFO(get_logger(), "Service %s call failed with error: {%s}", client->get_service_name(), "NULL");
      return response->success;
    }
    
    return false;
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;

  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(5);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomLifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}