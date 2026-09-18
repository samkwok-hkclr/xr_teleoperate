#include "dual_arm_hardware_interface/hardware_base.hpp"

#include <exception>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>

#include "dual_arm_hardware_interface/control_api_manager.hpp"

namespace dual_arm_hardware_interface
{

// ============================================================
// 
// ============================================================
HardwareBase::HardwareBase()
  : logger_(rclcpp::get_logger("dual_arm_hardware_interface"))
{
  // rclcpp::Logger has no public default constructor, so it must be
  // initialised here explicitly. on_init() will refine the logger name
  // using the component's own get_name().
}

// ============================================================
// on_init
// ============================================================
CallbackReturn HardwareBase::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger(std::string(get_name()) + "_hardware_interface");

  RCLCPP_INFO(logger_, "[%s] on_init: name=%s, joints=%zu", component_name(), get_name().c_str(), info.joints.size());

  // ---- 必需参数：config_prefix ----
  auto it = info.hardware_parameters.find("config_prefix");
  if (it == info.hardware_parameters.end() || it->second.empty())
  {
    RCLCPP_ERROR(logger_, "[%s] missing required hardware parameter 'config_prefix' (directory containing config.yaml)", component_name());
    return CallbackReturn::ERROR;
  }
  config_prefix_ = it->second;

  RCLCPP_INFO(logger_, "[%s] config_prefix = %s", component_name(), config_prefix_.c_str());

  // ---- Get shared ControlApi ----
  try
  {
    api_ = ControlApiManager::acquire(config_prefix_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_, "[%s] failed to acquire ControlApi: %s", component_name(), e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "[%s] acquired ControlApi, shared use_count=%ld", component_name(), static_cast<long>(api_.use_count()));

  const CallbackReturn init_result = on_init_impl();
  if (init_result != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[%s] on_init_impl failed", component_name());
    return init_result;
  }

  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_configure
// ============================================================
CallbackReturn HardwareBase::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "[%s] on_configure", component_name());

  if (!api_)
  {
    RCLCPP_ERROR(logger_, "[%s] ControlApi not available in on_configure", component_name());
    return CallbackReturn::ERROR;
  }

  const CallbackReturn result = on_configure_impl();
  if (result == CallbackReturn::SUCCESS)
  {
    RCLCPP_INFO(logger_, "[%s] on_configure done", component_name());
  }
  else
  {
    RCLCPP_ERROR(logger_, "[%s] on_configure failed in component hook", component_name());
  }
  return result;
}

// ============================================================
// on_activate
// ============================================================
CallbackReturn HardwareBase::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "[%s] on_activate", component_name());

  if (!api_)
  {
    RCLCPP_ERROR(logger_, "[%s] ControlApi not available in on_activate", component_name());
    return CallbackReturn::ERROR;
  }

  const CallbackReturn result = on_activate_impl();
  if (result != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "[%s] on_activate failed in component hook", component_name());
    return result;
  }

  RCLCPP_INFO(logger_, "[%s] on_activate done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate
// ============================================================
CallbackReturn HardwareBase::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "[%s] on_deactivate", component_name());

  // 无论子类钩子成功与否，都标记为不再激活，避免 read/write 继续下发命令
  const CallbackReturn result = on_deactivate_impl();

  if (result != CallbackReturn::SUCCESS)
  {
    RCLCPP_WARN(logger_,
                "[%s] on_deactivate component hook reported failure, "
                "component is marked inactive anyway",
                component_name());
  }
  else
  {
    RCLCPP_INFO(logger_, "[%s] on_deactivate done", component_name());
  }
  return result;
}

// ============================================================
// on_cleanup
// ============================================================
CallbackReturn HardwareBase::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "[%s] on_cleanup", component_name());

  const CallbackReturn result = on_cleanup_impl();

  // 释放对这个硬件组件的引用。若这是最后一个引用，ControlApiManager
  // 内部的 weak_ptr 会失效，SDK 被析构，UDP 端口释放。
  api_.reset();

  RCLCPP_INFO(logger_, "[%s] on_cleanup done", component_name());
  return result;
}

// ============================================================
// on_shutdown
// ============================================================
CallbackReturn HardwareBase::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "[%s] on_shutdown", component_name());

  if (is_active())
  {
    on_deactivate_impl();
  }

  api_.reset();

  RCLCPP_INFO(logger_, "[%s] on_shutdown done", component_name());
  return CallbackReturn::SUCCESS;
}

// ============================================================
// is_active
// ============================================================
bool HardwareBase::is_active() const
{
  return get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

}  // namespace dual_arm_hardware_interface