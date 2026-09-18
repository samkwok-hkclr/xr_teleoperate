#include "dual_arm_hardware_interface/control_api_manager.hpp"

#include <cstdio>
#include <stdexcept>
#include <utility>

namespace dual_arm_hardware_interface
{

std::mutex ControlApiManager::mutex_;
std::weak_ptr<dual_arm_v2_2_sdk::ControlApi> ControlApiManager::instance_;
std::string ControlApiManager::config_prefix_;

std::shared_ptr<dual_arm_v2_2_sdk::ControlApi> ControlApiManager::acquire(
    const std::string& config_prefix)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (auto existing = instance_.lock())
  {
    if (config_prefix != config_prefix_)
    {
      std::fprintf(stderr,
                   "[ControlApiManager] config_prefix mismatch: "
                   "existing='%s', requested='%s'\n",
                   config_prefix_.c_str(), config_prefix.c_str());
      throw std::runtime_error(
          "ControlApi already created with config_prefix='" + config_prefix_ +
          "', refusing to reuse it for config_prefix='" + config_prefix + "'");
    }

    std::fprintf(stderr,
                 "[ControlApiManager] reusing existing ControlApi "
                 "(use_count=%ld)\n",
                 static_cast<long>(existing.use_count()));
    return existing;
  }

  // No live instance: construct a fresh one. The SDK may throw on
  // configuration errors; we let that propagate so on_init() in the hardware
  // component can report a proper lifecycle failure.
  std::fprintf(stderr,
               "[ControlApiManager] creating new ControlApi with "
               "config_prefix='%s'\n",
               config_prefix.c_str());

  auto sp = std::make_shared<dual_arm_v2_2_sdk::ControlApi>(config_prefix);

  config_prefix_ = config_prefix;
  instance_ = sp;

  std::fprintf(stderr,
               "[ControlApiManager] ControlApi created, use_count=%ld\n",
               static_cast<long>(sp.use_count()));

  return sp;
}

std::size_t ControlApiManager::use_count()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return instance_.use_count();
}

std::string ControlApiManager::config_prefix()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_prefix_;
}

void ControlApiManager::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  instance_.reset();
  config_prefix_.clear();
}

}  // namespace dual_arm_hardware_interface