#ifndef DUAL_ARM_HARDWARE_INTERFACE__CONTROL_API_MANAGER_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__CONTROL_API_MANAGER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "control_api.h"

namespace dual_arm_hardware_interface
{

/**
 * @brief Process-wide shared ControlApi handle.
 *
 * The SDK (dual_arm_v2_2_sdk::ControlApi) binds a fixed local UDP port in its
 * constructor. Creating a second instance in the same process therefore fails
 * with EADDRINUSE, and even if the port could be reused, multiple receive
 * threads reading from the same socket would race.
 *
 * Because a single ControlApi already exposes every Group (LEFT_ARM,
 * RIGHT_ARM, HEAD, ELEVATOR, grippers, ...), all hardware-interface
 * components in one process must share one ControlApi instance. This manager
 * provides that shared instance.
 *
 * Lifecycle:
 *   - The first caller of acquire() creates the ControlApi and remembers the
 *     config_prefix used.
 *   - Subsequent callers receive the same shared_ptr as long as at least one
 *     strong reference is alive. If they pass a different config_prefix, the
 *     call throws std::runtime_error.
 *   - When the last shared_ptr is released, the ControlApi is destroyed and
 *     the local UDP port is freed. A later acquire() will recreate it.
 *
 * The manager itself holds only a weak_ptr, so it never keeps the SDK alive
 * by itself.
 */
class ControlApiManager
{
public:
  ControlApiManager() = delete;
  ~ControlApiManager() = delete;

  /**
   * @brief Get the shared ControlApi, creating it on first use.
   *
   * @param config_prefix  Directory that contains config.yaml. Must be
   *                       identical across every call while an instance is
   *                       alive; otherwise std::runtime_error is thrown.
   * @return shared_ptr to the process-wide ControlApi.
   * @throws std::runtime_error on config_prefix mismatch or SDK construction
   *         failure (the original exception is propagated).
   */
  static std::shared_ptr<dual_arm_v2_2_sdk::ControlApi> acquire(
      const std::string& config_prefix);

  /**
   * @brief Number of live shared_ptr owners of the current ControlApi.
   *
   * 0 means no ControlApi exists yet (or it was already released).
   * Useful for diagnostics / logging.
   */
  static std::size_t use_count();

  /**
   * @brief config_prefix used to create the current ControlApi.
   *
   * Empty if no ControlApi has been created yet.
   */
  static std::string config_prefix();

  /**
   * @brief Drop the manager's reference (does not force destruction of the
   *        SDK as long as user shared_ptrs are still alive).
   *
   * Provided for tests. In production the lifetime is driven entirely by
   * shared_ptr ownership.
   */
  static void reset();

private:
  static std::mutex mutex_;
  static std::weak_ptr<dual_arm_v2_2_sdk::ControlApi> instance_;
  static std::string config_prefix_;
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__CONTROL_API_MANAGER_HPP_