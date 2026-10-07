#ifndef DUAL_ARM_HARDWARE_INTERFACE__TELEMANIP_SESSION_HPP_
#define DUAL_ARM_HARDWARE_INTERFACE__TELEMANIP_SESSION_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// C ABI headers (package-local)
#include "telemanip_control_cuarm_2_2.h"

namespace dual_arm_hardware_interface
{

/**
 * @brief RAII wrapper around the telemanip cuarm_2_2 C ABI session.
 *
 * Ownership
 * ---------
 * Owns a TmsSession* for its whole lifetime (created in ctor, destroyed in dtor).
 * Non-copyable and non-movable: the session owns background threads, so moving
 * or copying it would be an error-prone operation with no upside here.
 *
 * Threading
 * ---------
 * The underlying C ABI is internally synchronized: every tms_* call is safe
 * to make from multiple threads. This wrapper adds no additional locking.
 *
 * Lifetime rule: the C ABI requires tms_session_destroy to be the LAST call
 * naming the handle and NOT to race with any other call. We honor that by
 * putting destroy() in the destructor only — never call any method after the
 * object goes out of scope.
 *
 * Error policy
 * ------------
 * - Constructor failure -> throws std::runtime_error (unrecoverable).
 * - Runtime failures    -> log a WARN and return false / nullptr / nullopt.
 */
class TelemanipSession
{
public:
  // ============================================================
  // Construction / destruction
  // ============================================================

  /**
   * @brief Create a session (blocking).
   *
   * Blocks for up to @p agent_wait_ms waiting for the agent's liveliness token,
   * then queries its CellConfig and validates every device_id + period_ms.
   *
   * @param logger          ROS2 logger for diagnostics.
   * @param cell_id         Cell id (e.g. "cell-cuarm1"). Must match the agent.
   * @param device_ids      Device names, e.g. {"left", "right", "head"}.
   * @param zenoh_config    Path to zenoh.json5 (absolute recommended).
   * @param period_ms       Command period, must equal the agent's (e.g. 10).
   * @param agent_wait_ms   Max wait for the agent token (100..=300000).
   *
   * @throws std::runtime_error on any TMS_ERR_* from tms_session_create.
   */
  TelemanipSession(rclcpp::Logger logger,
                   const std::string& cell_id,
                   const std::vector<std::string>& device_ids,
                   const std::string& zenoh_config,
                   uint64_t period_ms,
                   uint64_t agent_wait_ms = 10000);

  ~TelemanipSession();

  TelemanipSession(const TelemanipSession&)            = delete;
  TelemanipSession& operator=(const TelemanipSession&) = delete;
  TelemanipSession(TelemanipSession&&)                 = delete;
  TelemanipSession& operator=(TelemanipSession&&)      = delete;

  // ============================================================
  // Status queries
  // ============================================================

  /// True when the underlying TmsSession* is non-null.
  bool valid() const noexcept { return session_ != nullptr; }

  /// True while an agent liveliness token is present on the cell.
  bool agent_online();

  /// Command beat counter (advances even when nothing is staged).
  std::optional<uint64_t> tick();

  // ============================================================
  // Commands (stage only — never block, never send)
  // ============================================================

  /**
   * @brief Stage a group enable / disable target for the next beat.
   *
   * enable=false does NOT move the arm; it just stops accepting joint commands
   * for that group and lets the agent hold the last commanded pose.
   *
   * @return true if the target was accepted (staged == 0).
   */
  bool group_enable(const std::string& device_id, bool enable);

  /**
   * @brief Stage a joint target for the next beat.
   *
   * @param device_id        "left" / "right" (7 joints) or "head" (2 joints).
   * @param joints           Radian values, length must match the group.
   * @param n                Length of @p joints.
   * @param follow           false = one quintic point, true = streaming.
   * @param trajectory_mode  follow mode: 0 direct, 1 curve fit, 2 filter.
   * @param radio            curve fit 0..=100, filter 0..=999 (ignored for 0).
   *
   * @return true if accepted (staged == 0).
   */
  bool stage_joint(const std::string& device_id,
                   const float* joints, size_t n,
                   bool follow,
                   uint32_t trajectory_mode,
                   uint32_t radio);

  /**
   * @brief Stage a gripper position target for the next beat.
   *
   * position: 0 = open, 1000 = closed.
   * Legal only on "left" / "right"; the agent silently rejects "head".
   *
   * NOTE: always call with block=false from the RT control loop. block=true
   * would make the agent's dispatch task wait for the SDK to return, which
   * blocks all three groups (they share one Client lock) and skips beats.
   *
   * @return true if accepted (staged == 0).
   */
  bool stage_gripper_position(const std::string& device_id,
                              uint32_t position,
                              bool block = false,
                              int32_t timeout_sec = 0);

  /**
   * @brief Stage a gripper speed/force config for the next beat.
   *
   * speed, force: 1..=1000.
   * Same block semantics as stage_gripper_position().
   *
   * @return true if accepted (staged == 0).
   */
  bool stage_gripper_config(const std::string& device_id,
                            uint32_t speed,
                            uint32_t force,
                            bool block = false,
                            int32_t timeout_sec = 0);

  // ============================================================
  // State reads
  // ============================================================

  /**
   * @brief Zero-allocation read of one ARM group's 7 joint angles (radians).
   *
   * Only valid for arm groups. A head snapshot is 2 joints, so @p present
   * stays false and @p out is untouched — use read_state() for the head.
   *
   * @param out      Buffer with room for 7 floats (never written partially).
   * @param out_cap  Must be >= tms_cuarm_2_2_joint_count() (7).
   * @param present  Set to false when there is no snapshot yet.
   *
   * @return true if the CALL succeeded (regardless of @p present).
   */
  bool read_joint(const std::string& device_id,
                  float* out, size_t out_cap,
                  bool* present);

  struct JointStateDeleter
  {
    void operator()(TmsCuarm2_2JointState* p) const noexcept;
  };
  using JointStatePtr = std::unique_ptr<TmsCuarm2_2JointState, JointStateDeleter>;

  /**
   * @brief Allocated read of one group's full decoded state.
   *
   * Returns nullptr if there is no snapshot yet (idle group — not an error).
   * Works for both arms (7 joints + gripper) and the head (2 joints, no gripper).
   */
  JointStatePtr read_state(const std::string& device_id);

  // ============================================================
  // Escape hatch
  // ============================================================

  /// Direct access to the underlying C handle. Do not destroy it.
  TmsSession* raw() const noexcept { return session_; }

private:
  rclcpp::Logger logger_;
  TmsSession*    session_{nullptr};

  // Kept alive for the session's lifetime. The C ABI only requires these
  // strings to live through tms_session_create, but we copy them anyway
  // to avoid any dangling-pointer trap in the caller's parameters.
  std::string              cell_id_;
  std::vector<std::string> device_ids_;
};

}  // namespace dual_arm_hardware_interface

#endif  // DUAL_ARM_HARDWARE_INTERFACE__TELEMANIP_SESSION_HPP_