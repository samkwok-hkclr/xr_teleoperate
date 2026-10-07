#include "dual_arm_hardware_interface/telemanip_session.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace dual_arm_hardware_interface
{

// ============================================================
// Helpers
// ============================================================
namespace {

/// Map a TmsStatus code to a human-readable tag (for logs).
const char* tms_status_name(int rc)
{
  switch (rc) {
    case TMS_OK:                return "OK";
    case TMS_ERR_INVALID_ARG:   return "INVALID_ARG";
    case TMS_ERR_NOT_UTF8:      return "NOT_UTF8";
    case TMS_ERR_BUFFER_SMALL:  return "BUFFER_SMALL";
    case TMS_ERR_CONFIG:        return "CONFIG";
    case TMS_ERR_AGENT_TIMEOUT: return "AGENT_TIMEOUT";
    case TMS_ERR_VENDOR_REJECT: return "VENDOR_REJECT";
    case TMS_ERR_TRANSPORT:     return "TRANSPORT";
    case TMS_ERR_PANIC:         return "PANIC";
    default:                    return "UNKNOWN";
  }
}

/// Join a vector of strings for logging.
std::string join(const std::vector<std::string>& v, const char* sep = ",")
{
  std::ostringstream oss;
  for (size_t i = 0; i < v.size(); ++i) {
    oss << v[i];
    if (i + 1 < v.size()) oss << sep;
  }
  return oss.str();
}

}  // namespace

// ============================================================
// Construction / destruction
// ============================================================

TelemanipSession::TelemanipSession(rclcpp::Logger logger,
                                   const std::string& cell_id,
                                   const std::vector<std::string>& device_ids,
                                   const std::string& zenoh_config,
                                   uint64_t period_ms,
                                   uint64_t agent_wait_ms)
  : logger_(logger),
    cell_id_(cell_id),
    device_ids_(device_ids)
{
  RCLCPP_INFO(logger_,
      "[TelemanipSession] creating session: cell='%s' period_ms=%lu "
      "agent_wait_ms=%lu devices=[%s] zenoh='%s'",
      cell_id_.c_str(),
      static_cast<unsigned long>(period_ms),
      static_cast<unsigned long>(agent_wait_ms),
      join(device_ids_).c_str(),
      zenoh_config.c_str());

  // ---- 构造 device_ids 的 const char* 数组 ----
  // 指向 device_ids_ 内部的 std::string::c_str()，在 create 调用期间有效。
  // device_ids_ 是成员，生命周期 >= create 调用。
  std::vector<const char*> device_ptrs;
  device_ptrs.reserve(device_ids_.size());
  for (const auto& d : device_ids_) {
    device_ptrs.push_back(d.c_str());
  }

  // ---- 填 TmsControlConfig ----
  TmsControlConfig cfg;
  tms_control_config_init(&cfg);
  cfg.cell_id       = cell_id_.c_str();
  cfg.zenoh_config  = zenoh_config.c_str();
  cfg.device_ids    = device_ptrs.data();
  cfg.device_count  = device_ptrs.size();
  cfg.period_ms     = period_ms;
  cfg.agent_wait_ms = agent_wait_ms;
  // sender_id / session_id / max_tick_* 保留默认值即可

  // ---- create（阻塞，最长 agent_wait_ms） ----
  const auto t0 = std::chrono::steady_clock::now();
  const int rc = tms_session_create(&cfg, &session_);
  const auto dt_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

  if (rc != TMS_OK || session_ == nullptr)
  {
    session_ = nullptr;   // 防御：即使 C 层误写，也不让悬垂指针存在

    std::ostringstream oss;
    oss << "[TelemanipSession] tms_session_create failed after "
        << dt_ms << " ms: rc=" << rc << " (" << tms_status_name(rc) << "), "
        << "last_error='" << tms_last_error() << "'";

    // 打印针对性提示
    if (rc == TMS_ERR_AGENT_TIMEOUT) {
      oss << "\n  HINT: agent not seen within " << agent_wait_ms << " ms. "
          << "Check: (a) zenohd running; (b) telemanip-agent-cuarm_2_2 up; "
          << "(c) cell_id matches the agent's.";
    } else if (rc == TMS_ERR_CONFIG) {
      oss << "\n  HINT: config rejected. Check: "
          << "(a) period_ms matches the agent's; "
          << "(b) every device_id exists on the agent.";
    }

    RCLCPP_ERROR(logger_, "%s", oss.str().c_str());
    throw std::runtime_error(oss.str());
  }

  RCLCPP_INFO(logger_,
      "[TelemanipSession] session created in %.1f ms",
      dt_ms);
}

TelemanipSession::~TelemanipSession()
{
  if (session_ == nullptr) {
    return;
  }

  // C ABI: close() aborts the background tasks; destroy() frees the handle.
  // Both must not race with any other call — since this is a destructor,
  // no other call can happen concurrently by design.
  const int rc_close = tms_session_close(session_);
  if (rc_close != TMS_OK) {
    // 不抛异常，析构里只 log
    RCLCPP_WARN(logger_,
        "[TelemanipSession] tms_session_close returned rc=%d (%s)",
        rc_close, tms_status_name(rc_close));
  }

  tms_session_destroy(session_);
  session_ = nullptr;

  RCLCPP_INFO(logger_, "[TelemanipSession] session destroyed");
}

// ============================================================
// Status queries
// ============================================================

bool TelemanipSession::agent_online()
{
  if (!valid()) return false;

  bool online = false;
  const int rc = tms_session_agent_online(session_, &online);
  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] agent_online failed: rc=%d (%s), err='%s'",
        rc, tms_status_name(rc), tms_last_error());
    return false;
  }
  return online;
}

std::optional<uint64_t> TelemanipSession::tick()
{
  if (!valid()) return std::nullopt;

  uint64_t t = 0;
  const int rc = tms_session_tick(session_, &t);
  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] tick failed: rc=%d (%s), err='%s'",
        rc, tms_status_name(rc), tms_last_error());
    return std::nullopt;
  }
  return t;
}

// ============================================================
// Commands
// ============================================================

bool TelemanipSession::group_enable(const std::string& device_id, bool enable)
{
  if (!valid()) return false;

  int32_t staged = 0;
  const int rc = tms_cuarm_2_2_group_enable(
      session_, device_id.c_str(), enable, &staged);

  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] group_enable('%s', %d) failed: rc=%d (%s), err='%s'",
        device_id.c_str(), static_cast<int>(enable),
        rc, tms_status_name(rc), tms_last_error());
    return false;
  }

  if (staged != 0) {
    // 本地拒绝：设备不在握手名单 / 类型未注册 / 负载为空
    // 对于 group_enable(false)，SDK 会返回 staged=1（这是正常的语义，见测试程序）
    if (enable) {
      RCLCPP_WARN(logger_,
          "[TelemanipSession] group_enable('%s', true) locally rejected",
          device_id.c_str());
    }
    return false;
  }

  return true;
}

bool TelemanipSession::stage_joint(const std::string& device_id,
                                   const float* joints, size_t n,
                                   bool follow,
                                   uint32_t trajectory_mode,
                                   uint32_t radio)
{
  if (!valid()) return false;
  if (joints == nullptr || n == 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_joint('%s'): null or empty joints",
        device_id.c_str());
    return false;
  }

  int32_t staged = 0;
  const int rc = tms_cuarm_2_2_joint_cmd(
      session_, device_id.c_str(),
      joints, n,
      follow, trajectory_mode, radio,
      &staged);

  if (rc != TMS_OK) {
    // 编码阶段拒绝（关节数不对 / 非有限值 / follow 模式下 radio 越界）
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_joint('%s', n=%zu, follow=%d, mode=%u, radio=%u) "
        "failed: rc=%d (%s), err='%s'",
        device_id.c_str(), n, static_cast<int>(follow),
        trajectory_mode, radio,
        rc, tms_status_name(rc), tms_last_error());
    return false;
  }

  if (staged != 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_joint('%s') locally rejected "
        "(device/type not registered?)",
        device_id.c_str());
    return false;
  }

  return true;
}

bool TelemanipSession::stage_gripper_position(const std::string& device_id,
                                              uint32_t position,
                                              bool block,
                                              int32_t timeout_sec)
{
  if (!valid()) return false;

  // block=true 需要 timeout_sec > 0，否则编码阶段会拒绝
  if (block && timeout_sec <= 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_position('%s'): "
        "block=true requires timeout_sec > 0 (got %d)",
        device_id.c_str(), timeout_sec);
    return false;
  }

  int32_t staged = 0;
  const int rc = tms_cuarm_2_2_gripper_position(
      session_, device_id.c_str(),
      position, block, timeout_sec,
      &staged);

  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_position('%s', pos=%u, block=%d, timeout=%d) "
        "failed: rc=%d (%s), err='%s'",
        device_id.c_str(), position,
        static_cast<int>(block), timeout_sec,
        rc, tms_status_name(rc), tms_last_error());
    return false;
  }

  if (staged != 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_position('%s') locally rejected",
        device_id.c_str());
    return false;
  }

  return true;
}

bool TelemanipSession::stage_gripper_config(const std::string& device_id,
                                            uint32_t speed,
                                            uint32_t force,
                                            bool block,
                                            int32_t timeout_sec)
{
  if (!valid()) return false;

  if (block && timeout_sec <= 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_config('%s'): "
        "block=true requires timeout_sec > 0 (got %d)",
        device_id.c_str(), timeout_sec);
    return false;
  }

  int32_t staged = 0;
  const int rc = tms_cuarm_2_2_gripper_config(
      session_, device_id.c_str(),
      speed, force, block, timeout_sec,
      &staged);

  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_config('%s', speed=%u, force=%u, block=%d, timeout=%d) "
        "failed: rc=%d (%s), err='%s'",
        device_id.c_str(), speed, force,
        static_cast<int>(block), timeout_sec,
        rc, tms_status_name(rc), tms_last_error());
    return false;
  }

  if (staged != 0) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] stage_gripper_config('%s') locally rejected",
        device_id.c_str());
    return false;
  }

  return true;
}

// ============================================================
// State reads
// ============================================================

bool TelemanipSession::read_joint(const std::string& device_id,
                                  float* out, size_t out_cap,
                                  bool* present)
{
  if (!valid()) return false;
  if (out == nullptr || out_cap < tms_cuarm_2_2_joint_count()) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] read_joint('%s'): out buffer too small "
        "(need >= %u, got %zu)",
        device_id.c_str(),
        tms_cuarm_2_2_joint_count(), out_cap);
    return false;
  }
  if (present == nullptr) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] read_joint('%s'): present is null",
        device_id.c_str());
    return false;
  }

  bool pres = false;
  const int rc = tms_cuarm_2_2_session_joints(
      session_, device_id.c_str(),
      out, out_cap,
      &pres);

  if (rc != TMS_OK) {
    // 注意：head 会返回 present=false 但 rc=OK；此处 rc!=OK 才是真正的错误
    RCLCPP_WARN(logger_,
        "[TelemanipSession] read_joint('%s') failed: rc=%d (%s), err='%s'",
        device_id.c_str(),
        rc, tms_status_name(rc), tms_last_error());
    *present = false;
    return false;
  }

  *present = pres;
  return true;
}

void TelemanipSession::JointStateDeleter::operator()(TmsCuarm2_2JointState* p) const noexcept
{
  if (p != nullptr) {
    tms_cuarm_2_2_joint_state_free(p);
  }
}

TelemanipSession::JointStatePtr
TelemanipSession::read_state(const std::string& device_id)
{
  if (!valid()) return JointStatePtr(nullptr);

  TmsCuarm2_2JointState* state = nullptr;
  const int rc = tms_cuarm_2_2_session_state(
      session_, device_id.c_str(), &state);

  if (rc != TMS_OK) {
    RCLCPP_WARN(logger_,
        "[TelemanipSession] read_state('%s') failed: rc=%d (%s), err='%s'",
        device_id.c_str(),
        rc, tms_status_name(rc), tms_last_error());
    return JointStatePtr(nullptr);
  }

  // *state == nullptr + rc == OK 表示"尚无快照"（idle group），不是错误
  return JointStatePtr(state);
}

}  // namespace dual_arm_hardware_interface