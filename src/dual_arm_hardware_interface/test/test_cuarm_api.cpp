// ============================================================
// test_cuarm_api.cpp
//
// 独立测试 telemanip C API + zenoh 链路的最小可执行程序。
// 不依赖 ROS2，直接从命令行运行。
//
// 编译（动态库）：
//   g++ -std=c++17 -o test_cuarm_api test/test_cuarm_api.cpp \
//       -Iinclude/capi -Llib -ltelemanip_capi_cuarm_2_2 -lm
//
// 运行前设置库路径：
//   export LD_LIBRARY_PATH=$PWD/lib:$LD_LIBRARY_PATH
//
// 运行：
//   ./test_cuarm_api <zenoh.json5路径> <cell_id> [--test-motion]
//
// 例：
//   ./test_cuarm_api config/zenoh.json5 cell-cuarm1
//   ./test_cuarm_api config/zenoh.json5 cell-cuarm1 --test-motion
//
// 阶段：
//   [0] 打印库信息
//   [1] 解析参数
//   [2] tms_session_create（阻塞等待 agent）
//   [3] agent_online 检查
//   [4] query_config 拿 CellConfig
//   [5] group_enable("left", true)
//   [6] 轮询 session_joints("left") 直到拿到第一个 state
//   [7] hold 当前值，验证链路（不动）
//   [8] (--test-motion) +0.01 rad 微动，验证下发链路
//   [9] group_enable("left", false)
//   [10] close + destroy
//
// 安全提示：
//   - 默认只做 [7]（hold 不动）
//   - --test-motion 会真的移动 0.01 rad（约 0.57°），真机请确认安全
//   - 不发送任何"回零"或大幅运动
// ============================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>

#include "telemanip_control_cuarm_2_2.h"

// ------------------------------------------------------------
// 全局状态（用于回调）
// ------------------------------------------------------------
namespace {

struct AppState {
    std::atomic<uint64_t> state_cb_count{0};
    std::atomic<uint64_t> status_cb_count{0};
    std::atomic<uint64_t> last_state_tick{0};
    std::mutex           print_mutex;
};

AppState g_state;

// ------------------------------------------------------------
// 打印工具
// ------------------------------------------------------------
void print_sep(const char* title)
{
    std::printf("\n=== %s ===\n", title);
}

void print_err(const char* op, int rc)
{
    std::printf("  [FAIL] %s: rc=%d (%s), last_error=%s\n",
                op, rc,
                rc == TMS_OK ? "TMS_OK" : "ERR",
                tms_last_error());
}

bool check(int rc, const char* op)
{
    if (rc == TMS_OK) {
        std::printf("  [ OK ] %s\n", op);
        return true;
    }
    print_err(op, rc);
    return false;
}

// ------------------------------------------------------------
// 状态回调（在 SDK 后台线程调用，info 只在本次调用内有效）
// ------------------------------------------------------------
void on_state(const TmsEnvelopeInfo* info, void* /*user_data*/)
{
    if (!info) return;
    g_state.state_cb_count.fetch_add(1, std::memory_order_relaxed);
    g_state.last_state_tick.store(info->tick, std::memory_order_relaxed);

    // 只打印前 3 次和每 100 次
    const uint64_t n = g_state.state_cb_count.load();
    if (n <= 3 || (n % 100) == 0) {
        std::lock_guard<std::mutex> lk(g_state.print_mutex);
        std::printf("  [state cb] #%lu tick=%lu items=%zu sender=%s\n",
                    static_cast<unsigned long>(n),
                    static_cast<unsigned long>(info->tick),
                    info->item_count,
                    info->sender_id ? info->sender_id : "?");
    }
}

void on_status(const TmsEnvelopeInfo* info, void* /*user_data*/)
{
    if (!info) return;
    g_state.status_cb_count.fetch_add(1, std::memory_order_relaxed);

    // 解码第一条 status
    for (size_t i = 0; i < info->item_count; ++i) {
        const TmsItemInfo& it = info->items[i];
        if (it.type == tms_cuarm_2_2_type_status() && it.payload) {
            TmsCuarm2_2Status st{};
            if (tms_cuarm_2_2_decode_status(it.payload, it.payload_len, &st) == TMS_OK) {
                const uint64_t n = g_state.status_cb_count.load();
                if (n <= 3 || (n % 100) == 0) {
                    std::lock_guard<std::mutex> lk(g_state.print_mutex);
                    std::printf("  [status cb] #%lu dev=%s code=%d conn=%d wd=%d sent_ok=%lu sent_fail=%lu\n",
                                static_cast<unsigned long>(n),
                                it.device_id ? it.device_id : "?",
                                st.last_code,
                                static_cast<int>(st.arm_connected),
                                static_cast<int>(st.watchdog),
                                static_cast<unsigned long>(st.sent_ok),
                                static_cast<unsigned long>(st.sent_fail));
                }
            }
        }
    }
}

// ------------------------------------------------------------
// [0] 库信息
// ------------------------------------------------------------
void stage0_library_info()
{
    print_sep("[0] library info");
    std::printf("  tms_version()   = %s\n", tms_version());
    std::printf("  tms_abi_vendor()= %s\n", tms_abi_vendor());
    std::printf("  cuarm vendor    = 0x%04x (expect 0x0003)\n",
                tms_cuarm_2_2_vendor());
    std::printf("  joint_count     = %u (expect 7)\n",
                tms_cuarm_2_2_joint_count());
    std::printf("  head_joint_count= %u (expect 2)\n",
                tms_cuarm_2_2_head_joint_count());
}

// ------------------------------------------------------------
// [1] 解析命令行
// ------------------------------------------------------------
struct TestConfig {
    std::string zenoh_config;
    std::string cell_id;
    std::string test_device{"left"};
    std::vector<std::string> devices{"left", "right", "head"};
    uint64_t period_ms     = 50;
    uint64_t agent_wait_ms = 10000;   // 10s，比默认 30s 快
    bool     test_motion   = false;
};

bool stage1_parse_args(int argc, char** argv, TestConfig& cfg)
{
    print_sep("[1] parse arguments");

    if (argc < 3) {
        std::printf("  usage: %s <zenoh.json5> <cell_id> [--test-motion]\n", argv[0]);
        return false;
    }

    cfg.zenoh_config = argv[1];
    cfg.cell_id      = argv[2];

    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--test-motion") == 0) {
            cfg.test_motion = true;
        }
    }

    std::printf("  zenoh_config  = %s\n", cfg.zenoh_config.c_str());
    std::printf("  cell_id       = %s\n", cfg.cell_id.c_str());
    std::printf("  period_ms     = %lu\n", static_cast<unsigned long>(cfg.period_ms));
    std::printf("  agent_wait_ms = %lu\n", static_cast<unsigned long>(cfg.agent_wait_ms));
    std::printf("  devices       = ");
    for (size_t i = 0; i < cfg.devices.size(); ++i) {
        std::printf("%s%s", cfg.devices[i].c_str(), (i + 1 < cfg.devices.size() ? "," : ""));
    }
    std::printf("\n");
    std::printf("  test_device   = %s\n", cfg.test_device.c_str());
    std::printf("  test_motion   = %s\n", cfg.test_motion ? "YES (will move 0.01 rad)" : "no (hold only)");

    return true;
}

// ------------------------------------------------------------
// [2] 创建 session
// ------------------------------------------------------------
bool stage2_create_session(const TestConfig& cfg, TmsSession** out)
{
    print_sep("[2] tms_session_create");
    std::printf("  (this will block up to %lu ms waiting for agent)\n",
                static_cast<unsigned long>(cfg.agent_wait_ms));

    TmsControlConfig control_cfg;
    tms_control_config_init(&control_cfg);

    // 准备设备名数组（字符串指针必须在 create 期间有效）
    std::vector<const char*> device_ptrs;
    device_ptrs.reserve(cfg.devices.size());
    for (const auto& d : cfg.devices) {
        device_ptrs.push_back(d.c_str());
    }

    control_cfg.cell_id        = cfg.cell_id.c_str();
    control_cfg.zenoh_config   = cfg.zenoh_config.c_str();
    control_cfg.device_ids     = device_ptrs.data();
    control_cfg.device_count   = device_ptrs.size();
    control_cfg.period_ms      = cfg.period_ms;
    control_cfg.agent_wait_ms  = cfg.agent_wait_ms;

    const auto t0 = std::chrono::steady_clock::now();
    const int rc = tms_session_create(&control_cfg, out);
    const auto dt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf("  took %.1f ms\n", dt_ms);

    if (!check(rc, "tms_session_create")) {
        if (rc == TMS_ERR_AGENT_TIMEOUT) {
            std::printf("  HINT: agent not seen. Check:\n");
            std::printf("    - zenohd is running and reachable (%s)\n", cfg.zenoh_config.c_str());
            std::printf("    - telemanip-agent-cuarm_2_2 is running\n");
            std::printf("    - cell_id matches (%s)\n", cfg.cell_id.c_str());
        } else if (rc == TMS_ERR_CONFIG) {
            std::printf("  HINT: config rejected. Check:\n");
            std::printf("    - period_ms=%lu matches agent's\n",
                        static_cast<unsigned long>(cfg.period_ms));
            std::printf("    - device_ids all exist on agent\n");
        }
        return false;
    }
    return true;
}

// ------------------------------------------------------------
// [3] agent online 检查
// ------------------------------------------------------------
bool stage3_agent_online(TmsSession* s)
{
    print_sep("[3] tms_session_agent_online");
    bool online = false;
    if (!check(tms_session_agent_online(s, &online), "tms_session_agent_online")) {
        return false;
    }
    std::printf("  agent_online = %s\n", online ? "true" : "false");
    return online;
}

// ------------------------------------------------------------
// [4] query CellConfig
// ------------------------------------------------------------
bool stage4_query_config(TmsSession* s)
{
    print_sep("[4] tms_session_query_config");

    TmsEnvelopeInfo* env = nullptr;
    if (!check(tms_session_query_config(s, &env), "tms_session_query_config")) {
        return false;
    }
    if (!env || env->item_count == 0) {
        std::printf("  [WARN] empty envelope\n");
        if (env) tms_envelope_free(env);
        return false;
    }

    const TmsItemInfo& item = env->items[0];
    TmsCellConfig* cc = nullptr;
    if (!check(tms_decode_cell_config(item.payload, item.payload_len, &cc),
               "tms_decode_cell_config")) {
        tms_envelope_free(env);
        return false;
    }

    std::printf("  cell_id         = %s\n", cc->cell_id ? cc->cell_id : "?");
    std::printf("  period_ms       = %u\n", cc->period_ms);
    std::printf("  state_period_ms = %u\n", cc->state_period_ms);
    std::printf("  status_hz       = %u\n", cc->status_hz);
    std::printf("  watchdog_mult   = %u  (window = %u ms)\n",
                cc->watchdog_mult, cc->watchdog_mult * cc->period_ms);
    std::printf("  max_tick_bytes  = %u\n", cc->max_tick_bytes);
    std::printf("  max_tick_items  = %u\n", cc->max_tick_items);
    std::printf("  device_count    = %zu\n", cc->device_count);
    for (size_t i = 0; i < cc->device_count; ++i) {
        std::printf("    [%zu] %s\n", i, cc->device_ids[i] ? cc->device_ids[i] : "?");
    }

    tms_cell_config_free(cc);
    tms_envelope_free(env);
    return true;
}

// ------------------------------------------------------------
// [5]/[9] group_enable
// ------------------------------------------------------------
bool stage5_group_enable(TmsSession* s, const std::string& dev, bool enable)
{
    print_sep(enable ? "[5] group_enable(true)" : "[9] group_enable(false)");

    int32_t staged = 0;
    const int rc = tms_cuarm_2_2_group_enable(s, dev.c_str(), enable, &staged);

    if (!check(rc, "tms_cuarm_2_2_group_enable")) return false;

    std::printf("  staged=%d (0=accepted, 1=locally rejected)\n", staged);
    if (staged != 0) {
        std::printf("  [WARN] target was locally rejected\n");
    }
    return true;
}

// ------------------------------------------------------------
// [6] 轮询 state
// ------------------------------------------------------------
bool stage6_wait_state(TmsSession* s, const std::string& dev, int timeout_ms)
{
    print_sep("[6] wait for first state");

    const int    n_joints  = static_cast<int>(tms_cuarm_2_2_joint_count());
    const auto   deadline  = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(timeout_ms);
    int          polls     = 0;
    std::vector<float> joints(n_joints, 0.0F);
    bool         present   = false;

    while (std::chrono::steady_clock::now() < deadline) {
        joints.assign(n_joints, 0.0F);
        present = false;

        const int rc = tms_cuarm_2_2_session_joints(
            s, dev.c_str(),
            joints.data(), joints.size(),
            &present);

        if (rc != TMS_OK) {
            print_err("tms_cuarm_2_2_session_joints", rc);
            return false;
        }

        if (present) {
            std::printf("  got state after %d polls (%.0f ms)\n",
                        polls, polls * 10.0);
            std::printf("  joints(rad): ");
            for (size_t i = 0; i < joints.size(); ++i) {
                std::printf("%+.4f%s", joints[i], (i + 1 < joints.size() ? " " : ""));
            }
            std::printf("\n");
            return true;
        }

        ++polls;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::printf("  [FAIL] no state within %d ms (polls=%d)\n", timeout_ms, polls);
    return false;
}

// ------------------------------------------------------------
// [7] hold 当前值，确认链路
// ------------------------------------------------------------
bool stage7_hold(TmsSession* s, const std::string& dev)
{
    print_sep("[7] hold current value (no motion)");

    const int n = static_cast<int>(tms_cuarm_2_2_joint_count());
    std::vector<float> before(n, 0.0F), after(n, 0.0F);
    bool present = false;

    if (!check(tms_cuarm_2_2_session_joints(s, dev.c_str(),
                                            before.data(), before.size(), &present),
               "read before")) return false;
    if (!present) {
        std::printf("  [FAIL] no state before\n");
        return false;
    }

    int32_t staged = 0;
    if (!check(tms_cuarm_2_2_joint_cmd(s, dev.c_str(),
                                       before.data(), before.size(),
                                       /*follow=*/false,
                                       /*trajectory_mode=*/0,
                                       /*radio=*/0,
                                       &staged),
               "stage hold")) return false;
    std::printf("  staged=%d\n", staged);
    if (staged != 0) {
        std::printf("  [FAIL] locally rejected\n");
        return false;
    }

    // 等两个 beat 周期以上
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!check(tms_cuarm_2_2_session_joints(s, dev.c_str(),
                                            after.data(), after.size(), &present),
               "read after")) return false;
    if (!present) {
        std::printf("  [FAIL] no state after\n");
        return false;
    }

    // 比较
    float max_diff = 0.0F;
    for (int i = 0; i < n; ++i) {
        const float d = std::fabs(after[i] - before[i]);
        if (d > max_diff) max_diff = d;
    }
    std::printf("  max |after - before| = %.6f rad\n", max_diff);

    if (max_diff > 0.01F) {   // 1 mrad = 0.057°
        std::printf("  [FAIL] hold command caused motion (diff > 0.01 rad)\n");
        std::printf("  after: ");
        for (int i = 0; i < n; ++i) std::printf("%+.4f ", after[i]);
        std::printf("\n");
        return false;
    }

    std::printf("  [OK] hold verified\n");
    return true;
}

// ------------------------------------------------------------
// [8] 左右臂同步移动 target_deg 度 @ 100 Hz（均匀关节步长）
//
//   - 频率: 100 Hz (10 ms / step)
//   - 时长: duration_ms（默认 2000 ms → 200 步）
//   - 每步: target_deg / n_steps 度
//   - 每步对所有关节同时 stage 相同 offset
//   - follow=true, trajectory_mode=2 (filter), radio=500
//
//   ★ 单位：全部用度（deg）
//
//   安全性:
//     agent 参数 max_start_deg=5, max_step_deg=10
//     每步 10 / 200 = 0.05 度 < 10 度 ✅
//     第一步 offset < 5 度 ✅
// ------------------------------------------------------------
bool stage8_move_both_10deg(TmsSession* s,
                            const std::string& left_dev,
                            const std::string& right_dev,
                            int    duration_ms    = 2000,
                            float  target_deg     = 10.0F,
                            uint32_t trajectory_mode = 2,
                            uint32_t radio        = 500)
{
    print_sep("[8] move BOTH arms +10 deg @ 100 Hz");

    constexpr int   kPeriodMs = 10;              // 100 Hz
    const int       n_steps   = std::max(1, duration_ms / kPeriodMs);
    const int       n_joints  = static_cast<int>(tms_cuarm_2_2_joint_count()); // 7
    const float     step_deg  = target_deg / static_cast<float>(n_steps);

    std::printf("  duration      = %d ms (%d steps @ 100 Hz)\n",
                duration_ms, n_steps);
    std::printf("  target offset = %.3f deg (all joints)\n", target_deg);
    std::printf("  step / cycle  = %.4f deg\n", step_deg);
    std::printf("  follow=true  trajectory_mode=%u radio=%u\n",
                trajectory_mode, radio);

    // ---- 读当前值作为起点 ----
    std::vector<float> left_start(n_joints, 0.0F);
    std::vector<float> right_start(n_joints, 0.0F);
    bool present_l = false, present_r = false;

    if (!check(tms_cuarm_2_2_session_joints(s, left_dev.c_str(),
                                            left_start.data(), left_start.size(),
                                            &present_l),
               "read left start")) return false;
    if (!check(tms_cuarm_2_2_session_joints(s, right_dev.c_str(),
                                            right_start.data(), right_start.size(),
                                            &present_r),
               "read right start")) return false;
    if (!present_l || !present_r) {
        std::printf("  [FAIL] missing state for one arm\n");
        return false;
    }

    std::printf("  left  start : ");
    for (float v : left_start ) std::printf("%+.4f ", v);
    std::printf("\n");
    std::printf("  right start : ");
    for (float v : right_start) std::printf("%+.4f ", v);
    std::printf("\n");

    // ---- 逐周期 stage ----
    std::vector<float> left_cmd  = left_start;
    std::vector<float> right_cmd = right_start;

    int  stage_fail_l    = 0;
    int  stage_fail_r    = 0;
    int  periods_overrun = 0;

    auto next_tick = std::chrono::steady_clock::now();
    const auto t0  = next_tick;

    for (int i = 1; i <= n_steps; ++i)
    {
        next_tick += std::chrono::milliseconds(kPeriodMs);

        const float offset = step_deg * static_cast<float>(i);
        for (int j = 0; j < n_joints; ++j) {
            left_cmd [j] = left_start [j] + offset;
            right_cmd[j] = right_start[j] + offset;
        }

        int32_t staged_l = 0, staged_r = 0;
        const int rc_l = tms_cuarm_2_2_joint_cmd(
            s, left_dev.c_str(),
            left_cmd.data(), left_cmd.size(),
            /*follow=*/true, trajectory_mode, radio,
            &staged_l);
        const int rc_r = tms_cuarm_2_2_joint_cmd(
            s, right_dev.c_str(),
            right_cmd.data(), right_cmd.size(),
            /*follow=*/true, trajectory_mode, radio,
            &staged_r);

        if (rc_l != TMS_OK || staged_l != 0) ++stage_fail_l;
        if (rc_r != TMS_OK || staged_r != 0) ++stage_fail_r;

        if (i % 10 == 0 || i == 1 || i == n_steps) {
            std::printf("  step %3d/%d  offset=%+.4f deg  "
                        "stage_l=%d stage_r=%d\n",
                        i, n_steps, offset, staged_l, staged_r);
        }

        // 100 Hz 节奏（sleep_until 比 sleep_for 精度高）
        const auto now = std::chrono::steady_clock::now();
        if (now > next_tick) {
            ++periods_overrun;
        } else {
            std::this_thread::sleep_until(next_tick);
        }
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::printf("  staged %d steps in %.1f ms (target %.0f ms), "
                "overrun=%d\n",
                n_steps, elapsed_ms,
                static_cast<double>(n_steps * kPeriodMs),
                periods_overrun);
    std::printf("  stage failures: left=%d right=%d\n",
                stage_fail_l, stage_fail_r);

    // ---- 等 agent 到位 ----
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- 读终值 ----
    std::vector<float> left_end(n_joints, 0.0F);
    std::vector<float> right_end(n_joints, 0.0F);
    bool end_l = false, end_r = false;

    tms_cuarm_2_2_session_joints(s, left_dev.c_str(),
                                 left_end.data(), left_end.size(), &end_l);
    tms_cuarm_2_2_session_joints(s, right_dev.c_str(),
                                 right_end.data(), right_end.size(), &end_r);

    std::printf("  left  end   : ");
    for (float v : left_end ) std::printf("%+.4f ", v);
    std::printf("\n");
    std::printf("  right end   : ");
    for (float v : right_end) std::printf("%+.4f ", v);
    std::printf("\n");

    // ---- 报告实际偏移（度）----
    auto report_delta = [&](const char* label,
                            const std::vector<float>& start,
                            const std::vector<float>& end,
                            bool ok) {
        if (!ok) return;
        float min_d = 1e9F, max_d = -1e9F;
        for (int j = 0; j < n_joints; ++j) {
            const float d = end[j] - start[j];
            if (d < min_d) min_d = d;
            if (d > max_d) max_d = d;
        }
        std::printf("  %s delta  : min=%+.4f max=%+.4f deg (target +%.4f)\n",
                    label, min_d, max_d, target_deg);
    };
    report_delta("left ", left_start,  left_end,  end_l);
    report_delta("right", right_start, right_end, end_r);

    if (stage_fail_l > 0 || stage_fail_r > 0) {
        std::printf("  [WARN] some stages were rejected by the agent\n");
    }

    std::printf("  [OK] move completed\n");
    return true;
}

// ------------------------------------------------------------
// [8] 微动
// ------------------------------------------------------------
bool stage8_small_move(TmsSession* s, const std::string& dev)
{
    print_sep("[8] small move (+0.01 rad on J1)");

    const int n = static_cast<int>(tms_cuarm_2_2_joint_count());
    std::vector<float> before(n, 0.0F), after(n, 0.0F), target(n, 0.0F);
    bool present = false;

    if (!check(tms_cuarm_2_2_session_joints(s, dev.c_str(),
                                            before.data(), before.size(), &present),
               "read before")) return false;
    if (!present) {
        std::printf("  [FAIL] no state before\n");
        return false;
    }

    target = before;
    target[0] += 0.01F;   // ~0.57°，< max_start_deg=5° 且 < max_step_deg=10°

    std::printf("  target[0] = %.4f rad (was %.4f, delta +0.01)\n",
                target[0], before[0]);

    int32_t staged = 0;
    if (!check(tms_cuarm_2_2_joint_cmd(s, dev.c_str(),
                                       target.data(), target.size(),
                                       /*follow=*/false,
                                       /*trajectory_mode=*/0,
                                       /*radio=*/0,
                                       &staged),
               "stage small move")) return false;
    std::printf("  staged=%d\n", staged);
    if (staged != 0) {
        std::printf("  [FAIL] locally rejected\n");
        return false;
    }

    // 等 agent 执行（一帧点命令，agent 端有五次多项式插值）
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    if (!check(tms_cuarm_2_2_session_joints(s, dev.c_str(),
                                            after.data(), after.size(), &present),
               "read after")) return false;
    if (!present) {
        std::printf("  [FAIL] no state after\n");
        return false;
    }

    std::printf("  after[0] = %.4f rad (delta = %+.4f)\n",
                after[0], after[0] - before[0]);

    const float delta_seen = after[0] - before[0];
    if (delta_seen < 0.005F) {
        std::printf("  [FAIL] robot did not move (delta_seen=%.4f < 0.005)\n",
                    delta_seen);
        return false;
    }

    std::printf("  [OK] small move verified\n");
    return true;
}

// ------------------------------------------------------------
// [10] close + destroy
// ------------------------------------------------------------
void stage10_cleanup(TmsSession** s)
{
    print_sep("[10] cleanup");

    if (*s) {
        check(tms_session_close(*s), "tms_session_close");
        tms_session_destroy(*s);
        *s = nullptr;
        std::printf("  tms_session_destroy done\n");
    }
}

}  // namespace

// ============================================================
// main
// ============================================================
int main(int argc, char** argv)
{
    TestConfig cfg;
    TmsSession* session = nullptr;
    int exit_code = 0;

    stage0_library_info();

    if (!stage1_parse_args(argc, argv, cfg)) {
        return 2;
    }

    if (!stage2_create_session(cfg, &session)) {
        return 3;
    }

    // 注册回调（可选，仅用于观察）
    tms_session_on_state(session, on_state, nullptr);
    tms_session_on_status(session, on_status, nullptr);

    if (!stage3_agent_online(session)) {
        exit_code = 4;
        goto cleanup;
    }

    if (!stage4_query_config(session)) {
        exit_code = 5;
        goto cleanup;
    }

    if (!stage5_group_enable(session, cfg.test_device, true)) {
        exit_code = 6;
        goto cleanup;
    }
    
    if (!stage6_wait_state(session, cfg.test_device, 3000)) {
        exit_code = 7;
        goto cleanup;
    }

    if (!stage7_hold(session, cfg.test_device)) {
        exit_code = 8;
        goto cleanup;
    }

    if (cfg.test_motion) {
        if (!stage8_move_both_10deg(session,
                                    "left",  "right",
                                    /*duration_ms=*/2000,
                                    /*target_deg=*/10.0F,
                                    /*trajectory_mode=*/2,
                                    /*radio=*/500))
        {
            exit_code = 9;
            goto cleanup;
        }
    } else {
        print_sep("[8] both-arm move (skipped, use --test-motion to enable)");
    }

    // 打印回调统计
    print_sep("callback statistics");
    std::printf("  state_cb_count  = %lu\n",
                static_cast<unsigned long>(g_state.state_cb_count.load()));
    std::printf("  status_cb_count = %lu\n",
                static_cast<unsigned long>(g_state.status_cb_count.load()));
    std::printf("  last_state_tick = %lu\n",
                static_cast<unsigned long>(g_state.last_state_tick.load()));

cleanup:
    // 无论成功失败，都尝试 disable + cleanup
    if (session) {
        stage5_group_enable(session, cfg.test_device, false);
    }
    stage10_cleanup(&session);

    print_sep(exit_code == 0 ? "ALL PASSED" : "FAILED");
    return exit_code;
}