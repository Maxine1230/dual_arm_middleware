#include "common_types.hpp"
#include "logger/logger.hpp"
#include "utils/ring_buffer.hpp"
#include "utils/thread_pool.hpp"
#include "utils/state_machine.hpp"
#include "comm/serial_port.hpp"
#include "config/config_loader.hpp"
#include "state/arm_state_hub.hpp"
#include "trajectory/trajectory_generator.hpp"
#include "watchdog/watchdog.hpp"

#include <csignal>
#include <atomic>
#include <array>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>

namespace {

/** 与 STATUS_REPORT 新鲜度相关的健康状态（用于边沿触发日志） */
enum class TelHealth { Ok, Stale, NoRx };

struct TelemetryFault {
    bool      active{false};
    bool      no_rx{false};
    long long age_ms{0};
};

/** 主线程在串口 open 成功后写入：从该时刻起若仍无首帧则判掉线 */
struct TelemetryExpectCtx {
    std::mutex                                          mtx;
    std::array<bool, 2>                                 expect{false, false};
    std::array<std::chrono::steady_clock::time_point, 2> since{};
};

TelemetryFault check_telemetry_stale(ArmID arm,
                                     ArmStateHub& hub,
                                     SerialPort& port,
                                     bool expect_serial,
                                     std::chrono::steady_clock::time_point expect_since,
                                     int stale_ms,
                                     const char* arm_name,
                                     TelHealth& prev) {
    TelemetryFault fault{};
    if (stale_ms <= 0) {
        return fault;
    }
    if (!expect_serial || !port.is_open()) {
        return fault;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!hub.has_valid(arm)) {
        const auto age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - expect_since).count();
        fault.active = age_ms > stale_ms;
        fault.no_rx = true;
        fault.age_ms = age_ms;
        if (age_ms > stale_ms) {
            if (prev != TelHealth::NoRx) {
                LOG_WARN("monitor",
                         std::string("Telemetry OFFLINE (arm ") + arm_name
                             + "): no STATUS_REPORT " + std::to_string(age_ms)
                             + " ms after serial ready (threshold " + std::to_string(stale_ms)
                             + " ms)");
                prev = TelHealth::NoRx;
            }
        }
        return fault;
    }

    std::chrono::steady_clock::time_point last_rx{};
    if (!hub.last_rx_time(arm, last_rx)) {
        return fault;
    }

    const auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx).count();
    fault.active = age_ms > stale_ms;
    fault.no_rx = false;
    fault.age_ms = age_ms;
    if (age_ms > stale_ms) {
        if (prev != TelHealth::Stale) {
            LOG_WARN("monitor",
                     std::string("Telemetry STALE (arm ") + arm_name + "): last STATUS_REPORT "
                         + std::to_string(age_ms) + " ms ago (threshold " + std::to_string(stale_ms)
                         + " ms)");
            prev = TelHealth::Stale;
        }
    } else if (prev != TelHealth::Ok) {
        LOG_INFO("monitor",
                 std::string("Telemetry OK (arm ") + arm_name + "): STATUS_REPORT fresh again");
        prev = TelHealth::Ok;
    }
    return fault;
}

std::string fmt_float_array(const float* arr, int n) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6) << "[";
    for (int i = 0; i < n; ++i) {
        if (i) {
            os << ", ";
        }
        os << arr[i];
    }
    os << "]";
    return os.str();
}

/** 非 STATUS_REPORT 或长度不符时，打印帧头 + 前若干字节十六进制，便于对照线上数据 */
std::string fmt_payload_hex_preview(const RawFrame& f, size_t max_bytes = 32) {
    const size_t n = std::min(static_cast<size_t>(f.length), max_bytes);
    std::ostringstream os;
    os << std::hex << std::setfill('0');
    for (size_t i = 0; i < n; ++i) {
        if (i) {
            os << ' ';
        }
        os << std::setw(2) << static_cast<unsigned>(f.payload[i]);
    }
    if (f.length > max_bytes) {
        os << " ...";
    }
    return os.str();
}

/**
 * 串口 RX：先写入 ArmStateHub（STATUS_REPORT → 全局关节字典），再按端口打详细日志。
 */
void ingest_and_log_serial_rx(const char* port_tag, const RawFrame& f) {
    ArmStateHub::instance().ingest_frame(f);

    const char* type_name = "UNKNOWN";
    switch (f.type) {
    case FrameType::HEARTBEAT:     type_name = "HEARTBEAT";     break;
    case FrameType::JOINT_CMD:     type_name = "JOINT_CMD";     break;
    case FrameType::STATUS_REPORT: type_name = "STATUS_REPORT"; break;
    case FrameType::ACK:           type_name = "ACK";           break;
    case FrameType::ERROR_REPORT:  type_name = "ERROR_REPORT";  break;
    default: break;
    }

    if (f.type == FrameType::STATUS_REPORT
        && f.length == sizeof(StatusReportPayload)) {
        StatusReportPayload s{};
        std::memcpy(&s, f.payload, sizeof(s));
        const char* arm = (s.arm_id == 0) ? "LEFT" : "RIGHT";
        std::ostringstream msg;
        msg << "[" << port_tag << " RX] " << type_name
            << " | seq=" << static_cast<int>(f.seq)
            << " len=" << f.length
            << " crc=0x" << std::hex << std::setw(4) << std::setfill('0') << f.crc16
            << std::dec << std::setfill(' ')
            << "\n  arm=" << arm
            << " error_code=" << static_cast<int>(s.error_code)
            << " timestamp_ms=" << s.timestamp_ms
            << "\n  joint_pos[rad]   " << fmt_float_array(s.joint_pos, DOF)
            << "\n  joint_vel[rad/s] " << fmt_float_array(s.joint_vel, DOF);
        LOG_INFO("comm", msg.str());
        return;
    }

    std::ostringstream other;
    other << "[" << port_tag << " RX] " << type_name
          << " | seq=" << static_cast<int>(f.seq)
          << " len=" << f.length
          << " crc=0x" << std::hex << std::setw(4) << std::setfill('0') << f.crc16
          << std::dec << std::setfill(' ')
          << "\n  payload_hex: " << fmt_payload_hex_preview(f);
    LOG_INFO("comm", other.str());
}

}  // namespace

// ─── 全局停止信号 ────────────────────────────────────────
std::atomic<bool> g_shutdown{false};

void signal_handler(int sig) {
    std::cout << "\n[SIGNAL] Caught signal " << sig << ", shutting down...\n";
    g_shutdown = true;
}

// ─── 轨迹点环形缓冲区（轨迹线程 → 通信线程）────────────────
RingBuffer<JointPoint, 256> g_traj_buffer;

int main(int argc, char** argv) {
    // ── 信号处理 ──────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── 配置：优先加载项目内 config/system.json，失败则回退默认 ──
    SystemConfig cfg{};
    system_config_set_defaults(cfg);
    std::string    cfg_err;
    const auto     cfg_path = resolve_system_config_path(argc, argv);
    if (cfg_path) {
        if (!load_system_config_json(*cfg_path, cfg, &cfg_err)) {
            std::cerr << "[config] load failed: " << cfg_err << " — using built-in defaults\n";
        }
    } else {
        std::cerr << "[config] config/system.json not found (try cwd, ../config, or MIDDLEWARE_CONFIG) — using defaults\n";
    }

    // ── Logger（必须在 JSON 解析之后，才能使用 log_dir / log_level）────
    Logger::instance().init(cfg.log_dir, log_level_from_int(cfg.log_level));
    LOG_INFO("main", "=== Dual Arm Middleware starting ===");
    if (cfg_path) {
        LOG_INFO("main", "Loaded config: " + cfg_path->string());
    } else {
        LOG_INFO("main", "Using built-in defaults (no JSON file resolved)");
    }

    // ── 状态机（须先于 ArmStateHub 监听器，便于 STATUS_REPORT 驱动 ERROR）──────
    StateMachine fsm;
    fsm.on_state_change([](SystemState from, SystemState to) {
        // StateMachine 自己不依赖 Logger，用 cout 打印
        // 第四阶段改为 LOG_INFO
        std::cout << "[FSM] " << (int)from << " -> " << (int)to << "\n";
    });

    // ── ArmStatus 业务回调：下位机 error_code≠0 时切入 FSM::ERROR ──────────────
    ArmStateHub::instance().set_listener([&fsm](ArmID id, const ArmStatus& st) {
        LOG_DEBUG("arm_state",
                  std::string("ArmStateHub updated arm=")
                      + (id == ArmID::LEFT ? "LEFT" : "RIGHT")
                      + " err=" + std::to_string(static_cast<int>(st.error_code))
                      + " t_ms=" + std::to_string(st.timestamp_ms)
                      + " q0=" + std::to_string(st.joint_pos[0])
                      + " dq0=" + std::to_string(st.joint_vel[0]));

        if (st.error_code == 0) {
            return;
        }
        const SystemState cur = fsm.current();
        if (cur != SystemState::IDLE && cur != SystemState::RUNNING && cur != SystemState::PAUSED) {
            return;
        }
        if (fsm.transition(SystemState::ERROR)) {
            LOG_WARN("main",
                     std::string("FSM→ERROR: STATUS_REPORT arm=")
                         + (id == ArmID::LEFT ? "LEFT" : "RIGHT")
                         + " error_code=" + std::to_string(static_cast<int>(st.error_code)));
        }
    });

    // ── Watchdog ──────────────────────────────────────────
    Watchdog watchdog(cfg.watchdog_timeout_ms);
    watchdog.set_timeout_callback([&fsm](const std::string& name) {
        LOG_ERROR("main", "Watchdog triggered by: " + name);
        fsm.transition(SystemState::ERROR);
    });

    // ── 轨迹生成器（五次多项式 → 无锁 RingBuffer）────────────────
    TrajectoryGenerator<256> traj_gen(cfg.trajectory, g_traj_buffer);

    // ── 串口通信 ──────────────────────────────────────────
    SerialPort serial_left(cfg.serial_left);
    SerialPort serial_right(cfg.serial_right);

    serial_left.set_frame_callback([](const RawFrame& f) {
        ingest_and_log_serial_rx("serial_left", f);
    });
    serial_right.set_frame_callback([](const RawFrame& f) {
        ingest_and_log_serial_rx("serial_right", f);
    });

    // ── 通信线程：按 control_period_ms 从 RingBuffer 取 JointPoint → JOINT_CMD → 对应串口 ──
    std::thread comm_thread([&watchdog, &cfg, &serial_left, &serial_right]() {
        watchdog.register_thread("comm");
        LOG_INFO("comm", "Communication thread started");

        const auto period = std::chrono::duration<double, std::milli>(cfg.trajectory.control_period_ms);

        while (!g_shutdown) {
            watchdog.feed("comm");

            auto opt = g_traj_buffer.pop();
            if (opt.has_value()) {
                const JointPoint& pt = *opt;
                SerialPort* port = (pt.arm_id == ArmID::LEFT) ? &serial_left : &serial_right;
                if (!port->is_open()) {
                    LOG_WARN("comm", "skip send: serial not open for arm_id="
                                        + std::to_string(static_cast<int>(pt.arm_id)));
                } else if (!port->send_joint_command(pt)) {
                    LOG_WARN("comm", "send_joint_command failed (arm_id="
                                        + std::to_string(static_cast<int>(pt.arm_id)) + ")");
                } else {
                    LOG_DEBUG("comm",
                              "TX JOINT_CMD arm=" + std::to_string(static_cast<int>(pt.arm_id))
                                  + " t_ms=" + std::to_string(pt.timestamp_ms));
                }
            }

            std::this_thread::sleep_for(period);
        }

        LOG_INFO("comm", "Communication thread exiting");
    });

    // ── 串口 open 后写入 expect_since，供掉线（首帧超时）判定 ─────────────
    TelemetryExpectCtx tel_expect{};

    // ── 状态监控线程 ──────────────────────────────────────
    std::thread monitor_thread([&fsm, &watchdog, &cfg, &serial_left, &serial_right, &tel_expect]() {
        watchdog.register_thread("monitor");
        LOG_INFO("monitor", "Status monitor thread started");
        const int telemetry_fault_to_error_ms =
            std::max(cfg.telemetry_stale_ms, cfg.watchdog_timeout_ms);
        LOG_INFO("monitor",
                 "Telemetry fault escalation threshold(ms): "
                     + std::to_string(telemetry_fault_to_error_ms)
                     + " (max(telemetry_stale_ms, watchdog_timeout_ms))");

        TelHealth tel_prev_left{TelHealth::Ok};
        TelHealth tel_prev_right{TelHealth::Ok};
        bool      escalated_left{false};
        bool      escalated_right{false};

        while (!g_shutdown) {
            watchdog.feed("monitor");

            LOG_DEBUG("monitor", "System state=" + fsm.state_name(fsm.current()));

            bool       ex_l = false;
            bool       ex_r = false;
            std::chrono::steady_clock::time_point since_l{};
            std::chrono::steady_clock::time_point since_r{};
            {
                std::lock_guard<std::mutex> lk(tel_expect.mtx);
                ex_l    = tel_expect.expect[0];
                ex_r    = tel_expect.expect[1];
                since_l = tel_expect.since[0];
                since_r = tel_expect.since[1];
            }

            auto& hub = ArmStateHub::instance();
            const TelemetryFault left_fault =
                check_telemetry_stale(ArmID::LEFT, hub, serial_left, ex_l, since_l,
                                      cfg.telemetry_stale_ms, "LEFT", tel_prev_left);
            const TelemetryFault right_fault =
                check_telemetry_stale(ArmID::RIGHT, hub, serial_right, ex_r, since_r,
                                      cfg.telemetry_stale_ms, "RIGHT", tel_prev_right);

            auto escalate_fault = [&fsm, telemetry_fault_to_error_ms](const char* arm_name,
                                                                       const TelemetryFault& fault,
                                                                       bool& escalated) {
                if (!fault.active || telemetry_fault_to_error_ms <= 0) {
                    escalated = false;
                    return;
                }

                if (fault.age_ms < telemetry_fault_to_error_ms) {
                    return;
                }
                if (escalated) {
                    return;
                }

                const SystemState cur = fsm.current();
                if (cur == SystemState::ERROR) {
                    escalated = true;
                    return;
                }
                if (cur != SystemState::IDLE && cur != SystemState::RUNNING
                    && cur != SystemState::PAUSED) {
                    return;
                }

                if (fsm.transition(SystemState::ERROR)) {
                    LOG_ERROR("main",
                              std::string("FSM→ERROR: telemetry ")
                                  + (fault.no_rx ? "OFFLINE" : "STALE")
                                  + " arm=" + arm_name
                                  + " age_ms=" + std::to_string(fault.age_ms)
                                  + " escalate_ms="
                                  + std::to_string(telemetry_fault_to_error_ms));
                    escalated = true;
                }
            };

            escalate_fault("LEFT", left_fault, escalated_left);
            escalate_fault("RIGHT", right_fault, escalated_right);

            ArmStatus st{};
            if (hub.try_snapshot(ArmID::LEFT, st)) {
                LOG_DEBUG("monitor",
                          "LEFT arm pose[rad] " + fmt_float_array(st.joint_pos, DOF)
                              + " err=" + std::to_string(static_cast<int>(st.error_code)));
            }
            if (hub.try_snapshot(ArmID::RIGHT, st)) {
                LOG_DEBUG("monitor",
                          "RIGHT arm pose[rad] " + fmt_float_array(st.joint_pos, DOF)
                              + " err=" + std::to_string(static_cast<int>(st.error_code)));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        LOG_INFO("monitor", "Status monitor thread exiting");
    });

    // ── 启动所有模块 ──────────────────────────────────────
    watchdog.start();

    const bool serial_left_ready  = serial_left.open();
    const bool serial_right_ready = serial_right.open();
    {
        std::lock_guard<std::mutex> lk(tel_expect.mtx);
        if (serial_left_ready) {
            tel_expect.expect[0] = true;
            tel_expect.since[0]  = std::chrono::steady_clock::now();
        }
        if (serial_right_ready) {
            tel_expect.expect[1] = true;
            tel_expect.since[1]  = std::chrono::steady_clock::now();
        }
    }
    if (serial_left_ready) {
        serial_left.start();
    } else {
        LOG_WARN("main", "serial_left open failed (check " + cfg.serial_left.port + "), skip start");
    }
    if (serial_right_ready) {
        serial_right.start();
    } else {
        LOG_WARN("main", "serial_right open failed (check " + cfg.serial_right.port + "), skip start");
    }

    traj_gen.start();

    fsm.transition(SystemState::IDLE);
    fsm.transition(SystemState::RUNNING);

    LOG_INFO("main", "All threads running. Press Ctrl+C to stop.");

    // ── 测试用：极端运动边界（验证 quintic → RingBuffer → comm → JOINT_CMD）────────
    // 左臂 6 关节：起止位 ±π rad，起止速度/加速度取配置上限的 2 倍，段时长极短 → 高动态应力
    {
        constexpr float kPi = 3.14159265f;
        const float     vmax = static_cast<float>(2.0 * cfg.trajectory.max_velocity);
        const float     amax = static_cast<float>(2.0 * cfg.trajectory.max_acceleration);

        JointPoint test_start{};
        test_start.arm_id = ArmID::LEFT;
        for (int i = 0; i < DOF; ++i) {
            test_start.position[i]      = -kPi;
            test_start.velocity[i]      = -vmax;
            test_start.acceleration[i]  = -amax;
        }
        test_start.timestamp_ms = 0.f;

        JointPoint test_end{};
        test_end.arm_id = ArmID::LEFT;
        for (int i = 0; i < DOF; ++i) {
            test_end.position[i]      = kPi;
            test_end.velocity[i]      = vmax;
            test_end.acceleration[i]  = amax;
        }
        test_end.timestamp_ms = 0.f;

        constexpr double kTestDurationS = 0.05;  // 50 ms 内扫完全程，极陡

        if (traj_gen.plan(test_start, test_end, kTestDurationS)) {
            LOG_INFO("main",
                     "TEST trajectory armed: LEFT ±π rad / 50ms, |v|=2×cfg |a|=2×cfg (stress)");
        } else {
            LOG_WARN("main", "TEST trajectory plan() failed");
        }
    }

    // ── 主循环 ────────────────────────────────────────────
    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ── 优雅退出 ──────────────────────────────────────────
    LOG_INFO("main", "Shutdown sequence started...");
    fsm.transition(SystemState::SHUTDOWN);

    traj_gen.stop();
    serial_left.stop();
    serial_right.stop();
    watchdog.stop();

    if (comm_thread.joinable())    comm_thread.join();
    if (monitor_thread.joinable()) monitor_thread.join();

    LOG_INFO("main", "=== Middleware stopped cleanly ===");
    Logger::instance().stop();

    return 0;
}
