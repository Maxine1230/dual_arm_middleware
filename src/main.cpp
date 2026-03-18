#include "common_types.hpp"
#include "logger/logger.hpp"
#include "utils/ring_buffer.hpp"
#include "utils/thread_pool.hpp"
#include "utils/state_machine.hpp"
#include "comm/serial_port.hpp"
#include "trajectory/trajectory_generator.hpp"
#include "watchdog/watchdog.hpp"

#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

// ─── 全局停止信号 ────────────────────────────────────────
std::atomic<bool> g_shutdown{false};

void signal_handler(int sig) {
    std::cout << "\n[SIGNAL] Caught signal " << sig << ", shutting down...\n";
    g_shutdown = true;
}

// ─── 轨迹点环形缓冲区（轨迹线程 → 通信线程）────────────────
RingBuffer<JointPoint, 256> g_traj_buffer;

int main() {
    // ── 信号处理 ──────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── Logger ────────────────────────────────────────────
    Logger::instance().init("/tmp/middleware_log", LogLevel::DEBUG);
    LOG_INFO("main", "=== Dual Arm Middleware starting ===");

    // ── 配置（TODO 第四阶段：从 JSON 读取）────────────────────
    SystemConfig cfg;
    cfg.serial_left  = { "/dev/ttyS3",  115200, 100 };
    cfg.serial_right = { "/dev/ttyS4",  115200, 100 };
    cfg.trajectory   = { 1.0, 2.0, 256, 10.0 };
    cfg.log_dir      = "/tmp/middleware_log";
    cfg.log_level    = 0;
    cfg.watchdog_timeout_ms = 2000;

    // ── 状态机 ────────────────────────────────────────────
    StateMachine fsm;
    fsm.on_state_change([](SystemState from, SystemState to) {
        // StateMachine 自己不依赖 Logger，用 cout 打印
        // 第四阶段改为 LOG_INFO
        std::cout << "[FSM] " << (int)from << " -> " << (int)to << "\n";
    });

    // ── Watchdog ──────────────────────────────────────────
    Watchdog watchdog(cfg.watchdog_timeout_ms);
    watchdog.set_timeout_callback([&fsm](const std::string& name) {
        LOG_ERROR("main", "Watchdog triggered by: " + name);
        fsm.transition(SystemState::ERROR);
    });

    // ── 轨迹生成器 ────────────────────────────────────────
    TrajectoryGenerator traj_gen(cfg.trajectory);
    traj_gen.set_point_callback([](const JointPoint& pt) {
        // 生产者：压入环形缓冲区
        if (!g_traj_buffer.push(pt)) {
            LOG_WARN("traj", "Ring buffer full, dropping point");
        }
    });

    // ── 串口通信 ──────────────────────────────────────────
    SerialPort serial_left(cfg.serial_left);
    SerialPort serial_right(cfg.serial_right);

    serial_left.set_frame_callback([](const RawFrame& f) {
        LOG_DEBUG("comm", "RX frame type=" + std::to_string((int)f.type));
    });

    // ── 通信线程：消费环形缓冲区 → 发送给下位机 ───────────────
    std::thread comm_thread([&serial_left, &watchdog]() {
        watchdog.register_thread("comm");
        LOG_INFO("comm", "Communication thread started");

        while (!g_shutdown) {
            watchdog.feed("comm");

            // 消费者：从环形缓冲区取轨迹点
            auto pt = g_traj_buffer.pop();
            if (pt.has_value()) {
                // TODO (第二阶段): 构建帧 → serial_left.send_frame()
                LOG_DEBUG("comm", "Consumed trajectory point");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        LOG_INFO("comm", "Communication thread exiting");
    });

    // ── 状态监控线程 ──────────────────────────────────────
    std::thread monitor_thread([&fsm, &watchdog]() {
        watchdog.register_thread("monitor");
        LOG_INFO("monitor", "Status monitor thread started");

        while (!g_shutdown) {
            watchdog.feed("monitor");

            // TODO (第三阶段+): 读取 ArmStatus，检查异常
            LOG_DEBUG("monitor", "System state=" + fsm.state_name(fsm.current()));

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        LOG_INFO("monitor", "Status monitor thread exiting");
    });

    // ── 启动所有模块 ──────────────────────────────────────
    watchdog.start();
    // serial_left.start();   // TODO (第二阶段): 串口 ready 后解注释
    // serial_right.start();
    traj_gen.start();

    fsm.transition(SystemState::IDLE);
    fsm.transition(SystemState::RUNNING);

    LOG_INFO("main", "All threads running. Press Ctrl+C to stop.");

    // ── 主循环 ────────────────────────────────────────────
    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ── 优雅退出 ──────────────────────────────────────────
    LOG_INFO("main", "Shutdown sequence started...");
    fsm.transition(SystemState::SHUTDOWN);

    traj_gen.stop();
    // serial_left.stop();
    // serial_right.stop();
    watchdog.stop();

    if (comm_thread.joinable())    comm_thread.join();
    if (monitor_thread.joinable()) monitor_thread.join();

    LOG_INFO("main", "=== Middleware stopped cleanly ===");
    Logger::instance().stop();

    return 0;
}
