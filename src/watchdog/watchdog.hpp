#pragma once

/**
 * watchdog.hpp  —  线程级 Watchdog（第四阶段主角）
 *
 * 设计：
 *   每个受监控线程定期调用 feed("thread_name")
 *   Watchdog 线程检测超时 → 触发回调（重启 / 报警 / 进入 ERROR 态）
 *
 * 面试考点：
 *   Q: Watchdog 怎么检测线程死锁？
 *   A: 死锁的线程无法执行到 feed()，超时后 Watchdog 触发告警，
 *      可以通过 pthread_cancel 或 raise(SIGABRT) 让进程重启。
 */

#include <string>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

using TimeoutCallback = std::function<void(const std::string& thread_name)>;

class Watchdog {
public:
    explicit Watchdog(int timeout_ms);
    ~Watchdog();

    // 注册一个被监控的线程
    void register_thread(const std::string& name);

    // 被监控线程定期调用（喂狗）
    void feed(const std::string& name);

    // 设置超时回调
    void set_timeout_callback(TimeoutCallback cb);

    void start();
    void stop();

private:
    void watch_loop();

    int                     timeout_ms_;
    std::atomic<bool>       running_{false};
    std::thread             watch_thread_;
    TimeoutCallback         timeout_cb_;

    struct WatchEntry {
        std::chrono::steady_clock::time_point last_feed;
        bool alive{true};
    };

    std::mutex                               mutex_;
    std::unordered_map<std::string, WatchEntry> entries_;
};
