#include "watchdog.hpp"
#include "../logger/logger.hpp"

#include <chrono>

using namespace std::chrono;

Watchdog::Watchdog(int timeout_ms)
    : timeout_ms_(timeout_ms)
{}

Watchdog::~Watchdog() {
    stop();
}

void Watchdog::register_thread(const std::string& name) {
    std::lock_guard<std::mutex> lk(mutex_);
    entries_[name] = { steady_clock::now(), true };
    LOG_INFO("watchdog", "Registered thread: " + name);
}

void Watchdog::feed(const std::string& name) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = entries_.find(name);
    if (it != entries_.end()) {
        it->second.last_feed = steady_clock::now();
        it->second.alive     = true;
    }
}

void Watchdog::set_timeout_callback(TimeoutCallback cb) {
    timeout_cb_ = std::move(cb);
}

void Watchdog::start() {
    running_      = true;
    watch_thread_ = std::thread(&Watchdog::watch_loop, this);
    LOG_INFO("watchdog", "Watchdog started, timeout=" + std::to_string(timeout_ms_) + "ms");
}

void Watchdog::stop() {
    if (!running_) return;
    running_ = false;
    if (watch_thread_.joinable()) watch_thread_.join();
}

void Watchdog::watch_loop() {
    while (running_) {
        std::this_thread::sleep_for(milliseconds(timeout_ms_ / 2));

        std::lock_guard<std::mutex> lk(mutex_);
        auto now = steady_clock::now();

        for (auto& [name, entry] : entries_) {
            auto elapsed = duration_cast<milliseconds>(now - entry.last_feed).count();
            if (elapsed > timeout_ms_ && entry.alive) {
                entry.alive = false;
                LOG_ERROR("watchdog", "Thread timeout: " + name +
                          " (elapsed=" + std::to_string(elapsed) + "ms)");
                if (timeout_cb_) timeout_cb_(name);
            }
        }
    }
}
