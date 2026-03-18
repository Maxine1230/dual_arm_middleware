#pragma once

/**
 * thread_pool.hpp  —  通用线程池
 *
 * 本项目的用途：
 *   - 轨迹计算任务（CPU密集）
 *   - 日志写入任务（IO密集，但 Logger 自带线程）
 *
 * 注意：控制线程（comm/trajectory/watchdog）不走线程池，
 * 各自独立 std::thread，保证实时性可控。
 */

#include <vector>
#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <stdexcept>

class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads);
    ~ThreadPool();

    // 提交任务，返回 future 以获取结果
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args)
        -> std::future<std::invoke_result_t<F, Args...>>;

    size_t thread_count() const { return workers_.size(); }

private:
    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> task_queue_;
    std::mutex                        queue_mutex_;
    std::condition_variable           cv_;
    std::atomic<bool>                 stop_{false};
};

// ─── Template implementation ────────────────────────────

inline ThreadPool::ThreadPool(size_t num_threads) {
    for (size_t i = 0; i < num_threads; ++i) {
        workers_.emplace_back([this] {
            for (;;) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lk(queue_mutex_);
                    cv_.wait(lk, [this]{ return stop_ || !task_queue_.empty(); });
                    if (stop_ && task_queue_.empty()) return;
                    task = std::move(task_queue_.front());
                    task_queue_.pop();
                }
                task();
            }
        });
    }
}

inline ThreadPool::~ThreadPool() {
    stop_ = true;
    cv_.notify_all();
    for (auto& w : workers_) {
        if (w.joinable()) w.join();
    }
}

template<typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args)
    -> std::future<std::invoke_result_t<F, Args...>>
{
    using ReturnType = std::invoke_result_t<F, Args...>;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    auto future = task->get_future();
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (stop_) throw std::runtime_error("ThreadPool is stopped");
        task_queue_.emplace([task]{ (*task)(); });
    }
    cv_.notify_one();
    return future;
}
