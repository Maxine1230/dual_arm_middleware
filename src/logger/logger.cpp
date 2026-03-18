/**
 * logger.cpp  -- 异步日志系统实现
 *
 * 数据流：
 *   业务线程 LOG_* → log() → 入队 queue_ → notify_one
 *   worker_loop 被唤醒 → 出队 → write_entry() → stdout + 文件
 */

#include "logger.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ctime>

namespace fs = std::filesystem;
// ─────────────────────────────────────────────────────────────────────────────
// 单例实现（Meyers' Singleton）
// ─────────────────────────────────────────────────────────────────────────────
// C++11 保证：函数内 static 变量的初始化是线程安全的，且只执行一次
Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

// 析构时确保 worker 线程已停止，避免悬空引用
Logger::~Logger() {
    stop();
}

// ─────────────────────────────────────────────────────────────────────────────
// 初始化与停止
// ─────────────────────────────────────────────────────────────────────────────
//
// init() 流程：
//   1) 保存 log_dir、min_level
//   2) 创建日志目录（若不存在）
//   3) 设置 running_ = true
//   4) 启动 worker 线程，执行 worker_loop()
//
void Logger::init(const std::string& log_dir, LogLevel min_level) {
    if (running_ || worker_.joinable()) {
        return; // 防御性编程：防止重复初始化导致 crash
    }

    log_dir_   = log_dir;
    min_level_ = min_level;

    fs::create_directories(log_dir_);

    if (!log_dir_.empty()) {
        ofs_.open(log_dir_ + "/middleware.log", std::ios::app);
    }

    running_ = true;
    worker_  = std::thread(&Logger::worker_loop, this);

    info("logger", "Logger initialized. dir=" + log_dir_);
}

//
// stop() 流程：
//   1) 若已停止则直接返回
//   2) running_ = false，通知 worker 准备退出
//   3) cv_.notify_all() 唤醒可能阻塞在 cv_.wait 的 worker
//   4) join worker，等待其处理完队列中剩余日志后退出
//
// worker_loop 退出条件：running_ == false 且 queue_ 已清空
//
void Logger::stop() {
    if (!running_) return;
    running_ = false;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (ofs_.is_open()) ofs_.close();
}

// ─────────────────────────────────────────────────────────────────────────────
// 公共 API：log() 与便捷方法
// ─────────────────────────────────────────────────────────────────────────────
//
// log() 流程：
//   1) 若 level < min_level_，直接丢弃（减少无效入队）
//   2) 构造 LogEntry（含当前时间戳）
//   3) 持锁入队，锁粒度尽量小
//   4) notify_one 唤醒 worker（若有）
//
void Logger::log(LogLevel level, const std::string& tag, const std::string& msg) {
    if (level < min_level_) return;

    LogEntry entry{level, tag, msg, std::chrono::system_clock::now()};

    {
        std::lock_guard<std::mutex> lk(mutex_);
        queue_.push(std::move(entry));
    }
    cv_.notify_one();
}

void Logger::debug(const std::string& tag, const std::string& msg) { log(LogLevel::DEBUG, tag, msg); }
void Logger::info (const std::string& tag, const std::string& msg) { log(LogLevel::INFO,  tag, msg); }
void Logger::warn (const std::string& tag, const std::string& msg) { log(LogLevel::WARN,  tag, msg); }
void Logger::error(const std::string& tag, const std::string& msg) { log(LogLevel::ERROR, tag, msg); }

// ─────────────────────────────────────────────────────────────────────────────
// 后台写盘线程 worker_loop
// ─────────────────────────────────────────────────────────────────────────────
//
// 设计要点：
//   - 外层 while：保证 stop 后仍会处理完队列中剩余日志，避免丢失
//   - cv_.wait 条件：队列非空 或 running_ 为 false 时唤醒
//   - 出队在锁内，写盘在锁外：缩短持锁时间，减少对 log() 入队的阻塞
//
// 执行流程：
//   1) 若 running_ 或 queue_ 非空，继续循环
//   2) 加锁，等待“有新日志”或“stop 信号”
//   3) 内层 while：批量出队
//      a) 在锁内取出 front，pop，移动语义减少拷贝
//      b) 解锁
//      c) 调用 write_entry 写 stdout + 文件（不持锁）
//      d) 重新加锁，继续处理下一项
//   4) 队列清空后，若 running_ 为 false，退出
//
void Logger::worker_loop() {
    // 线程私有队列，用于批量交换日志，降低加锁时间
    std::queue<LogEntry> local_queue;

    while (running_ || !queue_.empty()) {
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this]{ return !queue_.empty() || !running_; });

            // 交换全局队列和本地队列，交换后 queue_ 立即可用于生产侧入队
            std::swap(queue_, local_queue);
        } // 唤醒后，立即解锁 mutex_，最大限度缩短持有时间

        while (!local_queue.empty()) {
            write_entry(local_queue.front());
            local_queue.pop();
        }

        // 写完一批日志，立刻 flush 文件，保证落盘性
        if (ofs_.is_open()) ofs_.flush();
    }
}

//
// write_entry：将单条日志输出到 stdout 和 ofs_（纯写入，不涉及打开/关闭）
//
void Logger::write_entry(const LogEntry& entry) {
    std::string line = format_entry(entry);

    std::cout << line << "\n";
    if (ofs_.is_open()) ofs_ << line << "\n";
}

//
// format_entry：格式化为 "[YYYY-MM-DD HH:MM:SS.mmm] [LEVEL] [tag] message"
//
std::string Logger::format_entry(const LogEntry& entry) const {
    static const char* level_str[] = {"DEBUG", "INFO ", "WARN ", "ERROR"};

    auto t  = std::chrono::system_clock::to_time_t(entry.timestamp);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  entry.timestamp.time_since_epoch()) % 1000;

    // 使用线程安全的 localtime_r
    struct tm tm_info;
    localtime_r(&t, &tm_info); // Linux 下的线程安全版本

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << ms.count()
        << " [" << level_str[static_cast<int>(entry.level)] << "]"
        << " [" << entry.tag << "] "
        << entry.message;

    return oss.str();
}
