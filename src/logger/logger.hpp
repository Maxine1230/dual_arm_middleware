#pragma once

/**
 * logger.hpp  -- 异步日志系统（对外接口）
 *
 * 目标：
 *   - 业务线程调用 LOG_* 宏时只负责入队，不做磁盘 IO
 *   - 后台 worker 线程统一格式化并写 stdout / 文件
 *
 * 核心特性：
 *   - 单例模式：全进程只有一个 Logger 实例
 *   - 线程安全：多线程可并发调用 LOG_* 宏
 *   - 日志分级：DEBUG / INFO / WARN / ERROR
 *
 * 使用步骤（建议）：
 *   1) 程序启动时初始化一次
 *      Logger::instance().init("/tmp/middleware_log", LogLevel::DEBUG);
 *   2) 业务代码直接使用宏
 *      LOG_INFO("comm", "serial port opened: /dev/ttyS3");
 *   3) 程序退出前停止
 *      Logger::instance().stop();
 *
 * 注意：
 *   - 这里不提供 printf/fmt 风格格式化，message 请先拼接为 std::string。
 *   - 必须先 init() 再写日志；stop() 后不应继续写日志。
 */

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <sstream>
#include <chrono>
#include <fstream>

// ─────────────────────────────────────────────────────────────────────────────
// 日志级别枚举
// ─────────────────────────────────────────────────────────────────────────────
// DEBUG: 调试信息，生产环境可关闭
// INFO:  正常流程信息
// WARN:  警告，不影响主流程
// ERROR: 错误，需关注
enum class LogLevel : uint8_t {
    DEBUG = 0,
    INFO  = 1,
    WARN  = 2,
    ERROR = 3,
};

// ─────────────────────────────────────────────────────────────────────────────
// 单条日志条目（内部结构，供队列传递）
// ─────────────────────────────────────────────────────────────────────────────
struct LogEntry {
    LogLevel    level;      // 日志级别
    std::string tag;        // 模块标识，如 "comm" "traj" "watchdog"
    std::string message;    // 日志正文
    std::chrono::system_clock::time_point timestamp;  // 入队时间戳
};

// ─────────────────────────────────────────────────────────────────────────────
// Logger 单例类
// ─────────────────────────────────────────────────────────────────────────────
class Logger {
public:
    // 获取全局单例（Meyers' Singleton，C++11 保证线程安全）
    static Logger& instance();

    // 初始化日志系统，创建目录并启动后台写盘线程
    void init(const std::string& log_dir, LogLevel min_level);
    // 停止日志系统，唤醒 worker 并 join，会尽量清空队列中剩余日志
    void stop();

    // 通用日志入口（通常由 LOG_* 宏调用）
    void log(LogLevel level, const std::string& tag, const std::string& msg);

    // 便捷方法
    void debug(const std::string& tag, const std::string& msg);
    void info (const std::string& tag, const std::string& msg);
    void warn (const std::string& tag, const std::string& msg);
    void error(const std::string& tag, const std::string& msg);

private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void worker_loop();                              // 后台线程主循环
    void write_entry(const LogEntry& entry);         // 单条日志落盘
    std::string format_entry(const LogEntry& entry) const;  // 格式化为字符串

    std::thread             worker_;   // 后台写盘线程
    std::mutex              mutex_;    // 保护 queue_
    std::condition_variable cv_;       // 唤醒 worker（有新日志或 stop）
    std::queue<LogEntry>    queue_;    // 待写入的日志队列
    std::atomic<bool>       running_{false};  // 运行标志

    LogLevel    min_level_{LogLevel::DEBUG};  // 最低输出级别：低于此级别的日志会被丢弃
    std::string log_dir_;                      // 日志文件目录
    std::ofstream ofs_;                        // 日志文件流，init 时打开，stop 时关闭
};

// ─────────────────────────────────────────────────────────────────────────────
// 对外宏接口：业务代码统一使用 LOG_*(tag, message)
// ─────────────────────────────────────────────────────────────────────────────
// 示例：
//   LOG_INFO("traj", "planner ready");
//   LOG_ERROR("comm", std::string("open failed: ") + port);
#define LOG_DEBUG(...) Logger::instance().debug(__VA_ARGS__)
#define LOG_INFO(...)  Logger::instance().info (__VA_ARGS__)
#define LOG_WARN(...)  Logger::instance().warn (__VA_ARGS__)
#define LOG_ERROR(...) Logger::instance().error(__VA_ARGS__)
