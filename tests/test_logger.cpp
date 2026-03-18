/*---——-------------------------------------------------------/
* test_logger.cpp
* 测试 logger 模块的功能，包括单线程和多线程日志记录
* 使用 Logger 单例初始化日志系统，记录不同级别的消息，并测试并发写入
/----------------------------------------------------------*/

#include "logger/logger.hpp"
#include <thread>
#include <chrono>
#include <iostream>

int main() {
    // 初始化 Logger 单例，设置日志目录为 /tmp/test_logger，日志级别为 DEBUG
    Logger::instance().init("/tmp/test_logger", LogLevel::DEBUG);

    // 记录不同级别的日志消息
    LOG_DEBUG("test", "This is a debug message");
    LOG_INFO ("test", "System initialized");
    LOG_WARN ("test", "Buffer almost full (90%)");
    LOG_ERROR("test", "Communication timeout");

    // 多线程并发写日志
    auto worker = [](const std::string& tag) {
        for (int i = 0; i < 20; ++i) {
            LOG_INFO(tag, "Message #" + std::to_string(i) + " from " + tag);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };

    std::thread t1(worker, "thread-1");
    std::thread t2(worker, "thread-2");
    std::thread t3(worker, "thread-3");

    t1.join(); t2.join(); t3.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LOG_INFO("test", "Logger test complete");
    Logger::instance().stop();

    std::cout << "[PASS] logger test - check /tmp/test_logger/middleware.log\n";
    return 0;
}
