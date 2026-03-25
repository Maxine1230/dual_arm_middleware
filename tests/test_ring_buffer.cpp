/* test_ring_buffer.cpp
* 这个文件是用来测试环形缓冲区（Ring Buffer）的功能
* 环形缓冲区是一种数据结构，用于在生产者和消费者之间传递数据
* 这里我们模拟一个生产者线程不断产生数据，一个消费者线程不断消费数据
* 测试的目的：看看当生产速度快于消费速度时，会不会丢失数据
*/


/* 包含必要的头文件 */
#include <cstddef>          // 掌管 size_t
#include <cstdint>          // 掌管 uint8_t, uint32_t 等
#include <initializer_list> // 提前安抚 GCC 的大括号初始化机制
#include "utils/ring_buffer.hpp"  // 包含我们自定义的环形缓冲区类
#include "common_types.hpp"       // 包含共同的数据类型定义，比如 JointPoint

#include <thread>      // 用于创建和管理线程
#include <cassert>     // 用于断言（assert），检查条件是否为真
#include <iostream>    // 用于输出到控制台
#include <atomic>      // 用于原子变量，确保多线程安全
#include <chrono>      // 用于时间相关的操作，比如睡眠

/**
 * 测试 SPSC 环形缓冲区：
 *   - 生产者线程以 1ms 为周期 push JointPoint
 *   - 消费者线程以 2ms 为周期 pop
 *   - 运行 1 秒，统计丢失的点数
 */

std::atomic<int> produced{0};
std::atomic<int> consumed{0};
std::atomic<int> dropped{0};

int main() {
    // 创建一个环形缓冲区，存储 JointPoint 类型的数据，容量为 64
    // RingBuffer 是一个模板类，第一个参数是数据类型，第二个是容量
    RingBuffer<JointPoint, 64> buf;

    // 原子布尔变量，用于控制线程的停止
    // 当 stop 为 true 时，线程会退出循环
    std::atomic<bool> stop{false};

    // 创建生产者线程
    // lambda 函数 [&] 表示捕获所有外部变量的引用
    std::thread producer([&]() {
        // 循环直到 stop 为 true
        while (!stop) {
            // 创建一个 JointPoint 对象，默认初始化
            JointPoint pt{};
            // 设置时间戳为当前已生产的数据数量
            pt.timestamp_ms = produced.load();
            // 尝试将数据放入缓冲区
            if (buf.push(pt)) {
                // 如果成功，增加生产计数
                produced++;
            } else {
                // 如果失败（缓冲区满），增加丢弃计数
                dropped++;
            }
            // 睡眠 1 毫秒，模拟生产周期
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // 创建消费者线程
    std::thread consumer([&]() {
        // 循环直到 stop 为 true
        while (!stop) {
            // 尝试从缓冲区取出数据
            auto pt = buf.pop();
            // 检查是否有数据（pop 返回 std::optional）
            if (pt.has_value()) {
                // 如果有数据，增加消费计数
                consumed++;
            }
            // 睡眠 2 毫秒，模拟消费周期（比生产慢）
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        // 线程停止后，清空缓冲区中剩余的所有数据
        while (auto pt = buf.pop()) consumed++;
    });

    // 主线程睡眠 1 秒，让生产者和消费者运行
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 设置停止标志
    stop = true;

    // 等待生产者线程结束
    producer.join();
    // 等待消费者线程结束
    consumer.join();

    // 输出统计结果
    std::cout << "Produced: " << produced << "\n";  // 生产了多少数据
    std::cout << "Consumed: " << consumed << "\n";  // 消费了多少数据
    std::cout << "Dropped:  " << dropped  << "\n";  // 丢弃了多少数据
    std::cout << "Buffer size check: capacity=" << buf.capacity() << "\n";  // 检查缓冲区容量

    // 断言：确保至少生产了一些数据
    assert(produced > 0);
    // 输出测试通过信息
    std::cout << "[PASS] ring_buffer test\n";
    return 0;
}

/*统计得在 ARM 板上测试，
当消费速度是生产速度一半时，64 格缓冲区的丢弃率约为 43%，
因此在实际控制场景中缓冲区大小和控制周期的匹配至关重要。*/