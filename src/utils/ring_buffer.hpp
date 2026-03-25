#pragma once

/**
 * ring_buffer.hpp  —  SPSC 无锁环形缓冲区
 *
 * 设计要点：
 *   - 只适用于 单生产者 / 单消费者 场景
 *   - 用 std::atomic 做 head/tail，避免互斥锁
 *   - 容量必须是 2 的幂，用位与代替取模（性能）
 *   - 用 alignas(64) 避免 false sharing（head/tail 各占一个 cacheline）
 *
 * 面试考点：
 *   Q: 为什么 head 和 tail 要放在不同 cacheline？
 *   A: 防止 false sharing。两个核心各自修改 head/tail，
 *      如果在同一 cacheline 会导致 cache 行反复失效，性能下降。
 *
 * 用法：
 *   RingBuffer<JointPoint, 256> buf;
 *   buf.push(point);   // 生产者线程
 *   buf.pop(point);    // 消费者线程
 */

 #include <cstddef>          // 掌管 size_t
#include <cstdint>          // 掌管 uint8_t, uint32_t 等
#include <initializer_list> // 提前安抚 GCC 的大括号初始化机制
#include <atomic>
#include <array>
#include <optional>
#include <cassert>

template<typename T, size_t Capacity>

// 环形缓冲区类模板，T 是存储的数据类型，Capacity 是缓冲区的容量（必须是 2 的幂）
class RingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");

public:
    RingBuffer() : head_(0), tail_(0) {}

    // 生产者调用 —— 返回 false 表示队列已满
    bool push(const T& item) {
        const size_t cur_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (cur_tail + 1) & mask_;

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false;  // full
        }

        buffer_[cur_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    // 生产者调用（移动语义）—— 避免大对象拷贝，如 JointPoint
    bool push(T&& item) {
        const size_t cur_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (cur_tail + 1) & mask_;

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false;  // full
        }

        buffer_[cur_tail] = std::move(item);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    // 消费者调用 —— 返回 std::nullopt 表示队列为空
    std::optional<T> pop() {
        const size_t cur_head = head_.load(std::memory_order_relaxed);

        if (cur_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt;  // empty
        }

        T item = std::move(buffer_[cur_head]);
        head_.store((cur_head + 1) & mask_, std::memory_order_release);
        return item;
    }

    // 消费者调用（输出参数）—— 返回 false 表示队列为空，避免 optional 开销
    bool pop(T& out) {
        const size_t cur_head = head_.load(std::memory_order_relaxed);

        if (cur_head == tail_.load(std::memory_order_acquire)) {
            return false;  // empty
        }

        out = std::move(buffer_[cur_head]);
        head_.store((cur_head + 1) & mask_, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    bool full() const {
        return ((tail_.load(std::memory_order_acquire) + 1) & mask_) ==
               head_.load(std::memory_order_acquire);
    }

    size_t size() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        return (t - h + Capacity) & mask_;
    }

    static constexpr size_t capacity() { return Capacity; }

private:
    static constexpr size_t mask_ = Capacity - 1;

    // 放在不同 cacheline，避免 false sharing
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;

    std::array<T, Capacity> buffer_;
};
