#pragma once

/**
 * state_machine.hpp  —  系统状态机
 *
 * 状态：INIT → IDLE → RUNNING ⇄ PAUSED
 *                          ↓
 *                        ERROR → SHUTDOWN
 *
 * 面试考点：
 *   Q: 为什么要状态机？
 *   A: 防止非法状态转换（e.g. 直接从INIT跳到RUNNING），
 *      让系统行为可预测，便于调试。
 */

#include "common_types.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <string>

class StateMachine {
public:
    using StateChangeCallback = std::function<void(SystemState from, SystemState to)>;

    StateMachine();

    // 尝试转换状态，成功返回 true
    bool transition(SystemState target);

    SystemState current() const;

    // 注册状态变化回调（线程安全）
    void on_state_change(StateChangeCallback cb);

    std::string state_name(SystemState s) const;

private:
    bool is_valid_transition(SystemState from, SystemState to) const;

    mutable std::mutex      mutex_;
    std::atomic<SystemState> state_{SystemState::INIT};
    StateChangeCallback     callback_;
};
