#include "state_machine.hpp"

StateMachine::StateMachine() = default;

bool StateMachine::transition(SystemState target) {
    StateChangeCallback cb_copy;
    SystemState from{};

    {
        std::lock_guard<std::mutex> lk(mutex_);
        const SystemState cur = state_.load(std::memory_order_relaxed);

        if (!is_valid_transition(cur, target)) return false;

        from = cur;
        state_.store(target, std::memory_order_release);
        cb_copy = callback_;
    }

    if (cb_copy) cb_copy(from, target);
    return true;
}

SystemState StateMachine::current() const {
    return state_.load(std::memory_order_acquire);
}

void StateMachine::on_state_change(StateChangeCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    callback_ = std::move(cb);
}

bool StateMachine::is_valid_transition(SystemState from, SystemState to) const {
    switch (from) {
        case SystemState::INIT:
            return to == SystemState::IDLE;
        case SystemState::IDLE:
            return to == SystemState::RUNNING || to == SystemState::SHUTDOWN
                || to == SystemState::ERROR;
        case SystemState::RUNNING:
            return to == SystemState::PAUSED || to == SystemState::ERROR || to == SystemState::IDLE;
        case SystemState::PAUSED:
            return to == SystemState::RUNNING || to == SystemState::IDLE || to == SystemState::ERROR;
        case SystemState::ERROR:
            return to == SystemState::SHUTDOWN || to == SystemState::IDLE;
        case SystemState::SHUTDOWN:
            return false;
        default:
            return false;
    }
}

std::string StateMachine::state_name(SystemState s) const {
    switch (s) {
        case SystemState::INIT:     return "INIT";
        case SystemState::IDLE:     return "IDLE";
        case SystemState::RUNNING:  return "RUNNING";
        case SystemState::PAUSED:   return "PAUSED";
        case SystemState::ERROR:    return "ERROR";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        default:                    return "UNKNOWN";
    }
}
