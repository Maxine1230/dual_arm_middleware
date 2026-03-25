#include "utils/state_machine.hpp"
#include <cassert>
#include <iostream>

int main() {
    StateMachine fsm;

    fsm.on_state_change([&fsm](SystemState from, SystemState to) {
        std::cout << "[FSM] " << fsm.state_name(from)
                  << " -> " << fsm.state_name(to) << "\n";
    });

    // 合法转换
    assert(fsm.transition(SystemState::IDLE));
    assert(fsm.current() == SystemState::IDLE);

    // IDLE 可直接因下位机 fault 进入 ERROR（与 STATUS_REPORT error_code 一致）
    assert(fsm.transition(SystemState::ERROR));
    assert(fsm.current() == SystemState::ERROR);
    assert(fsm.transition(SystemState::IDLE));

    assert(fsm.transition(SystemState::RUNNING));
    assert(fsm.transition(SystemState::PAUSED));
    assert(fsm.transition(SystemState::RUNNING));
    assert(fsm.transition(SystemState::ERROR));

    // 非法转换（ERROR 不能直接到 RUNNING）
    bool bad = fsm.transition(SystemState::RUNNING);
    assert(!bad);
    std::cout << "[OK] Illegal transition correctly rejected\n";

    assert(fsm.transition(SystemState::SHUTDOWN));

    std::cout << "[PASS] state_machine test\n";
    return 0;
}
