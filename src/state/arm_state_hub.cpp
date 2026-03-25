#include "arm_state_hub.hpp"

#include <cstring>

ArmStateHub& ArmStateHub::instance() {
    static ArmStateHub hub;
    return hub;
}

void ArmStateHub::ingest_frame(const RawFrame& f) {
    if (f.type != FrameType::STATUS_REPORT) {
        return;
    }
    if (f.length != sizeof(StatusReportPayload)) {
        return;
    }

    StatusReportPayload pl{};
    std::memcpy(&pl, f.payload, sizeof(pl));
    if (pl.arm_id > 1) {
        return;
    }

    ArmStatus st{};
    st.arm_id        = static_cast<ArmID>(pl.arm_id);
    std::memcpy(st.joint_pos, pl.joint_pos, sizeof(st.joint_pos));
    std::memcpy(st.joint_vel, pl.joint_vel, sizeof(st.joint_vel));
    st.error_code    = pl.error_code;
    st.timestamp_ms  = pl.timestamp_ms;

    ArmStatusListener cb_copy;
    {
        std::unique_lock<std::mutex> lk(mutex_);
        const int idx = static_cast<int>(st.arm_id);
        slots_[idx].status  = st;
        slots_[idx].valid   = true;
        slots_[idx].last_rx = std::chrono::steady_clock::now();
        slots_[idx].last_seq = f.seq;
        cb_copy = listener_;
    }

    if (cb_copy) {
        cb_copy(st.arm_id, st);
    }
}

bool ArmStateHub::try_snapshot(ArmID id, ArmStatus& out) const {
    const int idx = static_cast<int>(id);
    if (idx < 0 || idx > 1) {
        return false;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    if (!slots_[idx].valid) {
        return false;
    }
    out = slots_[idx].status;
    return true;
}

bool ArmStateHub::has_valid(ArmID id) const {
    const int idx = static_cast<int>(id);
    if (idx < 0 || idx > 1) {
        return false;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    return slots_[idx].valid;
}

void ArmStateHub::set_listener(ArmStatusListener cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    listener_ = std::move(cb);
}

bool ArmStateHub::last_rx_time(ArmID id, std::chrono::steady_clock::time_point& t) const {
    const int idx = static_cast<int>(id);
    if (idx < 0 || idx > 1) {
        return false;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    if (!slots_[idx].valid) {
        return false;
    }
    t = slots_[idx].last_rx;
    return true;
}
