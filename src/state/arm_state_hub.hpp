#pragma once

#include "common_types.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>

/**
 * 上位机全局机械臂遥测「字典」：消费从→主 STATUS_REPORT (0x03)，
 * 反序列化后按 arm_id 写入最新 ArmStatus，并可注册业务层回调（如监控、FSM）。
 *
 * 线程安全：ingest_frame 在串口 RX 线程调用；try_snapshot / is_valid 可在任意线程调用。
 */
class ArmStateHub {
public:
    static ArmStateHub& instance();

    ArmStateHub(const ArmStateHub&) = delete;
    ArmStateHub& operator=(const ArmStateHub&) = delete;

    /**
     * 由 SerialPort 收帧回调最先调用：仅处理 TYPE=STATUS_REPORT 且长度匹配的一帧
     * （CRC 已在 comm 层校验通过）。
     */
    void ingest_frame(const RawFrame& f);

    /** 若该臂已有至少一帧合法上报，拷贝出最新状态 */
    bool try_snapshot(ArmID id, ArmStatus& out) const;

    bool has_valid(ArmID id) const;

    /** 业务回调：每成功更新一条臂状态时调用（在互斥锁外执行，勿在回调里再调 ingest） */
    using ArmStatusListener = std::function<void(ArmID id, const ArmStatus& status)>;
    void set_listener(ArmStatusListener cb);

    /** 最近一次收到 STATUS_REPORT 的本地时间（无数据则返回 false） */
    bool last_rx_time(ArmID id, std::chrono::steady_clock::time_point& t) const;

private:
    ArmStateHub() = default;

    struct Slot {
        ArmStatus                                  status{};
        bool                                       valid{false};
        std::chrono::steady_clock::time_point     last_rx{};
        uint8_t                                    last_seq{0};
    };

    mutable std::mutex mutex_;
    Slot               slots_[2]{};
    ArmStatusListener  listener_{};
};
