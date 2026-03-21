#pragma once

/**
 * serial_port.hpp  —  串口通信模块（第二阶段主角）
 *
 * 设计要点：
 *   - 非阻塞 open，epoll 监听可读事件
 *   - 粘包拆包：逐字节状态机，天然免疫粘包 / 半包
 *   - CRC16 校验
 *   - 超时重发：seq 序列号 + 定时器
 *   - 心跳包：固定周期发送 HEARTBEAT 帧
 *
 * 帧格式规范见 src/common_types.hpp：
 *   SOF(0xAA 0x55) + TYPE + SEQ + LEN(LE16) + PAYLOAD + CRC16(LE16)
 *   CRC 覆盖 TYPE/SEQ/LEN/PAYLOAD，不包含 SOF 和 CRC 本身。
 *
 * 面试考点：
 *   Q: 为什么不用阻塞 read？
 *   A: 阻塞 read 会卡住线程，无法同时监听多路或响应停止信号。
 *      用 epoll 可以同时监听串口 fd 和 eventfd（用于优雅退出）。
 *
 *   Q: 粘包/半包怎么处理？
 *   A: 不做缓冲区累积再查找帧头 —— 用逐字节状态机。
 *      每个字节驱动一次状态转移，状态在两次 read() 之间保持。
 *      - 粘包：一次 read 返回多帧数据 → 循环喂字节，自然拆出多帧。
 *      - 半包：一次 read 只返回半帧 → 状态停在中间，下次 read 继续。
 *      好处：零拷贝、O(n) 单遍扫描、无 memmove。
 */

#include "common_types.hpp"
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <string>

// ── 字节流解析状态机状态 ─────────────────────────────
//
//  WAIT_SOF1 → WAIT_SOF2 → READ_TYPE → READ_SEQ
//      ↑                                    ↓
//  READ_CRC_H ← READ_CRC_L ← READ_PAYLOAD ← READ_LEN_H ← READ_LEN_L
//      │
//      └─→ 校验 CRC → 分发帧 → 回到 WAIT_SOF1
//
enum class ParseState : uint8_t {
    WAIT_SOF1,
    WAIT_SOF2,
    READ_TYPE,
    READ_SEQ,
    READ_LEN_L,
    READ_LEN_H,
    READ_PAYLOAD,
    READ_CRC_L,
    READ_CRC_H,
};

using FrameCallback = std::function<void(const RawFrame&)>;

class SerialPort {
public:
    explicit SerialPort(const SerialConfig& cfg);
    ~SerialPort();

    bool open();
    void close();
    bool is_open() const;

    bool send_frame(const RawFrame& frame);
    void set_frame_callback(FrameCallback cb);

    void start();
    void stop();

    /**
     * 喂入原始字节流。
     * rx_loop 每次 read() 拿到多少字节就原样传入，
     * 内部状态机逐字节驱动，收齐一帧后自动调用 frame_cb_。
     * 可在同一次调用中连续吐出多帧（粘包），
     * 也可跨多次调用拼出一帧（半包）。
     */
    void feed_bytes(const uint8_t* data, size_t len);

    static uint16_t calc_crc16(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);
    static uint16_t crc16_step(uint16_t crc, uint8_t byte);
    static RawFrame build_frame(FrameType type, const uint8_t* payload, uint16_t len);

private:
    void rx_loop();
    void heartbeat_loop();
    void on_frame_parsed();
    void reset_parser();

    SerialConfig config_;
    int          fd_{-1};
    int          epoll_fd_{-1};
    int          event_fd_{-1};

    std::atomic<bool>    running_{false};
    std::thread          rx_thread_;
    std::thread          hb_thread_;

    FrameCallback        frame_cb_;
    std::atomic<uint8_t> seq_{0};
    std::mutex           send_mutex_;

    // ── 字节流解析状态机（记忆变量）──────────────────
    // 在两次 feed_bytes() 调用之间保持不变，
    // 让状态机能跨越任意次 read() 边界。
    ParseState  parse_state_{ParseState::WAIT_SOF1};
    RawFrame    parse_frame_{};
    uint16_t    parse_payload_idx_{};
    uint16_t    parse_crc_acc_{0xFFFF};  // CRC 随字节流实时累积
};
