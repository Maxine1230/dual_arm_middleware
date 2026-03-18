#pragma once

/**
 * serial_port.hpp  —  串口通信模块（第二阶段主角）
 *
 * 当前阶段（框架期）：只放接口定义，函数体留 TODO
 * 第二阶段（第4-6周）：填入 termios + epoll 实现
 *
 * 设计要点：
 *   - 非阻塞 open，epoll 监听可读事件
 *   - 粘包拆包：按帧头 + 长度字段切割
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
 */

#include "common_types.hpp"
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <string>
#include <vector>

// 收到完整帧后的回调
using FrameCallback = std::function<void(const RawFrame&)>;

class SerialPort {
public:
    explicit SerialPort(const SerialConfig& cfg);
    ~SerialPort();

    // 打开串口（非阻塞模式）
    bool open();
    void close();
    bool is_open() const;

    // 发送帧（线程安全）
    bool send_frame(const RawFrame& frame);

    // 注册收帧回调
    void set_frame_callback(FrameCallback cb);

    // 启动收发线程
    void start();
    void stop();

    // ── 第二阶段实现 ────────────────────────────────
    // CRC16 计算
    static uint16_t calc_crc16(const uint8_t* data, size_t len);

    // 构建帧（填 header/seq/crc）
    static RawFrame build_frame(FrameType type, const uint8_t* payload, uint16_t len);

private:
    void rx_loop();          // epoll 接收线程
    void heartbeat_loop();   // 心跳发送线程
    bool parse_frame(const uint8_t* buf, size_t len, RawFrame& out);

    SerialConfig config_;
    int          fd_{-1};
    int          epoll_fd_{-1};
    int          event_fd_{-1};  // 用于优雅退出 epoll

    std::atomic<bool>    running_{false};
    std::thread          rx_thread_;
    std::thread          hb_thread_;

    FrameCallback        frame_cb_;

    // 粘包缓冲区
    std::vector<uint8_t> rx_buf_;

    // 序列号（超时重发用）
    std::atomic<uint8_t> seq_{0};

    std::mutex           send_mutex_;
};
