#include "serial_port.hpp"
#include "../logger/logger.hpp"

#include <unistd.h>
#include <algorithm>

/**
 * 第一阶段：这里只是框架骨架，函数体大多是 stub。
 * 第二阶段（第4-6周）逐步填入：
 *   - open()       : termios 配置
 *   - rx_loop()    : epoll + 粘包解析
 *   - send_frame() : write() + CRC
 *   - calc_crc16() : CRC16-CCITT 实现
 */

SerialPort::SerialPort(const SerialConfig& cfg)
    : config_(cfg)
    , rx_buf_()
{
    rx_buf_.reserve(1024);
}

SerialPort::~SerialPort() {
    stop();
    close();
}

bool SerialPort::open() {
    // TODO (第二阶段): open() + termios + O_NONBLOCK
    LOG_WARN("comm", "SerialPort::open() - stub, not implemented yet");
    return false;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (epoll_fd_ >= 0) {
        ::close(epoll_fd_);
        epoll_fd_ = -1;
    }
    if (event_fd_ >= 0) {
        ::close(event_fd_);
        event_fd_ = -1;
    }
}

bool SerialPort::is_open() const {
    return fd_ >= 0;
}

bool SerialPort::send_frame(const RawFrame& /*frame*/) {
    // TODO (第二阶段): write() + mutex 保护
    LOG_WARN("comm", "SerialPort::send_frame() - stub");
    return false;
}

void SerialPort::set_frame_callback(FrameCallback cb) {
    frame_cb_ = std::move(cb);
}

void SerialPort::start() {
    if (running_) return;
    if (!is_open()) {
        LOG_WARN("comm", "SerialPort::start() - port not open, threads not started");
        return;
    }
    running_ = true;
    // TODO (第二阶段): 启动 rx_thread_ 和 hb_thread_
    LOG_INFO("comm", "SerialPort threads started (stub mode)");
}

void SerialPort::stop() {
    if (!running_) return;
    running_ = false;

    // 向 event_fd_ 写入 8 字节，强行唤醒沉睡的 epoll_wait
    if (event_fd_ >= 0) {
        uint64_t wake_up_signal = 1;
        ::write(event_fd_, &wake_up_signal, sizeof(wake_up_signal));
    }

    if (rx_thread_.joinable()) rx_thread_.join();
    if (hb_thread_.joinable()) hb_thread_.join();
    LOG_INFO("comm", "SerialPort threads stopped");
}

void SerialPort::rx_loop() {
    // TODO (第二阶段): epoll_wait → read → parse_frame → frame_cb_
}

void SerialPort::heartbeat_loop() {
    // TODO (第二阶段): 固定周期发 HEARTBEAT 帧
}

bool SerialPort::parse_frame(const uint8_t* /*buf*/, size_t /*len*/, RawFrame& /*out*/) {
    // TODO (第二阶段): 查找帧头 → 读 length → 校验 CRC → 返回完整帧
    return false;
}

uint16_t SerialPort::calc_crc16(const uint8_t* /*data*/, size_t /*len*/) {
    // TODO (第二阶段): CRC16-CCITT (0xFFFF init, 0x1021 poly)
    return 0;
}

RawFrame SerialPort::build_frame(FrameType type, const uint8_t* payload, uint16_t len) {
    RawFrame f{};
    f.type   = type;
    f.length = std::min(len, static_cast<uint16_t>(sizeof(f.payload)));
    if (payload && f.length > 0) {
        std::copy(payload, payload + f.length, f.payload);
    }
    // TODO (第二阶段): 填 seq、计算 CRC
    return f;
}
