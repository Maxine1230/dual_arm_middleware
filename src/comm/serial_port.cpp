#include "serial_port.hpp"
#include "../logger/logger.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <cerrno>
#include <algorithm>
#include <cstring>
#include <chrono>

// ═════════════════════════════════════════════════════════════════════════════
//  baud_rate (int) → termios speed_t
// ═════════════════════════════════════════════════════════════════════════════

static speed_t baud_to_speed(int baud) {
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default:      return B115200;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  构造 / 析构
// ═════════════════════════════════════════════════════════════════════════════

SerialPort::SerialPort(const SerialConfig& cfg)
    : config_(cfg)
{}

SerialPort::~SerialPort() {
    stop();
    close();
}

// ═════════════════════════════════════════════════════════════════════════════
//  打开 / 关闭
// ═════════════════════════════════════════════════════════════════════════════

bool SerialPort::open() {
    if (fd_ >= 0) {
        LOG_WARN("comm", "SerialPort already open: " + config_.port);
        return true;
    }

    // ── 1. 打开设备文件 ─────────────────────────────────────────
    //  O_RDWR     : 读写
    //  O_NOCTTY   : 不让串口成为进程的控制终端
    //  O_NONBLOCK : 非阻塞，配合 epoll 使用
    fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        LOG_ERROR("comm", "open() failed: " + config_.port +
                  " errno=" + std::to_string(errno) + " " + strerror(errno));
        return false;
    }

    // ── 2. termios 配置 8N1 ─────────────────────────────────────
    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        LOG_ERROR("comm", "tcgetattr failed: " + std::string(strerror(errno)));
        ::close(fd_); fd_ = -1;
        return false;
    }

    const speed_t spd = baud_to_speed(config_.baud_rate);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // 8 数据位，无校验，1 停止位，无硬件流控
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |=  (CS8 | CLOCAL | CREAD);

    // 原始模式（关闭行编辑、回显、信号字符）
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 关闭软件流控、特殊输入处理
    tty.c_iflag &= ~(IXON | IXOFF | IXANY |
                      IGNBRK | BRKINT | PARMRK | ISTRIP |
                      INLCR | IGNCR | ICRNL);

    // 原始输出
    tty.c_oflag &= ~OPOST;

    // 非阻塞语义：VMIN=0, VTIME=0 → read() 立即返回可用字节
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        LOG_ERROR("comm", "tcsetattr failed: " + std::string(strerror(errno)));
        ::close(fd_); fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    // ── 3. eventfd：用于优雅退出 ────────────────────────────────
    //  stop() 向 event_fd_ 写 8 字节 → epoll_wait 被唤醒 → rx_loop 退出
    event_fd_ = ::eventfd(0, EFD_NONBLOCK);
    if (event_fd_ < 0) {
        LOG_ERROR("comm", "eventfd failed: " + std::string(strerror(errno)));
        ::close(fd_); fd_ = -1;
        return false;
    }

    // ── 4. epoll：同时监听串口 fd + eventfd ─────────────────────
    epoll_fd_ = ::epoll_create1(0);
    if (epoll_fd_ < 0) {
        LOG_ERROR("comm", "epoll_create1 failed: " + std::string(strerror(errno)));
        ::close(event_fd_); event_fd_ = -1;
        ::close(fd_);       fd_ = -1;
        return false;
    }

    epoll_event ev{};
    ev.events  = EPOLLIN;
    ev.data.fd = fd_;
    if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd_, &ev) != 0) {
        LOG_ERROR("comm", "epoll_ctl add serial fd failed");
        close();
        return false;
    }

    ev.data.fd = event_fd_;
    if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_, &ev) != 0) {
        LOG_ERROR("comm", "epoll_ctl add event fd failed");
        close();
        return false;
    }

    LOG_INFO("comm", "Opened " + config_.port +
             " @ " + std::to_string(config_.baud_rate) + " 8N1");
    return true;
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

// ═════════════════════════════════════════════════════════════════════════════
//  发送
// ═════════════════════════════════════════════════════════════════════════════

bool SerialPort::send_frame(const RawFrame& frame) {
    if (fd_ < 0) return false;

    // 将 RawFrame 序列化为连续线上字节
    uint8_t wire[FRAME_MAX_WIRE_SIZE];
    size_t pos = 0;

    wire[pos++] = FRAME_HEADER_1;
    wire[pos++] = FRAME_HEADER_2;
    wire[pos++] = static_cast<uint8_t>(frame.type);
    wire[pos++] = frame.seq;
    wire[pos++] = static_cast<uint8_t>(frame.length & 0xFF);
    wire[pos++] = static_cast<uint8_t>(frame.length >> 8);

    std::memcpy(wire + pos, frame.payload, frame.length);
    pos += frame.length;

    wire[pos++] = static_cast<uint8_t>(frame.crc16 & 0xFF);
    wire[pos++] = static_cast<uint8_t>(frame.crc16 >> 8);

    // 互斥锁保护：多线程（通信线程 + 心跳线程）可能并发调用
    std::lock_guard<std::mutex> lock(send_mutex_);

    const uint8_t* p = wire;
    size_t remaining = pos;
    while (remaining > 0) {
        ssize_t n = ::write(fd_, p, remaining);
        if (n > 0) {
            p         += n;
            remaining -= static_cast<size_t>(n);
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
            LOG_ERROR("comm", "write() failed: " + std::string(strerror(errno)));
            return false;
        }
    }
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  回调 / 线程管理
// ═════════════════════════════════════════════════════════════════════════════

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
    reset_parser();
    rx_thread_ = std::thread(&SerialPort::rx_loop, this);
    hb_thread_ = std::thread(&SerialPort::heartbeat_loop, this);
    LOG_INFO("comm", "SerialPort threads started");
}

void SerialPort::stop() {
    if (!running_) return;
    running_ = false;

    if (event_fd_ >= 0) {
        uint64_t wake_up_signal = 1;
        ::write(event_fd_, &wake_up_signal, sizeof(wake_up_signal));
    }

    if (rx_thread_.joinable()) rx_thread_.join();
    if (hb_thread_.joinable()) hb_thread_.join();
    LOG_INFO("comm", "SerialPort threads stopped");
}

// ═════════════════════════════════════════════════════════════════════════════
//  接收线程：epoll 驱动 + 非阻塞 read 排空内核缓冲 → feed_bytes
// ═════════════════════════════════════════════════════════════════════════════

void SerialPort::rx_loop() {
    constexpr int MAX_EVENTS = 2;   // fd_ + event_fd_
    epoll_event events[MAX_EVENTS];
    uint8_t buf[512];

    while (running_) {
        int nfds = ::epoll_wait(epoll_fd_, events, MAX_EVENTS, -1);
        if (nfds < 0) {
            if (errno == EINTR) {
                continue;
            }
            LOG_ERROR("comm", "epoll_wait: " + std::string(strerror(errno)));
            break;
        }

        // 先处理 eventfd：与 stop() 同批到达时优先退出，避免无意义读串口
        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd != event_fd_) {
                continue;
            }
            uint64_t drain = 0;
            (void)::read(event_fd_, &drain, sizeof(drain));  // 清空计数器，避免 LT 下重复触发
            return;
        }

        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd != fd_) {
                continue;
            }

            if (events[i].events & (EPOLLERR | EPOLLHUP)) {
                LOG_WARN("comm", "Serial fd EPOLLERR/EPOLLHUP, rx_loop exit");
                return;
            }

            // 非阻塞：把内核里积压的字节一次读尽，降低 epoll 往返次数
            for (;;) {
                ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n > 0) {
                    feed_bytes(buf, static_cast<size_t>(n));
                } else if (n == 0) {
                    LOG_WARN("comm", "Serial read EOF");
                    return;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        break;
                    }
                    LOG_ERROR("comm", "read: " + std::string(strerror(errno)));
                    return;
                }
            }
        }
    }
}

void SerialPort::heartbeat_loop() {
    while (running_) {
        RawFrame hb = build_frame(FrameType::HEARTBEAT, nullptr, 0);
        hb.seq = seq_.fetch_add(1, std::memory_order_relaxed);

        // seq 变了，CRC 必须重算
        const uint8_t hdr[4] = {
            static_cast<uint8_t>(hb.type),
            hb.seq,
            static_cast<uint8_t>(hb.length & 0xFF),
            static_cast<uint8_t>(hb.length >> 8),
        };
        hb.crc16 = calc_crc16(hb.payload, hb.length, calc_crc16(hdr, 4));

        send_frame(hb);

        // 以 config_.timeout_ms 为心跳间隔，分片休眠以便快速响应 stop()
        for (int slept = 0; running_ && slept < config_.timeout_ms; slept += 50) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  字节流解析状态机  ★ 核心算法 ★
// ═════════════════════════════════════════════════════════════════════════════
//
//  设计原则：
//    1. 每个字节只看一次，O(n) 单遍扫描，零 memmove。
//    2. 所有"记忆"保存在 parse_state_ / parse_frame_ / parse_payload_idx_，
//       因此无论 read() 返回多少字节、在哪里断开，都不会丢状态。
//    3. 粘包：一次 feed_bytes 内循环自然会吐出多帧。
//    4. 半包：状态停在中间，下次 feed_bytes 继续推进。
//    5. 噪声容错：任何阶段遇到不合法值都回退到 WAIT_SOF1。
//

void SerialPort::feed_bytes(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = data[i];

        switch (parse_state_) {

        case ParseState::WAIT_SOF1:
            if (b == FRAME_HEADER_1) {
                parse_state_ = ParseState::WAIT_SOF2;
            }
            break;

        case ParseState::WAIT_SOF2:
            if (b == FRAME_HEADER_2) {
                parse_frame_ = RawFrame{};
                parse_crc_acc_ = 0xFFFF;
                parse_state_ = ParseState::READ_TYPE;
            } else if (b == FRAME_HEADER_1) {
                // 连续 0xAA：前一个是噪声，当前可能才是真正的 SOF1
            } else {
                parse_state_ = ParseState::WAIT_SOF1;
            }
            break;

        case ParseState::READ_TYPE:
            parse_frame_.type = static_cast<FrameType>(b);
            parse_crc_acc_ = crc16_step(parse_crc_acc_, b);
            parse_state_ = ParseState::READ_SEQ;
            break;

        case ParseState::READ_SEQ:
            parse_frame_.seq = b;
            parse_crc_acc_ = crc16_step(parse_crc_acc_, b);
            parse_state_ = ParseState::READ_LEN_L;
            break;

        case ParseState::READ_LEN_L:
            parse_frame_.length = b;
            parse_crc_acc_ = crc16_step(parse_crc_acc_, b);
            parse_state_ = ParseState::READ_LEN_H;
            break;

        case ParseState::READ_LEN_H:
            parse_frame_.length |= static_cast<uint16_t>(b) << 8;
            parse_crc_acc_ = crc16_step(parse_crc_acc_, b);

            if (parse_frame_.length > FRAME_MAX_PAYLOAD_LEN) {
                LOG_WARN("comm", "RX frame len=" +
                         std::to_string(parse_frame_.length) + " exceeds max, dropped");
                parse_state_ = ParseState::WAIT_SOF1;
            } else if (parse_frame_.length == 0) {
                parse_state_ = ParseState::READ_CRC_L;
            } else {
                parse_payload_idx_ = 0;
                parse_state_ = ParseState::READ_PAYLOAD;
            }
            break;

        case ParseState::READ_PAYLOAD:
            parse_frame_.payload[parse_payload_idx_++] = b;
            parse_crc_acc_ = crc16_step(parse_crc_acc_, b);
            if (parse_payload_idx_ >= parse_frame_.length) {
                parse_state_ = ParseState::READ_CRC_L;
            }
            break;

        case ParseState::READ_CRC_L:
            parse_frame_.crc16 = b;
            parse_state_ = ParseState::READ_CRC_H;
            break;

        case ParseState::READ_CRC_H:
            parse_frame_.crc16 |= static_cast<uint16_t>(b) << 8;
            on_frame_parsed();
            parse_state_ = ParseState::WAIT_SOF1;
            break;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  一帧收齐后：校验 CRC → 分发回调
// ═════════════════════════════════════════════════════════════════════════════
//
//  CRC 已在 feed_bytes 逐字节累积完毕（parse_crc_acc_），
//  此处只需做一次 16-bit 整数比较，不再访问 payload 内存。

void SerialPort::on_frame_parsed() {
    if (parse_crc_acc_ != parse_frame_.crc16) {
        LOG_WARN("comm",
                 "CRC mismatch: got=0x" + std::to_string(parse_frame_.crc16) +
                 " expected=0x" + std::to_string(parse_crc_acc_) +
                 " seq=" + std::to_string(parse_frame_.seq));
        return;
    }

    if (frame_cb_) {
        frame_cb_(parse_frame_);
    }
}

void SerialPort::reset_parser() {
    parse_state_       = ParseState::WAIT_SOF1;
    parse_frame_       = RawFrame{};
    parse_payload_idx_ = 0;
    parse_crc_acc_     = 0xFFFF;
}

// ═════════════════════════════════════════════════════════════════════════════
//  CRC16-CCITT（初值 0xFFFF，多项式 0x1021，MSB-first）
// ═════════════════════════════════════════════════════════════════════════════

uint16_t SerialPort::crc16_step(uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021)
                             :  (crc << 1);
    }
    return crc;
}

uint16_t SerialPort::calc_crc16(const uint8_t* data, size_t len, uint16_t init) {
    uint16_t crc = init;
    for (size_t i = 0; i < len; ++i) {
        crc = crc16_step(crc, data[i]);
    }
    return crc;
}

// ═════════════════════════════════════════════════════════════════════════════
//  构建帧：填充 SOF / CRC，返回可直接序列化的 RawFrame
// ═════════════════════════════════════════════════════════════════════════════

RawFrame SerialPort::build_frame(FrameType type, const uint8_t* payload, uint16_t len) {
    RawFrame f{};
    f.type   = type;
    f.length = std::min(len, static_cast<uint16_t>(FRAME_MAX_PAYLOAD_LEN));
    if (payload && f.length > 0) {
        std::memcpy(f.payload, payload, f.length);
    }

    const uint8_t hdr[4] = {
        static_cast<uint8_t>(f.type),
        f.seq,
        static_cast<uint8_t>(f.length & 0xFF),
        static_cast<uint8_t>(f.length >> 8),
    };
    f.crc16 = calc_crc16(f.payload, f.length, calc_crc16(hdr, 4));

    return f;
}
