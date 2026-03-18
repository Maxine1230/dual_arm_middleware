#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <cstddef>

// ─────────────────────────────────────────────
//  系统级枚举
// ─────────────────────────────────────────────

enum class SystemState : uint8_t {
    INIT       = 0,
    IDLE       = 1,
    RUNNING    = 2,
    PAUSED     = 3,
    ERROR      = 4,
    SHUTDOWN   = 5,
};

enum class ArmID : uint8_t {
    LEFT  = 0,
    RIGHT = 1,
};

// ─────────────────────────────────────────────
//  轨迹点：单个关节空间采样
// ─────────────────────────────────────────────

constexpr uint8_t DOF = 6;  // 单臂自由度

struct JointPoint {
    ArmID  arm_id;
    float  position[DOF];      // rad
    float  velocity[DOF];      // rad/s
    float  acceleration[DOF];  // rad/s²
    float  timestamp_ms;       // 相对时间戳，单位 ms
};

// ─────────────────────────────────────────────
//  通信帧（上位机 <-> STM32）
// ─────────────────────────────────────────────

constexpr uint8_t FRAME_HEADER_1 = 0xAA;
constexpr uint8_t FRAME_HEADER_2 = 0x55;
constexpr uint16_t FRAME_MAX_PAYLOAD_LEN = 128;

/**
 * 线上的帧格式（字节序）：
 *
 * [0]   SOF1      固定 0xAA
 * [1]   SOF2      固定 0x55
 * [2]   TYPE      FrameType
 * [3]   SEQ       序列号（0~255循环）
 * [4]   LEN_L     payload 长度低字节（小端）
 * [5]   LEN_H     payload 长度高字节（小端）
 * [6..] PAYLOAD   0~128 字节
 * [N-2] CRC_L     CRC16-CCITT 低字节（小端）
 * [N-1] CRC_H     CRC16-CCITT 高字节（小端）
 *
 * CRC16 计算范围：TYPE + SEQ + LEN(2B) + PAYLOAD
 * （不包含 SOF 两字节，不包含 CRC 本身）
 */
constexpr size_t FRAME_OFFSET_SOF1    = 0;
constexpr size_t FRAME_OFFSET_SOF2    = 1;
constexpr size_t FRAME_OFFSET_TYPE    = 2;
constexpr size_t FRAME_OFFSET_SEQ     = 3;
constexpr size_t FRAME_OFFSET_LEN_L   = 4;
constexpr size_t FRAME_OFFSET_LEN_H   = 5;
constexpr size_t FRAME_OFFSET_PAYLOAD = 6;

constexpr size_t FRAME_FIXED_OVERHEAD = 2 /*SOF*/ + 1 /*TYPE*/ + 1 /*SEQ*/ + 2 /*LEN*/ + 2 /*CRC*/;
constexpr size_t FRAME_MIN_WIRE_SIZE  = FRAME_FIXED_OVERHEAD; // payload=0
constexpr size_t FRAME_MAX_WIRE_SIZE  = FRAME_FIXED_OVERHEAD + FRAME_MAX_PAYLOAD_LEN;

enum class FrameType : uint8_t {
    HEARTBEAT       = 0x01,
    JOINT_CMD       = 0x02,
    STATUS_REPORT   = 0x03,
    ACK             = 0x04,
    ERROR_REPORT    = 0x05,
};

// 原始帧（填充前）
struct RawFrame {
    uint8_t   header[2]  = {FRAME_HEADER_1, FRAME_HEADER_2};
    FrameType type;
    uint8_t   seq;        // 序列号，用于超时重发判断
    uint16_t  length;     // payload 长度
    uint8_t   payload[FRAME_MAX_PAYLOAD_LEN];
    uint16_t  crc16;
};

// ─────────────────────────────────────────────
//  下位机状态（STM32 上报）
// ─────────────────────────────────────────────

struct ArmStatus {
    ArmID   arm_id;
    float   joint_pos[DOF];
    float   joint_vel[DOF];
    uint8_t error_code;       // 0 = 正常
    float   timestamp_ms;
};

// ─────────────────────────────────────────────
//  系统配置（从 JSON 读取）
// ─────────────────────────────────────────────

struct SerialConfig {
    std::string port;       // e.g. "/dev/ttyS3"
    int         baud_rate;  // e.g. 115200
    int         timeout_ms;
};

struct TrajectoryConfig {
    double max_velocity;      // rad/s
    double max_acceleration;  // rad/s²
    int    buffer_size;       // 环形缓冲区容量
    double control_period_ms; // 控制周期
};

struct SystemConfig {
    SerialConfig    serial_left;
    SerialConfig    serial_right;
    TrajectoryConfig trajectory;
    std::string     log_dir;
    int             log_level;  // 0=DEBUG 1=INFO 2=WARN 3=ERROR
    int             watchdog_timeout_ms;
};
