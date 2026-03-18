# 通信协议设计文档

## 帧结构

```
+--------+--------+--------+--------+---------+---------+----------+--------+--------+
| 0xAA   | 0x55   | Type   | Seq    | Len(L)  | Len(H)  | Payload  | CRC(L) | CRC(H) |
| 1 byte | 1 byte | 1 byte | 1 byte | 1 byte  | 1 byte  | N bytes  | 1 byte | 1 byte |
+--------+--------+--------+--------+---------+---------+----------+--------+--------+
```

### 字段说明

| 字段 | 大小 | 说明 |
|------|------|------|
| Header | 2 bytes | 固定 `0xAA 0x55`，用于帧同步 |
| Type | 1 byte | 帧类型，见下表 |
| Seq | 1 byte | 序列号，0~255 循环，超时重发时用于去重 |
| Length | 2 bytes | Payload 长度，小端序 |
| Payload | N bytes | 最大 64 字节 |
| CRC16 | 2 bytes | 对 Type+Seq+Length+Payload 做 CRC16-CCITT |

### 为什么要加 Length 字段

- 串口是字节流，没有天然分包边界
- 只靠帧头无法确定帧结束位置
- Length 字段告诉接收方读多少字节，避免等待超时

## 帧类型

| Type | 值 | 方向 | 说明 |
|------|----|------|------|
| HEARTBEAT | 0x01 | 双向 | 每 500ms 发送，对方无响应则判断链路断开 |
| JOINT_CMD | 0x02 | 上→下 | 关节位置指令，6个 double（48 bytes） |
| STATUS_REPORT | 0x03 | 下→上 | STM32 上报关节位置+错误码 |
| ACK | 0x04 | 双向 | 对 JOINT_CMD 的确认 |
| ERROR_REPORT | 0x05 | 下→上 | STM32 异常上报 |

## 可靠性机制

### 超时重发
1. 上位机发送 JOINT_CMD，记录 `(seq, timestamp)`
2. 等待 ACK，超时 100ms 无 ACK → 重发（最多3次）
3. 3次重发失败 → 触发通信错误，状态机进入 ERROR

### 粘包处理
接收缓冲区维护状态机：
```
WAIT_HEADER_1 → WAIT_HEADER_2 → READ_TYPE → READ_SEQ →
READ_LEN_L → READ_LEN_H → READ_PAYLOAD → CHECK_CRC
```

### 心跳机制
- 上位机每 500ms 发送 HEARTBEAT
- 若 2000ms 内未收到 STM32 的任何帧，判定下位机掉线
- 掉线后禁止发送 JOINT_CMD，等待重连

## JOINT_CMD Payload 格式

```c
struct JointCmdPayload {
    uint8_t  arm_id;        // 0=左臂 1=右臂
    uint8_t  reserved[3];   // 对齐填充
    float    position[6];   // 关节角度 rad（float 节省带宽）
    float    velocity[6];   // 关节速度 rad/s
};  // 共 52 bytes
```

## CRC16-CCITT

- 初始值：`0xFFFF`
- 多项式：`0x1021`
- 覆盖范围：Type + Seq + Length(2) + Payload
