# dual_arm_middleware

双臂机器人中间件，运行于 RK3588 开发板（Orange Pi 5），负责上位机与两路 STM32 下位机之间的指令下发、状态回收、轨迹生成与全局安全管控。

> **第二阶段（串口通信）已完成**：`SerialPort` 已实现 `open()`（termios 8N1 + epoll + eventfd）、`rx_loop()` 非阻塞读、`send_frame()`、逐字节解析状态机（抗粘包/半包）、CRC16-CCITT 校验与心跳线程。

---

## 目录

- [项目简介](#项目简介)
- [系统架构](#系统架构)
- [模块说明](#模块说明)
- [通讯协议](#通讯协议)
- [目录结构](#目录结构)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)
- [配置文件](#配置文件)
- [测试](#测试)
- [开发路线图](#开发路线图)

---

## 项目简介


| 属性    | 说明                                     |
| ----- | -------------------------------------- |
| 目标平台  | Orange Pi 5 / RK3588，Linux 6.1，aarch64 |
| 语言标准  | C++17                                  |
| 构建系统  | CMake 3.16+                            |
| 下位机   | STM32（左臂 `/dev/ttyS3`，右臂 `/dev/ttyS4`） |
| 单臂自由度 | 6 DOF                                  |
| 控制周期  | 10 ms                                  |


---

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   dual_arm_middleware                │
│                                                     │
│  ┌─────────────┐      ┌───────────────────────┐     │
│  │  Trajectory  │─push─▶  RingBuffer<JointPt>  │     │
│  │  Generator   │      └──────────┬────────────┘     │
│  │（五次多项式）  │                 │ pop              │
│  └─────────────┘      ┌──────────▼────────────┐     │
│                        │    Communication       │     │
│  ┌─────────────┐       │      Thread           │     │
│  │ StateMachine│       │  SerialPort (epoll)   │     │
│  │INIT→IDLE→   │       └──────────┬────────────┘     │
│  │RUNNING→...  │                  │ UART              │
│  └─────────────┘       ┌──────────▼────────────┐     │
│                        │  STM32 Left / Right   │     │
│  ┌─────────────┐       │ /dev/ttyS3 /dev/ttyS4 │     │
│  │  Watchdog   │       └───────────────────────┘     │
│  │（线程超时检测）│                                    │
│  └─────────────┘                                    │
│                                                     │
│  ┌─────────────────────────────────────────────┐    │
│  │       Logger（异步，单独 worker 线程）         │    │
│  └─────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

---

## 模块说明

### `comm` — 串口通信（第二阶段 ✅）

- `open()`：`O_RDWR | O_NOCTTY | O_NONBLOCK`，`termios` 配置 8N1 原始模式；`eventfd` + `epoll` 同时监听串口可读与退出信号
- `rx_loop()`：`epoll_wait` → 非阻塞 `read` 排空内核缓冲 → `feed_bytes()`；优先处理 `eventfd` 并读空计数器
- 粘包 / 半包：**逐字节解析状态机**（`ParseState`），单遍扫描、无 memmove，CRC 在解析过程中增量累积
- CRC16-CCITT（初值 `0xFFFF`，多项式 `0x1021`），与协议定义一致
- 心跳线程：周期发送 `HEARTBEAT` 帧；`send_frame()` 互斥锁保护、线上字节序列化 + `write` 循环

### `trajectory` — 轨迹生成

- 五次多项式规划：同时约束初末位置、速度、加速度，保证加速度连续
- 独立推送线程，以 `control_period_ms` 为周期采样轨迹点
- 通过回调将 `JointPoint` 压入无锁环形缓冲区

### `utils/ring_buffer` — SPSC 无锁环形缓冲区

- 单生产者 / 单消费者，`std::atomic` head/tail，无锁
- 容量必须为 2 的幂，用位与替代取模
- `alignas(64)` 将 head/tail 分置独立 cacheline，避免 false sharing
- 支持 `push(T&&)`（移动语义）和 `pop()` 返回 `std::optional<T>`

### `utils/state_machine` — 状态机

合法状态转换关系：

```
INIT → IDLE → RUNNING ⇄ PAUSED
                  ↓
                ERROR → SHUTDOWN
```

非法转换会被静默拒绝并返回 `false`，防止系统进入非预期状态。

### `watchdog` — 线程级看门狗

- 受监控线程定期调用 `feed("name")`
- 看门狗检测线程超时（默认 2000 ms）后触发回调
- 回调内可将状态机切换为 `ERROR` 或触发重启

### `logger` — 异步日志

- 单例，全进程一个实例
- 业务线程调用 `LOG_*` 宏只负责入队（`std::queue`），不做磁盘 IO
- 后台 worker 线程统一格式化并写 stdout / 文件
- 分级：`DEBUG / INFO / WARN / ERROR`，可配置最低输出级别

---

## 通讯协议

串口线上帧格式（字节序，小端）：


| 偏移      | 字段        | 长度      | 说明             |
| ------- | --------- | ------- | -------------- |
| 0       | `SOF1`    | 1 B     | 固定 `0xAA`      |
| 1       | `SOF2`    | 1 B     | 固定 `0x55`      |
| 2       | `TYPE`    | 1 B     | 帧类型（见下表）       |
| 3       | `SEQ`     | 1 B     | 序列号，0~255 循环   |
| 4~5     | `LEN`     | 2 B     | Payload 长度，小端  |
| 6~N-3   | `PAYLOAD` | 0~128 B | 数据载荷           |
| N-2~N-1 | `CRC16`   | 2 B     | CRC16-CCITT，小端 |


> **CRC16 计算范围**：`TYPE + SEQ + LEN + PAYLOAD`，不含 `SOF` 和 `CRC` 本身。

帧类型定义：


| 值      | 名称              | 方向  | 说明                  |
| ------ | --------------- | --- | ------------------- |
| `0x01` | `HEARTBEAT`     | 主→从 | 心跳，固定周期发送           |
| `0x02` | `JOINT_CMD`     | 主→从 | 关节指令（含 6 DOF 位置/速度） |
| `0x03` | `STATUS_REPORT` | 从→主 | 下位机状态上报             |
| `0x04` | `ACK`           | 从→主 | 确认应答                |
| `0x05` | `ERROR_REPORT`  | 从→主 | 故障上报                |


---

## 目录结构

```
dual_arm_middleware/
├── CMakeLists.txt
├── config/
│   └── system.json          # 系统配置（串口、轨迹、日志参数）
├── docs/
│   ├── preview.md
│   └── protocol.md
├── src/
│   ├── main.cpp             # 程序入口，线程编排
│   ├── common_types.hpp     # 全局数据类型（JointPoint、RawFrame 等）
│   ├── comm/
│   │   ├── serial_port.hpp
│   │   └── serial_port.cpp  # 串口收发（epoll + 粘包拆包 + CRC）
│   ├── logger/
│   │   ├── logger.hpp
│   │   └── logger.cpp       # 异步日志
│   ├── trajectory/
│   │   ├── trajectory_generator.hpp
│   │   └── trajectory_generator.cpp  # 五次多项式轨迹规划
│   ├── utils/
│   │   ├── ring_buffer.hpp  # SPSC 无锁环形缓冲区
│   │   ├── state_machine.hpp / .cpp
│   │   └── thread_pool.hpp
│   └── watchdog/
│       ├── watchdog.hpp
│       └── watchdog.cpp
└── tests/
    ├── test_logger.cpp
    ├── test_ring_buffer.cpp
    └── test_state_machine.cpp
```

---

## 环境依赖


| 依赖            | 版本       | 说明                   |
| ------------- | -------- | -------------------- |
| CMake         | ≥ 3.16   | 构建系统                 |
| GCC / Clang   | 支持 C++17 | 推荐 GCC 10+           |
| nlohmann/json | 3.11.3   | JSON 配置解析，CMake 自动下载 |
| pthread       | 系统自带     | `std::thread` 依赖     |


在 Orange Pi 5（Ubuntu 22.04）上安装依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake git
```

---

## 构建与运行

```bash
# 克隆仓库
git clone https://github.com/<your-username>/dual_arm_middleware.git
cd dual_arm_middleware

# 创建构建目录
mkdir build && cd build

# 配置（Debug 模式）
cmake .. -DCMAKE_BUILD_TYPE=Debug

# 编译
make -j$(nproc)

# 运行（需要串口设备，否则串口模块处于 stub 模式）
./src/dual_arm_middleware
```

Release 模式构建：

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## 配置文件

`config/system.json` 控制所有运行时参数：

```json
{
    "serial_left":  { "port": "/dev/ttyS3", "baud_rate": 115200, "timeout_ms": 100 },
    "serial_right": { "port": "/dev/ttyS4", "baud_rate": 115200, "timeout_ms": 100 },
    "trajectory": {
        "max_velocity": 1.0,
        "max_acceleration": 2.0,
        "buffer_size": 256,
        "control_period_ms": 10.0
    },
    "log_dir": "/var/log/middleware",
    "log_level": 0,
    "watchdog_timeout_ms": 2000
}
```

`log_level`：`0`=DEBUG，`1`=INFO，`2`=WARN，`3`=ERROR。

---

## 测试

```bash
cd build
make -j$(nproc)

# 运行全部测试
ctest --output-on-failure

# 单独运行某个测试
./tests/test_ring_buffer
./tests/test_logger
./tests/test_state_machine
```

---

## 开发路线图


| 阶段   | 内容                                                      | 状态   |
| ---- | ------------------------------------------------------- | ---- |
| 第一阶段 | 框架搭建：目录结构、CMake、Logger、RingBuffer、StateMachine、Watchdog | ✅ 完成 |
| 第二阶段 | 串口通信：`open`/termios、epoll + eventfd、`rx_loop`、`send_frame`、字节流解析状态机、CRC16、心跳 | ✅ **已完成** |
| 第三阶段 | 轨迹规划：五次多项式求解、推送线程、双臂同步                                  | 待开始  |
| 第四阶段 | JSON 配置加载、ArmStatus 回收解析、故障处理闭环                         | 待开始  |
| 第五阶段 | 性能调优、集成测试、上板联调                                          | 待开始  |


