# dual_arm_middleware

双臂机器人中间件，运行于 RK3588 开发板（Orange Pi 5），负责上位机与两路 STM32 下位机之间的指令下发、状态回收、轨迹生成与全局安全管控。

> **第二阶段（串口通信）已完成**：`open()` / `termios` 8N1、`epoll` + `eventfd`、`rx_loop()`、`send_frame()`、`send_joint_command()`（`JOINT_CMD` 载荷）、逐字节解析状态机、CRC16-CCITT、心跳线程。  
> **第三阶段（轨迹与主链路）已贯通**：五次多项式（τ 域闭式系数）、`TrajectoryGenerator` burst 灌入 `RingBuffer`、`main` 中通信线程按 `control_period_ms` 下发关节指令。  
> **第四阶段（配置 + 状态回收 + 故障闭环）已完成**：`system.json` 解析、`STATUS_REPORT(0x03)` 反序列化入全局状态字典、`error_code`/遥测陈旧联动 `FSM -> ERROR`。

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


| 属性    | 说明                                                                                               |
| ----- | ------------------------------------------------------------------------------------------------ |
| 目标平台  | Orange Pi 5 / RK3588，Linux 6.1，aarch64                                                           |
| 语言标准  | C++17                                                                                            |
| 构建系统  | CMake 3.16+                                                                                      |
| 下位机   | STM32；串口设备名因板卡而异（如左 `/dev/ttyS3`、右 `/dev/ttyS9`），**务必** `ls /dev/ttyS`* 后改 `config` 或 `main` 中路径 |
| 单臂自由度 | 6 DOF                                                                                            |
| 控制周期  | 10 ms                                                                                            |


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
│  ┌─────────────┐       │  ttyS* 按硬件接线配置   │     │
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
- `read()` 返回 0（EOF/hangup）时仅记录告警并**继续**监听，避免接收线程一次性退出导致再无 RX

### `trajectory` — 轨迹生成（第三阶段 ✅）

- 单段 `plan(start, end, duration)`：每关节五次多项式，τ 域 3×3 系数用**固定逆矩阵闭式解**（O(1)），无高斯消元
- 生成器线程 **burst**：按 `control_period_ms` 作为时间步长密集算点，全速 `push` 进 `RingBuffer`；池满时短睡退让，**不按 10ms 节拍睡眠**
- 通信线程按 `control_period_ms`（如 10ms）从 `RingBuffer` `pop`，调用 `SerialPort::send_joint_command()` 发往对应臂串口

### `utils/ring_buffer` — SPSC 无锁环形缓冲区

- 单生产者 / 单消费者，`std::atomic` head/tail，无锁
- 容量必须为 2 的幂，用位与替代取模
- `alignas(64)` 将 head/tail 分置独立 cacheline，避免 false sharing
- 支持 `push(T&&)`（移动语义）和 `pop()` 返回 `std::optional<T>`

### `utils/state_machine` — 状态机

合法状态转换关系：

```
INIT → IDLE → RUNNING ⇄ PAUSED
       ↓        ↓
     ERROR ←────┘
       ↓
    SHUTDOWN
```

非法转换会被静默拒绝并返回 `false`，防止系统进入非预期状态。  
其中 `IDLE/RUNNING/PAUSED -> ERROR` 允许业务层在故障场景统一收敛到 `ERROR`。

### `state/arm_state_hub` — ArmStatus 全局状态字典（第四阶段 ✅）

- 消费从机上报 `STATUS_REPORT(0x03)`，按 `arm_id` 反序列化为 `ArmStatus`
- 线程安全保存左右臂最新状态：`try_snapshot()` / `has_valid()` / `last_rx_time()`
- 提供 `set_listener()` 业务回调，可在更新后挂接 FSM、诊断、录包等逻辑
- 在 `main` 中已接入：`error_code != 0` 时尝试触发 `FSM -> ERROR`

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


| 值      | 名称              | 方向  | 说明                              |
| ------ | --------------- | --- | ------------------------------- |
| `0x01` | `HEARTBEAT`     | 主→从 | 心跳，固定周期发送                       |
| `0x02` | `JOINT_CMD`     | 主→从 | 关节指令（含 6 DOF 位置/速度）             |
| `0x03` | `STATUS_REPORT` | 从→主 | 下位机状态上报（关节位置/速度、error_code、时间戳） |
| `0x04` | `ACK`           | 从→主 | 确认应答                            |
| `0x05` | `ERROR_REPORT`  | 从→主 | 故障上报                            |


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
│   ├── state/
│   │   ├── arm_state_hub.hpp
│   │   └── arm_state_hub.cpp # STATUS_REPORT 回收字典 + 业务回调
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

# 运行（需存在对应 tty 设备；无设备时 open 会失败并打日志）
./src/middleware
```

Release 模式构建：

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## 配置文件

程序启动时由 `src/config/config_loader.cpp` 解析 `**config/system.json**`（失败则使用内置默认值，并打印到 `stderr`）。

**解析路径优先级**（先找到者为准）：

1. 环境变量 `**MIDDLEWARE_CONFIG`**（指向任意绝对/相对路径的 JSON 文件）
2. 当前工作目录下的 `config/system.json`
3. `../config/system.json`、`../../config/system.json`（便于从 `build/` 目录运行）
4. 可执行文件相邻目录向上查找的 `config/system.json`

`config/system.json` 控制运行时参数，例如：

```json
{
    "serial_left":  { "port": "/dev/ttyS3", "baud_rate": 115200, "timeout_ms": 100 },
    "serial_right": { "port": "/dev/ttyS9", "baud_rate": 115200, "timeout_ms": 100 },
    "trajectory": {
        "max_velocity": 1.0,
        "max_acceleration": 2.0,
        "buffer_size": 256,
        "control_period_ms": 10.0
    },
    "log_dir": "/var/log/middleware",
    "log_level": 0,
    "watchdog_timeout_ms": 2000,
    "telemetry_stale_ms": 1500
}
```

`log_level`：`0`=DEBUG，`1`=INFO，`2`=WARN，`3`=ERROR。解析成功后 `**Logger::init(log_dir, log_level)**` 才执行，因此日志目录与最低级别来自 JSON。

`telemetry_stale_ms`：遥测新鲜度阈值（ms），超过该时间未收到 `STATUS_REPORT` 判为陈旧/掉线。  
故障升级阈值采用 `max(telemetry_stale_ms, watchdog_timeout_ms)`：先判陈旧/掉线，再达到联动阈值后触发 `FSM -> ERROR`，与看门狗超时策略形成统一故障闭环。

> **依赖**：首次 CMake 配置时会自动下载 `third_party/nlohmann/json.hpp`（离线环境请手动放置该单头文件）。

> **板卡提示**：Orange Pi 5 Plus 等机型常见为 `ttyS3` / `ttyS9`，请按 `ls /dev/ttyS`* 修改 JSON 中的 `port`。

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


| 阶段   | 内容                                                                           | 状态       |
| ---- | ---------------------------------------------------------------------------- | -------- |
| 第一阶段 | 框架搭建：目录结构、CMake、Logger、RingBuffer、StateMachine、Watchdog                      | ✅ 完成     |
| 第二阶段 | 串口通信：`open`/termios、epoll + eventfd、`rx_loop`、`send_frame`、字节流解析状态机、CRC16、心跳 | ✅ **完成** |
| 第三阶段 | 轨迹规划：五次多项式闭式系数、burst 灌池、主线程通信下发 `JOINT_CMD`、载荷与协议对齐                          | ✅ **完成** |
| 第四阶段 | JSON 配置加载（`system.json` → `SystemConfig`）、ArmStatus 回收解析、故障处理闭环              | ✅ 完成     |
| 第五阶段 | 性能调优、集成测试、上板联调                                                               | 待开始      |


