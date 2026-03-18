项目框架总览
面向嵌入式 Linux 的双臂控制中间件骨架，当前处于“架构搭建完成 + 核心功能分阶段填充”的阶段。

整体分层可以概括为：

上层规划/算法(未来RL)
→ TrajectoryGenerator（轨迹规划/采样）
→ RingBuffer<SPSC>（跨线程解耦）
→ SerialPort（协议封装/串口收发）
→ STM32 + 伺服执行机构

代码模块与职责
src/main.cpp：系统编排入口，负责初始化、线程启动、状态切换、优雅退出。
src/common_types.hpp：核心数据契约（JointPoint、RawFrame、ArmStatus、SystemConfig 等）。
src/trajectory/：轨迹模块，定义了五次多项式轨迹接口（plan/sample/solve_quintic）。
src/comm/：通信模块，定义串口协议与收发流程（SerialPort、CRC、帧构建）。
src/utils/ring_buffer.hpp：单生产者单消费者无锁环形缓冲，做实时链路解耦。
src/utils/state_machine.*：系统状态机，限制合法状态迁移。
src/watchdog/：线程级看门狗，超时回调触发错误态。
src/logger/：异步日志单例，控制线程只入队，IO在线程中处理。
config/system.json：配置样例存在，但当前 main 仍是硬编码配置。
tests/：已有 ring_buffer / logger / state_machine 的基础测试程序。