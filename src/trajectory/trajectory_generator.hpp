#pragma once

/**
 * trajectory_generator.hpp — 五次多项式关节空间插补 + 无锁队列输出
 *
 * 职责（单一）：
 *   1. 接收首尾两个 JointPoint（同一 arm_id）及段时长 duration_s；
 *   2. 对每个关节独立求解五次多项式系数，使位置/速度/加速度在 t=0 与 t=T 与给定边界一致；
 *   3. 启动后台线程：control_period_ms 仅表示**采样时间步长**（与通信侧节拍一致），
 *      生成器在一次 burst 内全速算点并灌满 RingBuffer，不按该周期睡眠；池满时短暂退让。
 *      通信线程可稳定每 10ms 从池中取点，与「抽水泵」式灌池解耦。
 *
 * 五次多项式（每关节）：
 *   q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵ ,  t ∈ [0, T] , T = duration_s
 *   约束：q(0), q'(0), q''(0), q(T), q'(T), q''(T) 由 start/end 的 position/velocity/acceleration 给出。
 *
 * 说明：
 *   - 系数在内部用 double 计算，写入 JointPoint 时转为 float。
 *   - QueueCapacity 须为 2 的幂（RingBuffer 要求）。
 *   - 与 generator 线程并发调用 plan() 时，内部 mutex 保护系数与段标志。
 */

#include "common_types.hpp"
#include "utils/ring_buffer.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>

// ── 单关节五次多项式系数（对应 q(t)=Σ a_i t^i）────────────────
struct QuinticCoeffs {
    double a[6]{};  // a0 … a5
};

/**
 * @tparam QueueCapacity  RingBuffer 容量，须为 2 的幂（如 256）
 */
template <size_t QueueCapacity>
class TrajectoryGenerator {
    static_assert((QueueCapacity & (QueueCapacity - 1)) == 0,
                  "QueueCapacity must be a power of 2");

public:
    /**
     * @param cfg          轨迹参数（控制周期、限幅等）
     * @param output_queue 由调用方拥有的 SPSC 无锁环形缓冲区；本类仅作为生产者 push
     */
    explicit TrajectoryGenerator(const TrajectoryConfig& cfg,
                                 RingBuffer<JointPoint, QueueCapacity>& output_queue);
    ~TrajectoryGenerator();

    TrajectoryGenerator(const TrajectoryGenerator&) = delete;
    TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;

    /**
     * 规划单段轨迹：start / end 须为同一机械臂（arm_id 一致）。
     * @param duration_s 段时长（秒），必须 > 0
     * @return 边界非法或时长非法时返回 false
     */
    bool plan(const JointPoint& start, const JointPoint& end, double duration_s);

    /** 启动采样线程（仅当未运行时有效） */
    void start();

    /** 停止采样线程并 join */
    void stop();

    [[nodiscard]] bool is_running() const noexcept { return running_.load(std::memory_order_acquire); }

private:
    void generator_loop();

    /** 在当前已规划系数下，于相对时间 t_ms（毫秒）处采样整点 JointPoint */
    JointPoint sample_at(double t_ms) const;

    /** 单关节五次多项式：给定边界与时长 T（秒），求 a0…a5 */
    static QuinticCoeffs solve_quintic(
        double q0, double v0, double a0_init,
        double q1, double v1, double a1_end,
        double T_seconds);

    TrajectoryConfig                           config_;
    RingBuffer<JointPoint, QueueCapacity>&    output_queue_;

    std::array<QuinticCoeffs, DOF> coeffs_{};
    ArmID                          arm_id_{ArmID::LEFT};
    double                         duration_ms_{0.0};
    bool                           segment_pending_{false};

    std::atomic<bool>              running_{false};
    std::thread                    gen_thread_;
    mutable std::mutex             traj_mutex_;
};

// 显式实例化声明（见 trajectory_generator.cpp），便于其它翻译单元链接
extern template class TrajectoryGenerator<256>;
