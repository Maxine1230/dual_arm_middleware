#pragma once

/**
 * trajectory_generator.hpp  —  五次多项式轨迹生成（第三阶段主角）
 *
 * 当前阶段：接口定义 + 数学背景注释
 *
 * 五次多项式：
 *   q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
 * 约束条件：
 *   q(0)=q0,  q(T)=q1
 *   dq(0)=v0, dq(T)=v1
 *   ddq(0)=a0_init, ddq(T)=a1_final
 *
 * 面试考点：
 *   Q: 为什么用五次多项式而不是三次？
 *   A: 五次多项式可以同时约束初末位置/速度/加速度，
 *      保证加速度连续，避免关节冲击（jerk 有限）。
 */

#include "common_types.hpp"
#include <vector>
#include <array>
#include <functional>
#include <atomic>
#include <thread>

// 单关节五次多项式系数
struct QuinticCoeffs {
    double a[6];  // a0..a5
    double duration; // T (s)
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator(const TrajectoryConfig& cfg);
    ~TrajectoryGenerator();

    // 规划从 start → end 的轨迹（阻塞，但快，~微秒级）
    bool plan(const JointPoint& start, const JointPoint& end, double duration_s);

    // 启动轨迹推送线程（以 cfg.control_period_ms 为周期采样 → 压入 ring_buffer）
    void start();
    void stop();

    // 注册推送回调（推给通信线程）
    using PointCallback = std::function<void(const JointPoint&)>;
    void set_point_callback(PointCallback cb);

    bool is_running() const { return running_; }

private:
    void generator_loop();
    JointPoint sample(double t_ms) const;  // 在当前轨迹上采样

    // 单关节五次多项式求解
    static QuinticCoeffs solve_quintic(
        double q0, double v0, double a0,
        double q1, double v1, double a1,
        double T);

    TrajectoryConfig    config_;
    std::atomic<bool>   running_{false};
    std::thread         gen_thread_;

    PointCallback       point_cb_;

    // 当前轨迹系数（第三阶段：加锁或双缓冲）
    std::array<QuinticCoeffs, DOF> coeffs_left_;
    std::array<QuinticCoeffs, DOF> coeffs_right_;
};
