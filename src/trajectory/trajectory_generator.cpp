#include "trajectory_generator.hpp"
#include "../logger/logger.hpp"

#include <chrono>
#include <cmath>

namespace {

// q(t) = Σ a_i t^i , t 单位为秒
void eval_quintic(const QuinticCoeffs& c, double t, double& q, double& qd, double& qdd) {
    const double a0 = c.a[0], a1 = c.a[1], a2 = c.a[2];
    const double a3 = c.a[3], a4 = c.a[4], a5 = c.a[5];
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    q   = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    qd  = a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4;
    qdd = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3;
}

// ── 归一化时间 τ = t/T 下的「上帝公式」──────────────────────────────
//
// 令 q̃(τ) = Σ b_i τ^i ，τ∈[0,1]，则 q(t) = q̃(t/T) ⇒ a_i = b_i / T^i 。
// 在 τ 域，未知量 (b3,b4,b5) 的系数矩阵为常数：
//   [ 1   1   1  ] [b3]   [rhs_q]
//   [ 3   4   5  ] [b4] = [rhs_v]
//   [ 6  12  20  ] [b5]   [rhs_a]
// det = 2 ，逆矩阵为闭式（一次乘加，O(1)）：
//   M⁻¹ = ½ · [ 20  -8   1 ]
//            [-30  14  -2 ]
//            [ 12  -6   1 ]
//
// rhs_q = q1 - b0 - b1 - b2
// rhs_v = v1·T - b1 - 2·b2
// rhs_a = a1_end·T² - 2·b2
// 其中 b0=q0, b1=v0·T, b2=(a0_init/2)·T² 。

JointPoint build_joint_point(const std::array<QuinticCoeffs, DOF>& coeff,
                             ArmID aid,
                             double t_ms)
{
    JointPoint pt{};
    pt.arm_id = aid;
    const double t_sec = t_ms * 1e-3;
    for (int j = 0; j < DOF; ++j) {
        double q, qd, qdd;
        eval_quintic(coeff[static_cast<size_t>(j)], t_sec, q, qd, qdd);
        pt.position[j]     = static_cast<float>(q);
        pt.velocity[j]     = static_cast<float>(qd);
        pt.acceleration[j] = static_cast<float>(qdd);
    }
    pt.timestamp_ms = static_cast<float>(t_ms);
    return pt;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
//  TrajectoryGenerator
// ═════════════════════════════════════════════════════════════════════════════

template <size_t QueueCapacity>
TrajectoryGenerator<QueueCapacity>::TrajectoryGenerator(
    const TrajectoryConfig& cfg,
    RingBuffer<JointPoint, QueueCapacity>& output_queue)
    : config_(cfg)
    , output_queue_(output_queue)
{}

template <size_t QueueCapacity>
TrajectoryGenerator<QueueCapacity>::~TrajectoryGenerator() {
    stop();
}

template <size_t QueueCapacity>
bool TrajectoryGenerator<QueueCapacity>::plan(
    const JointPoint& start, const JointPoint& end, double duration_s)
{
    if (start.arm_id != end.arm_id) {
        LOG_WARN("traj", "plan: start/end arm_id mismatch");
        return false;
    }
    if (duration_s <= 0.0 || !std::isfinite(duration_s)) {
        LOG_WARN("traj", "plan: invalid duration_s");
        return false;
    }

    std::lock_guard<std::mutex> lock(traj_mutex_);

    arm_id_ = start.arm_id;
    duration_ms_ = duration_s * 1000.0;

    for (int j = 0; j < DOF; ++j) {
        coeffs_[static_cast<size_t>(j)] = solve_quintic(
            static_cast<double>(start.position[j]),
            static_cast<double>(start.velocity[j]),
            static_cast<double>(start.acceleration[j]),
            static_cast<double>(end.position[j]),
            static_cast<double>(end.velocity[j]),
            static_cast<double>(end.acceleration[j]),
            duration_s);
    }

    segment_pending_ = true;
    LOG_INFO("traj", "plan: segment armed, duration=" + std::to_string(duration_s) + " s");
    return true;
}

template <size_t QueueCapacity>
void TrajectoryGenerator<QueueCapacity>::start() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }
    gen_thread_ = std::thread(&TrajectoryGenerator::generator_loop, this);
    LOG_INFO("traj", "TrajectoryGenerator started");
}

template <size_t QueueCapacity>
void TrajectoryGenerator<QueueCapacity>::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (gen_thread_.joinable()) {
        gen_thread_.join();
    }
    LOG_INFO("traj", "TrajectoryGenerator stopped");
}

template <size_t QueueCapacity>
void TrajectoryGenerator<QueueCapacity>::generator_loop() {
    using namespace std::chrono;

    const double dt_ms = config_.control_period_ms;
    if (dt_ms <= 0.0) {
        LOG_ERROR("traj", "control_period_ms must be > 0");
        return;
    }

    while (running_.load(std::memory_order_acquire)) {
        std::array<QuinticCoeffs, DOF> local{};
        double                       T_ms = 0.0;
        ArmID                        aid  = ArmID::LEFT;
        bool                         has_segment = false;

        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            if (segment_pending_) {
                local         = coeffs_;
                T_ms          = duration_ms_;
                aid           = arm_id_;
                segment_pending_ = false;
                has_segment   = true;
            }
        }

        if (!has_segment) {
            // 无新段：水池已灌完，生成器休息，等下一次 plan()
            std::this_thread::sleep_for(milliseconds(1));
            continue;
        }

        // ── Burst 灌池：按 dt_ms 在时间上离散，但计算/推送全速进行，不按 10ms 节拍睡眠 ──
        for (double t_ms = 0.0; t_ms <= T_ms + 1e-6 && running_.load(std::memory_order_acquire);
             t_ms += dt_ms) {
            const JointPoint pt = build_joint_point(local, aid, t_ms);

            while (running_.load(std::memory_order_acquire)) {
                if (output_queue_.push(pt)) {
                    break;  // 入队成功，下一时刻
                }
                // RingBuffer 满：消费者尚未抽走，短暂退让避免空转占满 CPU
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
        // 本段所有点已入队（或 running 已 false）：回到外层，无新 plan 则 sleep
    }
}

template <size_t QueueCapacity>
JointPoint TrajectoryGenerator<QueueCapacity>::sample_at(double t_ms) const {
    return build_joint_point(coeffs_, arm_id_, t_ms);
}

template <size_t QueueCapacity>
QuinticCoeffs TrajectoryGenerator<QueueCapacity>::solve_quintic(
    double q0, double v0, double a0_init,
    double q1, double v1, double a1_end,
    double T)
{
    QuinticCoeffs c{};
    if (T <= 1e-9 || !std::isfinite(T)) {
        c.a[0] = q0;
        return c;
    }

    const double T2 = T * T;

    // τ 域前 3 个系数（与 t 域关系：a_i = b_i / T^i）
    const double b0 = q0;
    const double b1 = v0 * T;
    const double b2 = 0.5 * a0_init * T2;

    const double rhs_q = q1 - b0 - b1 - b2;
    const double rhs_v = v1 * T - b1 - 2.0 * b2;
    const double rhs_a = a1_end * T2 - 2.0 * b2;

    // M⁻¹ · rhs ，M 为 [1,1,1; 3,4,5; 6,12,20]，det=2
    const double b3 = 0.5 * (20.0 * rhs_q - 8.0 * rhs_v + rhs_a);
    const double b4 = 0.5 * (-30.0 * rhs_q + 14.0 * rhs_v - 2.0 * rhs_a);
    const double b5 = 0.5 * (12.0 * rhs_q - 6.0 * rhs_v + rhs_a);

    c.a[0] = b0;
    c.a[1] = b1 / T;
    c.a[2] = b2 / T2;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    c.a[3] = b3 / T3;
    c.a[4] = b4 / T4;
    c.a[5] = b5 / T5;
    return c;
}

template class TrajectoryGenerator<256>;
