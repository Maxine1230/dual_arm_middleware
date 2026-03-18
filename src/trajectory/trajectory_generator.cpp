#include "trajectory_generator.hpp"
#include "../logger/logger.hpp"

#include <cmath>
#include <chrono>
#include <thread>

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryConfig& cfg)
    : config_(cfg)
{}

TrajectoryGenerator::~TrajectoryGenerator() {
    stop();
}

bool TrajectoryGenerator::plan(const JointPoint& /*start*/,
                                const JointPoint& /*end*/,
                                double /*duration_s*/) {
    // TODO (第三阶段): 调用 solve_quintic() 填充 coeffs_left_/right_
    LOG_WARN("traj", "plan() - stub");
    return false;
}

void TrajectoryGenerator::start() {
    if (running_) return;
    running_    = true;
    gen_thread_ = std::thread(&TrajectoryGenerator::generator_loop, this);
    LOG_INFO("traj", "TrajectoryGenerator started");
}

void TrajectoryGenerator::stop() {
    if (!running_) return;
    running_ = false;
    if (gen_thread_.joinable()) gen_thread_.join();
    LOG_INFO("traj", "TrajectoryGenerator stopped");
}

void TrajectoryGenerator::set_point_callback(PointCallback cb) {
    point_cb_ = std::move(cb);
}

void TrajectoryGenerator::generator_loop() {
    using namespace std::chrono;
    auto period = milliseconds(static_cast<int>(config_.control_period_ms));
    auto next   = steady_clock::now();

    while (running_) {
        next += period;

        // TODO (第三阶段): sample(t) → point_cb_()
        // 当前只打印心跳，证明线程在跑
        LOG_DEBUG("traj", "generator tick");

        std::this_thread::sleep_until(next);
    }
}

JointPoint TrajectoryGenerator::sample(double /*t_ms*/) const {
    // TODO (第三阶段): 代入五次多项式公式
    return JointPoint{};
}

QuinticCoeffs TrajectoryGenerator::solve_quintic(
    double q0, double v0, double a0_val,
    double q1, double v1, double a1_val,
    double T)
{
    /**
     * TODO (第三阶段): 求解 6x6 线性方程组
     * 可直接用 Eigen 的 Matrix6d 和 colPivHouseholderQr()
     * 也可手推公式（推荐，速度快，面试能手写）
     */
    (void)q0; (void)v0; (void)a0_val;
    (void)q1; (void)v1; (void)a1_val;
    (void)T;

    QuinticCoeffs c{};
    c.duration = T;
    return c;
}
