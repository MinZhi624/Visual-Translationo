#include "armor_plate_planner/BallisticSolver.hpp"
#include <Eigen/Dense>
#include <cmath>

BallisticSolver::BallisticSolver(double bullet_speed, double gravity)
    : bullet_speed_(bullet_speed), gravity_(gravity)
{
}

BallisticResult BallisticSolver::solve(
    const Eigen::Vector3d& target_point) const
{
    return solveByParabola(target_point);
}

BallisticResult BallisticSolver::solveByParabola(const Eigen::Vector3d& target_point) const
{
    BallisticResult result;

    // 基本参数检查
    if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) {
        result.valid = false;
        return result;
    }

    // 输入 finite 检查 -- 防止上游数据出现问题
    if (!target_point.allFinite()) {
        result.valid = false;
        return result;
    }

    double s = target_point.head<2>().norm();
    // 水平距离过近检查
    if (s <= 0.1) {
        result.valid = false;
        return result;
    }

    double z = target_point.z();
    double v = bullet_speed_;
    double v2 = v * v;
    double g = gravity_;
    double s2 = s * s;

    // A = gs^2 / 2v^2
    double A = g * s2 / (2.0 * v2);

    // D = s^2 - 4A(A + z)
    double D = s2 - 4.0 * A * (A + z);
    if (D < 0.0 || !std::isfinite(D)) {
        result.valid = false;
        return result;
    }

    double sqrt_D = std::sqrt(D);

    // 稳定低弹道解：tan(pitch) = 2(A + z) / (s + sqrt(D))
    double tan_pitch = 2.0 * (A + z) / (s + sqrt_D);
    double pitch = std::atan2(tan_pitch, 1.0);

    // flight_time 检查
    double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 1e-6) {
        result.valid = false;
        return result;
    }
    result.flight_time = s / (v * cos_pitch);

    if (!std::isfinite(result.flight_time) || result.flight_time <= 0.0) {
        result.valid = false;
        return result;
    }

    // 虚拟补偿点: 沿射击方向在水平距离处的瞄准高度
    result.compensated_point.x() = target_point.x();
    result.compensated_point.y() = target_point.y();
    result.compensated_point.z() = s * tan_pitch;

    // 结果 finite 检查
    if (!result.compensated_point.allFinite()) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    return result;
}
