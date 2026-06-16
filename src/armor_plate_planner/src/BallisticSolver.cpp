#include "armor_plate_planner/BallisticSolver.hpp"
#include <Eigen/Dense>
#include <cmath>

BallisticSolver::BallisticSolver(double bullet_speed, double gravity)
    : bullet_speed_(bullet_speed), gravity_(gravity)
{
}

BallisticResult BallisticSolver::solve(
    const geometry_msgs::msg::Point & target_point) const
{
    return solveByParabola(target_point);

    // 旧公式备用注释：
    // BallisticResult result;
    // if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) { result.valid = false; return result; }
    // double dx = target_point.x, dy = target_point.y, dz = target_point.z;
    // if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) { result.valid = false; return result; }
    // double horizontal_dist = std::sqrt(dx * dx + dy * dy);
    // if (horizontal_dist < 0.1) { result.valid = false; return result; }
    // double v = bullet_speed_, g = gravity_, v2 = v * v, v4 = v2 * v2;
    // double discriminant = v4 - g * (g * horizontal_dist * horizontal_dist + 2.0 * dz * v2);
    // if (discriminant < 0.0 || !std::isfinite(discriminant)) { result.valid = false; return result; }
    // double tan_pitch = (v2 - std::sqrt(discriminant)) / (g * horizontal_dist);
    // if (!std::isfinite(tan_pitch)) { result.valid = false; return result; }
    // double cos_pitch = 1.0 / std::sqrt(1.0 + tan_pitch * tan_pitch);
    // if (std::abs(cos_pitch) < 1e-6) { result.valid = false; return result; }
    // result.flight_time = horizontal_dist / (v * cos_pitch);
    // if (!std::isfinite(result.flight_time) || result.flight_time <= 0.0) { result.valid = false; return result; }
    // result.compensated_point.x = target_point.x;
    // result.compensated_point.y = target_point.y;
    // result.compensated_point.z = horizontal_dist * tan_pitch;
    // if (!std::isfinite(result.compensated_point.x) || !std::isfinite(result.compensated_point.y) || !std::isfinite(result.compensated_point.z)) { result.valid = false; return result; }
    // result.valid = true;
    // return result;
}

BallisticResult BallisticSolver::solveByParabola(const geometry_msgs::msg::Point & target_point) const
{
    BallisticResult result;

    // 基本参数检查
    if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) {
        result.valid = false;
        return result;
    }

    // 输入 finite 检查 -- 防止上游数据出现问题
    if (!std::isfinite(target_point.x) || !std::isfinite(target_point.y) || !std::isfinite(target_point.z)) {
        result.valid = false;
        return result;
    }

    Eigen::Vector3d target(target_point.x, target_point.y, target_point.z);
    double s = target.head<2>().norm();
    // 水平距离过近检查
    if (s <= 0.1) {
        result.valid = false;
        return result;
    }

    double z = target.z();
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
    result.compensated_point.x = target.x();
    result.compensated_point.y = target.y();
    result.compensated_point.z = s * tan_pitch;

    // 结果 finite 检查
    if (!std::isfinite(result.compensated_point.x) ||
        !std::isfinite(result.compensated_point.y) ||
        !std::isfinite(result.compensated_point.z))
    {
        result.valid = false;
        return result;
    }

    result.valid = true;
    return result;
}