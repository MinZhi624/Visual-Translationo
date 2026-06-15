#include "armor_plate_planner/BallisticSolver.hpp"
#include <cmath>
#include <limits>

BallisticSolver::BallisticSolver(double bullet_speed, double gravity)
    : bullet_speed_(bullet_speed), gravity_(gravity)
{
}

BallisticResult BallisticSolver::solve(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & shooter_origin) const
{
    BallisticResult result;

    // 基本参数检查
    if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) {
        result.valid = false;
        return result;
    }

    double dx = target_point.x - shooter_origin.x;
    double dy = target_point.y - shooter_origin.y;
    double dz = target_point.z - shooter_origin.z;

    // 输入 finite 检查
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
        result.valid = false;
        return result;
    }

    double horizontal_dist = std::sqrt(dx * dx + dy * dy);

    // 水平距离过近检查
    if (horizontal_dist < 0.1) {
        result.valid = false;
        return result;
    }

    double v = bullet_speed_;
    double g = gravity_;
    double v2 = v * v;
    double v4 = v2 * v2;

    // 无阻力低弹道解析解: D = v⁴ - g(g·x² + 2·y·v²)
    double discriminant = v4 - g * (g * horizontal_dist * horizontal_dist + 2.0 * dz * v2);

    if (discriminant < 0.0 || !std::isfinite(discriminant)) {
        result.valid = false;
        return result;
    }

    // 低弹道: tan(pitch) = (v² - sqrt(D)) / (g·x)
    double tan_pitch = (v2 - std::sqrt(discriminant)) / (g * horizontal_dist);

    if (!std::isfinite(tan_pitch)) {
        result.valid = false;
        return result;
    }

    double cos_pitch = 1.0 / std::sqrt(1.0 + tan_pitch * tan_pitch);
    if (std::abs(cos_pitch) < 1e-6) {
        result.valid = false;
        return result;
    }

    result.flight_time = horizontal_dist / (v * cos_pitch);

    // flight_time 检查
    if (!std::isfinite(result.flight_time) || result.flight_time <= 0.0) {
        result.valid = false;
        return result;
    }

    // 虚拟补偿点: 沿射击方向在水平距离处的瞄准高度
    result.compensated_point.x = target_point.x;
    result.compensated_point.y = target_point.y;
    result.compensated_point.z = shooter_origin.z + horizontal_dist * tan_pitch;

    // 结果 finite 检查
    if (!std::isfinite(result.compensated_point.x) ||
        !std::isfinite(result.compensated_point.y) ||
        !std::isfinite(result.compensated_point.z)) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    return result;
}
