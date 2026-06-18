#include "armor_plate_planner/BallisticSolver.hpp"
#include <Eigen/Dense>
#include <cmath>

BallisticSolver::BallisticSolver(double bullet_speed, double gravity, double drag_coeff)
    : bullet_speed_(bullet_speed), gravity_(gravity), drag_coeff_(drag_coeff)
{
}

BallisticResult BallisticSolver::solve(const Eigen::Vector3d& target_point) const
{
    return solveByDirectDrag(target_point);
}

BallisticResult BallisticSolver::solveByDirectDrag(const Eigen::Vector3d& target_point) const
{
    BallisticResult result;

    // 基本参数检查
    if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) {
        result.valid = false;
        return result;
    }

    // 输入 finite 检查
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
    double g = gravity_;
    double k = drag_coeff_;

    // B = (e^(ks) - 1) / k, k=0 时 B=s
    double B;
    if (std::abs(k) < 1e-10) {
        B = s;
    } else {
        B = (std::exp(k * s) - 1.0) / k;
    }

    // C = gB² / (2v²)
    double B2 = B * B;
    double v2 = v * v;
    double C = g * B2 / (2.0 * v2);

    // D = B² - 4C(C + z)
    double D = B2 - 4.0 * C * (C + z);
    if (D < 0.0 || !std::isfinite(D)) {
        result.valid = false;
        return result;
    }

    double sqrt_D = std::sqrt(D);

    // 低弹道解：tan(θ) = 2(C + z) / (B + √D)
    double tan_pitch = 2.0 * (C + z) / (B + sqrt_D);
    double pitch = std::atan2(tan_pitch, 1.0);

    // flight_time = B / (v * cos(θ))
    double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 1e-6) {
        result.valid = false;
        return result;
    }
    result.flight_time = B / (v * cos_pitch);

    if (!std::isfinite(result.flight_time) || result.flight_time <= 0.0) {
        result.valid = false;
        return result;
    }

    // 补偿点：沿射击方向在水平距离处的瞄准高度
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

BallisticResult BallisticSolver::solveByNewtonDrag(const Eigen::Vector3d& target_point) const
{
    BallisticResult result;

    // 基本参数检查
    if (bullet_speed_ <= 0.0 || gravity_ <= 0.0) {
        result.valid = false;
        return result;
    }

    if (!target_point.allFinite()) {
        result.valid = false;
        return result;
    }

    double s = target_point.head<2>().norm();
    if (s <= 0.1) {
        result.valid = false;
        return result;
    }

    double z = target_point.z();
    double v = bullet_speed_;
    double g = gravity_;
    double k = drag_coeff_;

    // B = (e^(ks) - 1) / k, k=0 时 B=s
    double B;
    if (std::abs(k) < 1e-10) {
        B = s;
    } else {
        B = (std::exp(k * s) - 1.0) / k;
    }

    // 初始猜测
    double pitch = std::atan2(B, v);
    double gB2v2 = g * B * B / (v * v);

    // 牛顿迭代
    constexpr int MAX_ITER = 100;
    constexpr double EPSILON = 1e-10;

    for (int i = 0; i < MAX_ITER; i++) {
        double cos_t = std::cos(pitch);
        if (std::abs(cos_t) < 1e-10) {
            result.valid = false;
            return result;
        }

        double tan_t = std::tan(pitch);
        double sec2 = 1.0 / (cos_t * cos_t);

        // 命中高度
        double hit_z = B * tan_t - 0.5 * gB2v2 * sec2;
        // 残差
        double residual = z - hit_z;

        // 收敛检查
        if (std::abs(residual) < EPSILON) break;

        // 导数
        double derivative = -B * sec2 + gB2v2 * sec2 * tan_t;
        if (std::abs(derivative) < 1e-10) {
            result.valid = false;
            return result;
        }

        // 牛顿更新
        pitch -= residual / derivative;
    }

    // flight_time = B / (v * cos(θ))
    double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 1e-6) {
        result.valid = false;
        return result;
    }
    result.flight_time = B / (v * cos_pitch);

    if (!std::isfinite(result.flight_time) || result.flight_time <= 0.0) {
        result.valid = false;
        return result;
    }

    double tan_pitch = std::tan(pitch);

    // 补偿点
    result.compensated_point.x() = target_point.x();
    result.compensated_point.y() = target_point.y();
    result.compensated_point.z() = s * tan_pitch;

    if (!result.compensated_point.allFinite()) {
        result.valid = false;
        return result;
    }

    result.valid = true;
    return result;
}
