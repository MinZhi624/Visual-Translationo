#pragma once

#include <Eigen/Core>

struct BallisticResult
{
    Eigen::Vector3d compensated_point = Eigen::Vector3d::Zero();
    double flight_time = 0.0;
    bool valid = false;
};

class BallisticSolver
{
private:
    double bullet_speed_;
    double gravity_;
    double drag_coeff_;  // 空气阻力系数 k = 0.5*ρ*A/m

    // 有阻力解析法（主要方法）
    BallisticResult solveByDirectDrag(const Eigen::Vector3d& target_point) const;
    // 有阻力牛顿迭代法（备用方法）
    BallisticResult solveByNewtonDrag(const Eigen::Vector3d& target_point) const;

public:
    BallisticSolver(double bullet_speed = 25.0, double gravity = 9.81, double drag_coeff = 0.0);

    BallisticResult solve(const Eigen::Vector3d& target_point) const;
};
