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

    // 抛物线弹道解（无阻力低弹道）
    BallisticResult solveByParabola(const Eigen::Vector3d& target_point) const;

public:
    BallisticSolver(double bullet_speed = 25.0, double gravity = 9.81);

    BallisticResult solve(const Eigen::Vector3d& target_point) const;
};
