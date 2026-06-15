#pragma once

#include <geometry_msgs/msg/point.hpp>

struct BallisticResult
{
    geometry_msgs::msg::Point compensated_point;
    double flight_time = 0.0;
    bool valid = false;
};

class BallisticSolver
{
private:
    double bullet_speed_;
    double gravity_;

public:
    BallisticSolver(double bullet_speed = 25.0, double gravity = 9.81);

    BallisticResult solve(
        const geometry_msgs::msg::Point & target_point,
        const geometry_msgs::msg::Point & shooter_origin) const;
};
