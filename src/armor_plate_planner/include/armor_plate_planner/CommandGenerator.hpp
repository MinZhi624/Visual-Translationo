#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <armor_plate_interfaces/msg/gimbal_angle.hpp>

struct GimbalDelta
{
    float delta_yaw = 0.0f;
    float delta_pitch = 0.0f;
};

class CommandGenerator
{
private:
    bool gimbal_received_ = false;

public:
    CommandGenerator() = default;

    void setGimbalReceived(bool received) { gimbal_received_ = received; }

    GimbalDelta generate(
        const geometry_msgs::msg::Point & target_point_world,
        const geometry_msgs::msg::Point & shooter_origin,
        const armor_plate_interfaces::msg::GimbalAngle & current_gimbal);
};
