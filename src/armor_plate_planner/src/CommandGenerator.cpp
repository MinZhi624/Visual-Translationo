#include "armor_plate_planner/CommandGenerator.hpp"
#include <cmath>

GimbalDelta CommandGenerator::generate(
    const geometry_msgs::msg::Point & target_point_world,
    const geometry_msgs::msg::Point & shooter_origin,
    const armor_plate_interfaces::msg::GimbalAngle & current_gimbal)
{
    GimbalDelta delta;

    if (!gimbal_received_) {
        return delta;
    }

    if (!std::isfinite(current_gimbal.yaw_abs) || !std::isfinite(current_gimbal.pitch_abs)) {
        return delta;
    }

    double dx = target_point_world.x - shooter_origin.x;
    double dy = target_point_world.y - shooter_origin.y;
    double dz = target_point_world.z - shooter_origin.z;

    // 输入 finite 检查
    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
        return delta;
    }

    double horizontal_dist = std::sqrt(dx * dx + dy * dy);

    float target_yaw = static_cast<float>(std::atan2(dy, dx));
    float target_pitch = static_cast<float>(std::atan2(dz, horizontal_dist));

    delta.delta_yaw = target_yaw - current_gimbal.yaw_abs;
    delta.delta_pitch = target_pitch - current_gimbal.pitch_abs;

    while (delta.delta_yaw > static_cast<float>(M_PI)) {
        delta.delta_yaw -= 2.0f * static_cast<float>(M_PI);
    }
    while (delta.delta_yaw < -static_cast<float>(M_PI)) {
        delta.delta_yaw += 2.0f * static_cast<float>(M_PI);
    }

    if (!std::isfinite(delta.delta_yaw) || !std::isfinite(delta.delta_pitch)) {
        delta.delta_yaw = 0.0f;
        delta.delta_pitch = 0.0f;
    }

    return delta;
}
