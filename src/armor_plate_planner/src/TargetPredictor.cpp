#include "armor_plate_planner/TargetPredictor.hpp"
#include <cmath>

armor_plate_interfaces::msg::TrackedTarget TargetPredictor::predict(
    const armor_plate_interfaces::msg::TrackedTarget & state,
    double dt) const
{
    armor_plate_interfaces::msg::TrackedTarget predicted = state;

    if (dt > 0.0) {
        // 平移目标中心
        predicted.center_world.x += state.center_velocity.x * dt;
        predicted.center_world.y += state.center_velocity.y * dt;
        predicted.center_world.z += state.center_velocity.z * dt;

        // 更新 yaw
        double delta_yaw = state.yaw_rate * dt;
        predicted.yaw += delta_yaw;

        // 刚体外推：每块装甲板先平移再绕新中心旋转 delta_yaw
        double cos_dy = std::cos(delta_yaw);
        double sin_dy = std::sin(delta_yaw);

        for (auto & armor : predicted.armors) {
            // 平移到以旧中心为原点的相对坐标
            double rel_x = armor.position_world.x - state.center_world.x;
            double rel_y = armor.position_world.y - state.center_world.y;

            // XY 平面旋转
            double rot_x = rel_x * cos_dy - rel_y * sin_dy;
            double rot_y = rel_x * sin_dy + rel_y * cos_dy;

            // 平移到新中心
            armor.position_world.x = predicted.center_world.x + rot_x;
            armor.position_world.y = predicted.center_world.y + rot_y;
            armor.position_world.z += state.center_velocity.z * dt;
            armor.yaw_world += delta_yaw;
        }
    }

    return predicted;
}
