#include "armor_plate_planner/CommandGenerator.hpp"
#include <armor_plate_common/transform.hpp>
#include <armor_plate_common/geometry.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace apc = armor_plate_common;

GimbalDelta CommandGenerator::generate(
    const geometry_msgs::msg::Point & target_point_world,
    const GimbalData & current_gimbal)
{
    GimbalDelta delta;

    if (!gimbal_received_) {
        return delta;
    }

    if (!std::isfinite(current_gimbal.yaw_abs) || !std::isfinite(current_gimbal.pitch_abs)) {
        return delta;
    }

    Eigen::Vector3d target_world(target_point_world.x, target_point_world.y, target_point_world.z);

    // World -> Gimbal 坐标变换
    Eigen::Matrix3d R_gimbal_world = apc::calculateRGimbalWorld(current_gimbal.yaw_abs, current_gimbal.pitch_abs);
    Eigen::Vector3d target_gimbal = R_gimbal_world * target_world;

    Eigen::Vector3d ypd = apc::calculateYPD(target_gimbal);

    // 输入 finite 检查
    if (!std::isfinite(ypd.x()) || !std::isfinite(ypd.y())) {
        return delta;
    }

    delta.delta_yaw = static_cast<float>(ypd.x());
    delta.delta_pitch = static_cast<float>(ypd.y());

    return delta;
}
