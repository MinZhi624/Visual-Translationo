#include "armor_plate_tracker/CoordinateTransformer.hpp"
#include "armor_plate_common/geometry.hpp"
#include "armor_plate_common/transform.hpp"

void CoordinateTransformer::update(const GimbalData & gimbal)
{
    R_world_gimbal_ = armor_plate_common::calculateRWorldGimbal(gimbal.yaw_abs, gimbal.pitch_abs);

    R_world_camera_ = R_world_gimbal_ * armor_plate_common::R_GIMBAL_CAMERA;
    R_camera_world_ = R_world_camera_.transpose();

    q_world_gimbal_ = Eigen::Quaterniond(R_world_gimbal_);
    q_gimbal_camera_ = Eigen::Quaterniond(armor_plate_common::R_GIMBAL_CAMERA);
}

// ========== 坐标变换 ==========

Eigen::Vector3d CoordinateTransformer::cameraToWorld(const Eigen::Vector3d & xyz) const
{
    return R_world_camera_ * xyz;
}

Eigen::Vector3d CoordinateTransformer::worldToCamera(const Eigen::Vector3d & xyz) const
{
    return R_camera_world_ * xyz;
}

Eigen::Quaterniond CoordinateTransformer::cameraToWorld(const Eigen::Quaterniond & q) const
{
    return q_world_gimbal_ * q_gimbal_camera_ * q;
}

Eigen::Quaterniond CoordinateTransformer::worldToCamera(const Eigen::Quaterniond & q) const
{
    return q_gimbal_camera_.conjugate() * q_world_gimbal_.conjugate() * q;
}

// ========== 更新装甲板 ==========

void CoordinateTransformer::updateTrackerArmor(TrackerArmor & armor) const
{
    armor.ypd_gimbal_ = armor_plate_common::calculateYPD(
        armor_plate_common::R_GIMBAL_CAMERA * R_camera_world_ * armor.xyz_world_);
}
