#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace armor_plate_common
{

// camera: X右 Y下 Z前
// gimbal/world base: X前 Y左 Z上
inline const Eigen::Matrix3d R_GIMBAL_CAMERA =
    (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();

inline const Eigen::Matrix3d R_CAMERA_GIMBAL = R_GIMBAL_CAMERA.transpose();

inline Eigen::Matrix3d calculateRWorldGimbal(double yaw, double pitch)
{
    const Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return R_yaw * R_pitch;
}

// 计算从世界系到云台系的旋转矩阵
inline Eigen::Matrix3d calculateRGimbalWorld(double yaw, double pitch)
{
    return calculateRWorldGimbal(yaw, pitch).transpose();
}

}  // namespace armor_plate_common
