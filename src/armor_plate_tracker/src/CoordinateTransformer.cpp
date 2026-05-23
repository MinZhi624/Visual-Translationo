#include "armor_plate_tracker/CoordinateTransformer.hpp"
#include <cmath>

// 相机坐标系 -> 云台坐标系
// camera: X右 Y下 Z前
// gimbal: X前 Y左 Z上
const Eigen::Matrix3d CoordinateTransformer::R_gimbal_camera_ =
    (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();

const Eigen::Matrix3d CoordinateTransformer::R_camera_gimbal_ = R_gimbal_camera_.transpose();

Eigen::Matrix3d CoordinateTransformer::calcR_world_gimbal(double yaw, double pitch)
{
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    return R_yaw * R_pitch;
}

void CoordinateTransformer::update(double yaw_abs, double pitch_abs)
{
    R_world_gimbal_ = calcR_world_gimbal(yaw_abs, pitch_abs);
    R_gimbal_world_ = R_world_gimbal_.transpose();

    R_world_camera_ = R_world_gimbal_ * R_gimbal_camera_;
    R_camera_world_ = R_world_camera_.transpose();

    q_world_gimbal_ = Eigen::Quaterniond(R_world_gimbal_);
    q_gimbal_camera_ = Eigen::Quaterniond(R_gimbal_camera_);
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

// ========== 角度计算 ==========

Eigen::Vector3d CoordinateTransformer::calcYPR(const Eigen::Quaterniond & q)
{
    // 提取 yaw (Z), pitch (Y), roll (X) — Tait-Bryan ZYX
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    return {yaw, pitch, roll};
}

Eigen::Vector3d CoordinateTransformer::calcYPD(const Eigen::Vector3d & xyz)
{
    double yaw = -std::atan2(xyz.x(), xyz.z());
    double horizontal_dist = std::sqrt(xyz.x() * xyz.x() + xyz.z() * xyz.z());
    double pitch = std::atan2(-xyz.y(), horizontal_dist);
    double distance = xyz.norm();
    return {yaw, pitch, distance};
}

// ========== 更新装甲板 ==========

void CoordinateTransformer::updateTrackerArmor(TrackerArmor & armor) const
{
    if (armor.source_ == TrackerArmor::Source::CAMERA) {
        armor.xyz_world_ = cameraToWorld(armor.xyz_camera_);
        armor.q_world_armor_ = cameraToWorld(armor.q_camera_armor_);
    } else {
        armor.xyz_camera_ = worldToCamera(armor.xyz_world_);
        armor.q_camera_armor_ = worldToCamera(armor.q_world_armor_);
    }
    armor.ypr_camera_ = calcYPR(armor.q_camera_armor_);
    armor.ypr_world_ = calcYPR(armor.q_world_armor_);
    armor.ypd_camera_ = calcYPD(armor.xyz_camera_);
    armor.ypd_world_ = calcYPD(armor.xyz_world_);
}
