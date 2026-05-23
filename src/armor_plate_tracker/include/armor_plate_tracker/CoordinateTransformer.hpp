#pragma once
#include "TrackerArmor.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

class CoordinateTransformer
{
public:
    CoordinateTransformer() = default;

    // 每帧更新云台角度
    void update(double yaw_abs, double pitch_abs);

    // ========== 坐标变换 ==========
    Eigen::Vector3d cameraToWorld(const Eigen::Vector3d & xyz) const;
    Eigen::Vector3d worldToCamera(const Eigen::Vector3d & xyz) const;
    Eigen::Quaterniond cameraToWorld(const Eigen::Quaterniond & q) const;
    Eigen::Quaterniond worldToCamera(const Eigen::Quaterniond & q) const;

    // ========== 角度计算（统一入口） ==========

    // 从四元数提取 yaw/pitch/roll
    static Eigen::Vector3d calcYPR(const Eigen::Quaterniond & q);

    // 从位置向量计算指向角: (yaw, pitch, distance)
    static Eigen::Vector3d calcYPD(const Eigen::Vector3d & xyz);

    // ========== 更新装甲板（自动检测来源，补全另一侧数据） ==========
    void updateTrackerArmor(TrackerArmor & armor) const;

private:
    // 相机 -> 云台（固定）
    static const Eigen::Matrix3d R_gimbal_camera_;
    static const Eigen::Matrix3d R_camera_gimbal_;

    // 云台 -> 世界（每帧更新）
    Eigen::Matrix3d R_world_gimbal_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_gimbal_world_{Eigen::Matrix3d::Identity()};

    // 组合矩阵
    Eigen::Matrix3d R_world_camera_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_camera_world_{Eigen::Matrix3d::Identity()};

    // 四元数版本
    Eigen::Quaterniond q_world_gimbal_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond q_gimbal_camera_{Eigen::Quaterniond::Identity()};

    // 计算云台->世界旋转矩阵
    static Eigen::Matrix3d calcR_world_gimbal(double yaw, double pitch);
};
