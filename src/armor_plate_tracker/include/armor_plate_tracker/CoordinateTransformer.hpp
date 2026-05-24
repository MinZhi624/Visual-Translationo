#pragma once
#include "TrackerArmor.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

class CoordinateTransformer
{
public:
    CoordinateTransformer() = default;

    void update(double yaw_abs, double pitch_abs);

    Eigen::Vector3d cameraToWorld(const Eigen::Vector3d & xyz) const;
    Eigen::Vector3d worldToCamera(const Eigen::Vector3d & xyz) const;
    Eigen::Quaterniond cameraToWorld(const Eigen::Quaterniond & q) const;
    Eigen::Quaterniond worldToCamera(const Eigen::Quaterniond & q) const;

    static Eigen::Vector3d calcYPR(const Eigen::Quaterniond & q);
    static Eigen::Vector3d calcYPD(const Eigen::Vector3d & xyz);

    void updateTrackerArmor(TrackerArmor & armor) const;
private:
    static const Eigen::Matrix3d R_gimbal_camera_;
    static const Eigen::Matrix3d R_camera_gimbal_;

    Eigen::Matrix3d R_world_gimbal_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_gimbal_world_{Eigen::Matrix3d::Identity()};

    Eigen::Matrix3d R_world_camera_{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_camera_world_{Eigen::Matrix3d::Identity()};

    Eigen::Quaterniond q_world_gimbal_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond q_gimbal_camera_{Eigen::Quaterniond::Identity()};

    static Eigen::Matrix3d calcR_world_gimbal(double yaw, double pitch);
};
