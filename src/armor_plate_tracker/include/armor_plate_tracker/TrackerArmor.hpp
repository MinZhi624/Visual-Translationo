#pragma once
#include <armor_plate_interfaces/ArmorTypes.hpp>
#include <armor_plate_interfaces/armor_geometry.hpp>
#include <armor_plate_common/geometry.hpp>
#include <armor_plate_common/angle.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apc = armor_plate_common;
using armor_plate_interfaces::ARMOR_PITCH_RAD;

class TrackerArmor
{
public:
    ArmorName armor_name = ArmorName::NONE;

    Eigen::Vector3d xyz_world_{0, 0, 0};
    float image_distance_to_center = 0.0f;
    Eigen::Vector3d ypr_world_{0, 0, 0};
    Eigen::Vector3d ypd_world_{0, 0, 0};
    Eigen::Vector3d ypd_gimbal_{0, 0, 0};

    Eigen::Quaterniond q_world_armor_{1, 0, 0, 0};

    TrackerArmor() = default;

    // 世界坐标系构造（Identification → Tracker 入口）
    TrackerArmor(const Eigen::Vector3d & xyz_world, double yaw_world, float image_dist = 0.0f)
        : xyz_world_(xyz_world), image_distance_to_center(image_dist)
    {
        ypd_world_ = apc::calculateYPD(xyz_world);
        ypr_world_ = Eigen::Vector3d(
            yaw_world,
            ARMOR_PITCH_RAD,
            0.0);
        q_world_armor_ =
            Eigen::AngleAxisd(yaw_world, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ARMOR_PITCH_RAD, Eigen::Vector3d::UnitY());
    }

    // EKF 滤波输出构造（ypda → xyz + yaw）
    TrackerArmor(const Eigen::Vector4d & ypda_world)
        : ypd_world_(ypda_world.head<3>())
    {
        double angle = ypda_world[3];

        xyz_world_ = apc::calculateXYZ(ypd_world_);

        ypr_world_ = Eigen::Vector3d(
            angle,
            ARMOR_PITCH_RAD,
            0.0);
        q_world_armor_ =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ARMOR_PITCH_RAD, Eigen::Vector3d::UnitY());
    }
};
