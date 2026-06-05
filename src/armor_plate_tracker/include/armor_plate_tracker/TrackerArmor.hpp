#pragma once
#include <armor_plate_interfaces/ArmorTypes.hpp>
#include <armor_plate_common/geometry.hpp>
#include <armor_plate_common/angle.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
        ypd_world_ = armor_plate_common::calculateYPD(xyz_world);
        ypr_world_ = Eigen::Vector3d(
            yaw_world,
            armor_plate_common::degToRad(15.0),
            0.0);
        q_world_armor_ =
            Eigen::AngleAxisd(yaw_world, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(armor_plate_common::degToRad(15.0), Eigen::Vector3d::UnitY());
    }

    // EKF 滤波输出构造（ypda → xyz + yaw）
    TrackerArmor(const Eigen::Vector4d & ypda_world)
        : ypd_world_(ypda_world.head<3>())
    {
        double yaw   = ypda_world[0];
        double pitch = ypda_world[1];
        double dist  = ypda_world[2];
        double angle = ypda_world[3];

        double cos_p = std::cos(pitch);
        xyz_world_.x() = dist * cos_p * std::cos(yaw);
        xyz_world_.y() = dist * cos_p * std::sin(yaw);
        xyz_world_.z() = dist * std::sin(pitch);

        ypr_world_ = Eigen::Vector3d(
            angle,
            armor_plate_common::degToRad(15.0),
            0.0);
        q_world_armor_ =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(armor_plate_common::degToRad(15.0), Eigen::Vector3d::UnitY());
    }
};
