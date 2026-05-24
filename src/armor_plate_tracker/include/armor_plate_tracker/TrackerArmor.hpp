#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

class TrackerArmor
{
public:
    enum class Source { CAMERA, WORLD };
    Source source_;
    int id = -1;
    float image_distance_to_center = 0.0f;

    Eigen::Vector3d xyz_camera_{0, 0, 0};
    Eigen::Vector3d xyz_world_{0, 0, 0};

    Eigen::Vector3d ypr_camera_{0, 0, 0};
    Eigen::Vector3d ypr_world_{0, 0, 0};

    Eigen::Vector3d ypd_camera_{0, 0, 0};
    Eigen::Vector3d ypd_world_{0, 0, 0};

    Eigen::Quaterniond q_camera_armor_{1, 0, 0, 0};
    Eigen::Quaterniond q_world_armor_{1, 0, 0, 0};

    TrackerArmor() = default;

    // 相机系数据的构造（原始检测）
    TrackerArmor(const Eigen::Vector3d & xyz_camera, const Eigen::Quaterniond & q_camera_armor)
        : source_(Source::CAMERA), xyz_camera_(xyz_camera), q_camera_armor_(q_camera_armor)
    {
    }

    // 世界坐标系构造（滤波数据：位置 + yaw）
    TrackerArmor(const Eigen::Vector3d & xyz_world, double yaw)
        : source_(Source::WORLD), xyz_world_(xyz_world),
          q_world_armor_(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
    {
    }
};
