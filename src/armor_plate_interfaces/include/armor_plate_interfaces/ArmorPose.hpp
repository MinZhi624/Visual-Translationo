#pragma once
#include "armor_plate_interfaces/ArmorTypes.hpp"
#include <Eigen/Core>

struct ArmorPose {
    Eigen::Vector3d xyz_world = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    ArmorName name = ArmorName::NONE;
    ArmorType type = ArmorType::SMALL;
};
