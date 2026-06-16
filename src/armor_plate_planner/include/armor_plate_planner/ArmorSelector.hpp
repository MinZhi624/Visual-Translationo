#pragma once

#include <armor_plate_interfaces/ArmorPose.hpp>
#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <cstddef>

struct PlannerArmor {
    ArmorPose pose;
    size_t index = 0;
};

class ArmorSelector
{
private:
    double max_face_angle_;

public:
    explicit ArmorSelector(double max_face_angle = 1.0472);

    std::optional<PlannerArmor> select(
        const std::vector<PlannerArmor>& armors) const;

    // 计算装甲板正面朝向得分
    double computeFacingScore(
        const Eigen::Vector3d& armor_position,
        double armor_yaw) const;
};
