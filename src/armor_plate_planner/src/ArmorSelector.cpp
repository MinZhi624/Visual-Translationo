#include "armor_plate_planner/ArmorSelector.hpp"
#include <armor_plate_interfaces/GimbalData.hpp>
#include <armor_plate_common/geometry.hpp>
#include <armor_plate_common/angle.hpp>
#include <cmath>

namespace apc = armor_plate_common;

ArmorSelector::ArmorSelector(double max_face_angle)
    : max_face_angle_(max_face_angle)
{
}

double ArmorSelector::computeFacingScore(
    const geometry_msgs::msg::Point & armor_position,
    double armor_yaw) const
{
    // 计算射手→装甲板方向角 yaw_to_armor
    Eigen::Vector3d armor_xyz(armor_position.x, armor_position.y, armor_position.z);
    double yaw_to_armor = apc::calculateYPD(armor_xyz).x();

    // 朝向差: 装甲板法线与射手方向的夹角
    double delta_angle = apc::normalizeRadAngle(armor_yaw - yaw_to_armor);

    // cos(delta_angle): 越正对射手得分越高（最大 1，最小 -1）
    return std::cos(delta_angle);
}

std::optional<armor_plate_interfaces::msg::TrackedArmor> ArmorSelector::select(
    const std::vector<armor_plate_interfaces::msg::TrackedArmor> & armors) const
{
    if (armors.empty()) return std::nullopt;

    double best_score = -2.0;
    size_t best_idx = 0;
    bool found = false;

    for (size_t i = 0; i < armors.size(); ++i) {
        double score = computeFacingScore(armors[i].position_world, armors[i].yaw_world);

        if (score >= std::cos(max_face_angle_) && score > best_score) {
            best_score = score;
            best_idx = i;
            found = true;
        }
    }

    if (!found) {
        return std::nullopt;
    }

    return armors[best_idx];
}
