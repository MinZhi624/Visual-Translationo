#include "armor_plate_planner/ArmorSelector.hpp"
#include <armor_plate_common/geometry.hpp>
#include <armor_plate_common/angle.hpp>
#include <cmath>

namespace apc = armor_plate_common;

ArmorSelector::ArmorSelector(double max_face_angle)
    : max_face_angle_(max_face_angle)
{
}

double ArmorSelector::computeFacingScore(
    const Eigen::Vector3d& armor_position,
    double armor_yaw) const
{
    // 计算射手→装甲板方向角 yaw_to_armor
    double yaw_to_armor = apc::calculateYPD(armor_position).x();

    // 朝向差: 装甲板法线与射手方向的夹角
    double delta_angle = apc::normalizeRadAngle(armor_yaw - yaw_to_armor);

    // cos(delta_angle): 越正对射手得分越高（最大 1，最小 -1）
    return std::cos(delta_angle);
}

std::optional<PlannerArmor> ArmorSelector::select(
    const std::vector<PlannerArmor>& armors) const
{
    if (armors.empty()) return std::nullopt;

    double best_score = -2.0;
    size_t best_idx = 0;
    bool found = false;

    for (size_t i = 0; i < armors.size(); ++i) {
        double score = computeFacingScore(armors[i].pose.xyz_world, armors[i].pose.yaw);

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
