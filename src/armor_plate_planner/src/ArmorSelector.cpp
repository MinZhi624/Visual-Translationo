#include "armor_plate_planner/ArmorSelector.hpp"
#include <cmath>

ArmorSelector::ArmorSelector(double max_face_angle)
    : max_face_angle_(max_face_angle)
{
}

double ArmorSelector::computeFacingScore(
    const geometry_msgs::msg::Point & armor_position,
    const geometry_msgs::msg::Point & target_center,
    const geometry_msgs::msg::Point & shooter_origin) const
{
    double ox = armor_position.x - target_center.x;
    double oy = armor_position.y - target_center.y;
    double oz = armor_position.z - target_center.z;
    double out_len = std::sqrt(ox * ox + oy * oy + oz * oz);
    if (out_len < 1e-6) return 0.0;
    ox /= out_len;
    oy /= out_len;
    oz /= out_len;

    double tx = shooter_origin.x - armor_position.x;
    double ty = shooter_origin.y - armor_position.y;
    double tz = shooter_origin.z - armor_position.z;
    double to_len = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (to_len < 1e-6) return 0.0;
    tx /= to_len;
    ty /= to_len;
    tz /= to_len;

    // 从车辆旋转中心指向装甲板中心，表示装甲板朝向车外的法向。
    // outward = normalize(armor_position - target_center);

    // 从装甲板中心指向我方射击原点。
    // to_shooter = normalize(shooter_origin - armor_position);

    // 点积接近 1 表示正面朝向我方，接近 0 表示侧面，
    // 接近 -1 表示背面朝向我方。
    // facing_score = dot(outward, to_shooter);

    return ox * tx + oy * ty + oz * tz;
}

std::optional<armor_plate_interfaces::msg::TrackedArmor> ArmorSelector::select(
    const std::vector<armor_plate_interfaces::msg::TrackedArmor> & armors,
    const geometry_msgs::msg::Point & shooter_origin,
    const geometry_msgs::msg::Point & target_center) const
{
    if (armors.empty()) return std::nullopt;

    double best_score = -2.0;
    size_t best_idx = 0;
    bool found = false;

    for (size_t i = 0; i < armors.size(); ++i) {
        double score = computeFacingScore(armors[i].position_world, target_center, shooter_origin);

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
