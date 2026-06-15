#pragma once

#include <armor_plate_interfaces/msg/tracked_target.hpp>
#include <armor_plate_interfaces/msg/tracked_armor.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <optional>

class ArmorSelector
{
private:
    double max_face_angle_;

public:
    explicit ArmorSelector(double max_face_angle = 1.0472);

    std::optional<armor_plate_interfaces::msg::TrackedArmor> select(
        const std::vector<armor_plate_interfaces::msg::TrackedArmor> & armors,
        const geometry_msgs::msg::Point & shooter_origin,
        const geometry_msgs::msg::Point & target_center) const;

    double computeFacingScore(
        const geometry_msgs::msg::Point & armor_position,
        const geometry_msgs::msg::Point & target_center,
        const geometry_msgs::msg::Point & shooter_origin) const;
};
