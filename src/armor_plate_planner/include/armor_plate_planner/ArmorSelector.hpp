#pragma once

#include <armor_plate_interfaces/msg/tracked_target.hpp>
#include <armor_plate_interfaces/msg/tracked_armor.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <optional>

class ArmorSelector
{
private:
    double max_face_angle_;

public:
    explicit ArmorSelector(double max_face_angle = 1.0472);

    std::optional<armor_plate_interfaces::msg::TrackedArmor> select(
        const std::vector<armor_plate_interfaces::msg::TrackedArmor> & armors) const;

    // 计算装甲板正面朝向得分
    // armor_position: 装甲板世界坐标
    // armor_yaw: 装甲板自身朝向（TrackedArmor.yaw_world）
    // 返回值: -cos(armor_yaw - yaw_to_armor)，越大表示越正对射手
    double computeFacingScore(
        const geometry_msgs::msg::Point & armor_position,
        double armor_yaw) const;
};
