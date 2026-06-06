#pragma once

#include "armor_plate_tracker/TrackerArmor.hpp"
#include <armor_plate_interfaces/ArmorPose.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/time.hpp>

using visualization_msgs::msg::Marker;

/*
    这里是创建世界坐标系下的各种标记的函数
*/

std::vector<Marker> createCarMarkers(
    const std::array<ArmorPose, 4> & armor_list,
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & car_speed,
    const rclcpp::Time & stamp,
    int id
);
Marker createMeasurementMarker(
    const TrackerArmor & armor,
    const rclcpp::Time & stamp,
    int id
);

Marker createFilteredMarker(
    const TrackerArmor & armor,
    const rclcpp::Time & stamp,
    int id
);
///////// 辅助函数 ///////
Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id, float scale, float r, float g, float b, float a);

Marker createBoxMarker(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float r, float g, float b, float a);


Marker createArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float shaft_diameter,
    float head_diameter,
    float head_length,
    float r, float g, float b, float a);

Marker createTextMarker(
    const Eigen::Vector3d& position,
    const std::string& text,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float scale,
    float r, float g, float b, float a);

