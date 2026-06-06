#include "armor_plate_tracker/DebugTracker.hpp"
#include <armor_plate_interfaces/armor_geometry.hpp>

using armor_plate_interfaces::SMALL_ARMOR_WIDTH;
using armor_plate_interfaces::SMALL_ARMOR_HEIGHT;

// 装甲板厚度（marker 深度，测试用平面模型）
static constexpr float ARMOR_DEPTH = 0.010f;  // 10mm

std::vector<Marker> createCarMarkers(
    const std::array<ArmorPose, 4> & armor_list,
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & car_speed,
    const rclcpp::Time & stamp,
    int base_id
)
{
    std::vector<Marker> markers;
    markers.reserve(11);

    // 中心点球体（绿色）
    markers.push_back(createSphereMarker(
        center, "world", stamp, base_id + 0,
        0.08f, 0.0f, 1.0f, 0.0f, 1.0f));

    // 旋转轴箭头（绿色，向上0.5m）
    Eigen::Vector3d axis_top = center + Eigen::Vector3d(0.0, 0.0, 0.5);
    markers.push_back(createArrowMarker(
        center, axis_top, "world", stamp, base_id + 1,
        0.02f, 0.06f, 0.0f,
        0.0f, 1.0f, 0.0f, 1.0f));

    // 中心速度箭头（黄色）
    constexpr double kVelocityScale = 0.5;
    Eigen::Vector3d arrow_end = center + car_speed * kVelocityScale;
    markers.push_back(createArrowMarker(
        center, arrow_end, "world", stamp, base_id + 2,
        0.02f, 0.06f, 0.0f,
        1.0f, 1.0f, 0.0f, 1.0f));

    // 四个预测装甲板（id 5-8）+ 文字标签（id 9-12）
    for (int i = 0; i < 4; ++i) {
        double angle = armor_list[i].yaw;
        Eigen::Vector3d pos = armor_list[i].xyz_world;
        Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
        markers.push_back(createBoxMarker(
            pos, q, "world", stamp, base_id + 5 + i,
            0.0f, 1.0f, 0.0f, 1.0f));
        markers.push_back(createTextMarker(
            pos, "id=" + std::to_string(i),
            "world", stamp, base_id + 9 + i,
            0.08f, 1.0f, 1.0f, 1.0f, 1.0f));
    }

    return markers;
}

Marker createMeasurementMarker(
    const TrackerArmor & armor,
    const rclcpp::Time & stamp,
    int id
)
{
    return createBoxMarker(
        armor.xyz_world_, armor.q_world_armor_,
        "world", stamp, id,
        1.0f, 0.0f, 0.0f, 1.0f);
}

Marker createFilteredMarker(
    const TrackerArmor & armor,
    const rclcpp::Time & stamp,
    int id
)
{
    return createBoxMarker(
        armor.xyz_world_, armor.q_world_armor_,
        "world", stamp, id,
        0.0f, 0.0f, 1.0f, 1.0f);
}

visualization_msgs::msg::Marker createSphereMarker(
    const Eigen::Vector3d& position,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id, float scale, float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_sphere";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(100.0);
    return marker;
}

visualization_msgs::msg::Marker createBoxMarker(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_box";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = orientation.w();
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    // 装甲板尺寸: 厚度10mm, 宽度135mm, 高度55mm
    marker.scale.x = ARMOR_DEPTH;
    marker.scale.y = SMALL_ARMOR_WIDTH;
    marker.scale.z = SMALL_ARMOR_HEIGHT;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(100.0);
    return marker;
}

visualization_msgs::msg::Marker createArrowMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float shaft_diameter,
    float head_diameter,
    float head_length,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_velocity";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = start.x();
    p_start.y = start.y();
    p_start.z = start.z();
    p_end.x = end.x();
    p_end.y = end.y();
    p_end.z = end.z();
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);

    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.scale.z = head_length;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(100.0);
    return marker;
}

visualization_msgs::msg::Marker createTextMarker(
    const Eigen::Vector3d& position,
    const std::string& text,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    float scale,
    float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "tracker_text";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z() + 0.1;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(100.0);
    return marker;
}


