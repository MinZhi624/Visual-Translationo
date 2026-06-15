#pragma once
#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_tracker/DebugTracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracked_targets.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackedTargets;
using armor_plate_interfaces::msg::TrackerDebug;

class ArmorPlateTracker : public rclcpp::Node
{
private:
    // ===== 装甲板跟踪器  ===== //
    Tracker tracker_;
    double max_lost_time_;
    // ===== ROS 相关  ===== //
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::Publisher<TrackedTargets>::SharedPtr tracked_targets_pub_;
    // 数据可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    // ===== 时间相关 ===== //
    double current_time_ = 0.0;
    builtin_interfaces::msg::Time image_stamp_;

    // ===== DEBUG =====//
    bool debug_;
    rclcpp::Publisher<TrackerDebug>::SharedPtr tracker_debug_pub_;

    void init();
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg);
    void publishMarkerArray(const rclcpp::Time& now);
    void publish(const ArmorPlates::SharedPtr armor_plates);

public:
    ArmorPlateTracker();
};
