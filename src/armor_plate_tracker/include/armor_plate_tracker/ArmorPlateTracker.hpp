#pragma once
#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_tracker/DebugTracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/tracker_data.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::TrackerData;
using armor_plate_interfaces::msg::TrackerDebug;

class ArmorPlateTracker : public rclcpp::Node
{
private:
    // ===== 装甲板跟踪器  ===== //
    Tracker tracker_;
    double max_lost_time_;
    double mutation_yaw_threshold_;
    // ===== ROS 相关  ===== //
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::Publisher<AimCommand>::SharedPtr aim_command_pub_;
    rclcpp::Publisher<TrackerData>::SharedPtr tracker_data_pub_;
    // 数据可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    // ===== 时间相关 ===== //
    double current_time_ = 0.0;
    builtin_interfaces::msg::Time image_stamp_;

    // ===== DEBUG =====//
    bool debug_;
    bool diagnostic_log_ = false;
    rclcpp::Publisher<TrackerDebug>::SharedPtr tracker_debug_pub_;

    void init();
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg);
    void publishMarkerArray(const rclcpp::Time& now);
    void publish(const ArmorPlates::SharedPtr armor_plates);

public:
    ArmorPlateTracker();
};
