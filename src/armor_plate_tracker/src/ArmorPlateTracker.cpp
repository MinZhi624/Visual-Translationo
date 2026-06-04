#include "armor_plate_tracker/ArmorPlateTracker.hpp"

ArmorPlateTracker::ArmorPlateTracker() : Node("armor_plate_tracker_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "Armor Plate Tracker节点创建成功！");
    init();
}

void ArmorPlateTracker::ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg)
{
    // 数据获取
    image_stamp_ = msg->header.stamp;
    current_time_ = image_stamp_.sec + image_stamp_.nanosec * 1e-9;
    const auto& armor_plates = msg->armor_plates;
    GimbalData gimbal{msg->gimbal_yaw_abs, msg->gimbal_pitch_abs};
    tracker_.Update(armor_plates, current_time_, gimbal);
    publish(msg);
}

void ArmorPlateTracker::publish(const ArmorPlates::SharedPtr armor_plates)
{
    const auto & measured = tracker_.getMeasuredArmor();
    const auto & filtered = tracker_.getFilterArmor();
    // AimCommand 和 TrackerData 仅在跟踪成功时发送
    if (!tracker_.isLost()) {
        if(tracker_.isSend()) {
            AimCommand aim_command;
            aim_command.delta_pitch = tracker_.getPitch();
            aim_command.delta_yaw = tracker_.getYaw();
            aim_command_pub_->publish(aim_command);
        }
        

        // RCLCPP_INFO(this->get_logger(),
        //     "delta_yaw=%.4f rad (%.2f deg), delta_pitch=%.4f rad (%.2f deg)",
        //     aim_command.delta_yaw, aim_command.delta_yaw * 180.0 / M_PI,
        //     aim_command.delta_pitch, aim_command.delta_pitch * 180.0 / M_PI);

        TrackerData tracker_data_msg;
        tracker_data_msg.header = armor_plates->header;
        tracker_data_msg.measurement_yaw = measured.ypd_gimbal_.x();
        tracker_data_msg.measurement_pitch = measured.ypd_gimbal_.y();
        tracker_data_msg.filter_yaw = filtered.ypd_gimbal_.x();
        tracker_data_msg.filter_pitch = filtered.ypd_gimbal_.y();
        if (tracker_data_pub_) {
            tracker_data_pub_->publish(tracker_data_msg);
        }
    }

    auto now = this->now();
    // 发布可视化数据
    publishMarkerArray(now);
    ////////// DEBUG //////////
    if (debug_) {
        TrackerDebug debug_msg = tracker_.CreatedebugMsg(image_stamp_);
        tracker_debug_pub_->publish(debug_msg);
    }
}

void ArmorPlateTracker::init()
{
    // ===== 参数获取 ===== //
    max_lost_time_ = this->declare_parameter<double>("max_lost_time", 0.5);
    mutation_yaw_threshold_ = this->declare_parameter<double>("mutation_yaw_threshold", 3.0);
    // ===== ROS 相关 ===== //
    armor_plates_sub_ = this->create_subscription<ArmorPlates>(
        "armor_plates",
        rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
    );
    aim_command_pub_ = this->create_publisher<AimCommand>("aim_command", rclcpp::SensorDataQoS());
    tracker_data_pub_ = this->create_publisher<TrackerData>("tracker_data", 10);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    // ===== DEBUG ===== //
    debug_ = this->declare_parameter<bool>("debug", false);
    if(debug_) {
        RCLCPP_INFO(this->get_logger(), "TRACKER 启动DEBUG模式");
        tracker_debug_pub_ = this->create_publisher<TrackerDebug>("tracker_debug", 10);
    }
    // ===== 装甲板跟踪器 ===== //
    tracker_.setMaxLostTime(max_lost_time_);
    tracker_.reset();

    if (debug_) RCLCPP_INFO(this->get_logger(), "启动DEBUG模式");
}

void ArmorPlateTracker::publishMarkerArray(const rclcpp::Time& now)
{
    auto center = tracker_.getCenterPointWorld();
    const auto & measured = tracker_.getMeasuredArmor();
    const auto & filtered = tracker_.getFilterArmor();
    visualization_msgs::msg::MarkerArray arr;

    // 观测装甲板（红色）
    arr.markers.push_back(createMeasurementMarker(measured, now, 3));
    // 滤波装甲板（绿色）
    arr.markers.push_back(createFilteredMarker(filtered, now, 4));

    // 车体相关 marker（中心点、速度、预测装甲板）
    if (!tracker_.isLost()) {
        auto armor_list = tracker_.getTrackerArmorList();
        auto car_markers = createCarMarkers(
            armor_list, center, tracker_.getCenterVelocity(), now, 0);
        arr.markers.insert(arr.markers.end(), car_markers.begin(), car_markers.end());
    }

    marker_array_pub_->publish(arr);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateTracker>());
    rclcpp::shutdown();
    return 0;
}
