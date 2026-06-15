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
    // Always publish TrackedTargets
    {
        TrackedTargets targets_msg;
        targets_msg.header = armor_plates->header;

        // LOST 时 targets 为空数组；DETECTING/TRACKING/TEMP_LOST 时发布 EKF 数据
        if (!tracker_.isLost()) {
            armor_plate_interfaces::msg::TrackedTarget target;
            target.track_id = 0;  // 第一版单目标
            target.armor_name = static_cast<int32_t>(tracker_.getArmorName());
            target.tracking_state = armor_plate_interfaces::trackerStateToUint8(tracker_.getState());

            // Fill from EKF state
            auto ekf_state = tracker_.getEKFState();
            target.center_world.x = ekf_state[0];  // x_c
            target.center_world.y = ekf_state[2];  // y_c
            target.center_world.z = ekf_state[4];  // z_c
            target.center_velocity.x = ekf_state[1];  // v_x
            target.center_velocity.y = ekf_state[3];  // v_y
            target.center_velocity.z = ekf_state[5];  // v_z
            target.yaw = ekf_state[6];
            target.yaw_rate = ekf_state[7];
            target.radius = ekf_state[8];
            target.radius_offset = ekf_state[9];
            target.height_offset = ekf_state[10];

            // Reconstruct armor plates from EKF state
            target.armors = tracker_.reconstructArmors();

            targets_msg.targets.push_back(target);
        }

        tracked_targets_pub_->publish(targets_msg);
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
    // ===== ROS 相关 ===== //
    armor_plates_sub_ = this->create_subscription<ArmorPlates>(
        "armor_plates",
        rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
    );
    tracked_targets_pub_ = this->create_publisher<TrackedTargets>("/tracked_targets", rclcpp::SensorDataQoS());
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
