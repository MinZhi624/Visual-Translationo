#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_tracker/DebugTracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/tracker_data.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
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
    rclcpp::Publisher<TrackerDebug>::SharedPtr tracker_debug_pub_;

    void init()
    {
        // ===== 参数获取 ===== //
        max_lost_time_ = this->declare_parameter<double>("max_lost_time", 0.5);
        mutation_yaw_threshold_ = this->declare_parameter<double>("mutation_yaw_threshold", 3.0);
        // ===== ROS 相关 ===== //
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
             .best_effort()
             .durability_volatile();
        armor_plates_sub_ = this->create_subscription<ArmorPlates>(
            "armor_plates", 
            qos,
            std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
        );
        aim_command_pub_ = this->create_publisher<AimCommand>("aim_command", qos);
        tracker_data_pub_ = this->create_publisher<TrackerData>("tracker_data", 10);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        // ===== DEBUG ===== //
        debug_ = this->declare_parameter<bool>("debug", false);
        if(debug_) {
            tracker_debug_pub_ = this->create_publisher<TrackerDebug>("tracker_debug", 10);
        }
        // ===== 装甲板跟踪器 ===== //
        tracker_.setMaxLostTime(max_lost_time_);
        tracker_.setMutationThreshold(mutation_yaw_threshold_);
        tracker_.reset();
        
        if (debug_) RCLCPP_INFO(this->get_logger(), "启动DEBUG模式");
    }
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg)
    {
        // 数据获取
        image_stamp_ = msg->header.stamp;
        current_time_ = image_stamp_.sec + image_stamp_.nanosec * 1e-9;
        const auto& armor_plates = msg->armor_plates;
        tracker_.Update(armor_plates, current_time_, msg->gimbal_yaw_abs, msg->gimbal_pitch_abs);
        publish(msg);
    }
    void publishMarkerArray(const rclcpp::Time& now)
    {
        auto center = tracker_.getCenterPointWorld();
        const auto & measured = tracker_.getMeasuredArmor();
        const auto & filtered = tracker_.getFilterArmor();
        int selected_id = tracker_.getSelectedArmorId();
        visualization_msgs::msg::MarkerArray arr;
        // 旋转轴（绿色中心点 + 向上的绿色箭头)
        arr.markers.push_back(createSphereMarker(
            center, "world", now, 0,
            0.08f,
            0.0f, 1.0f, 0.0f, 1.0f
        ));
        Eigen::Vector3d axis_top = center + Eigen::Vector3d(0.0, 0.0, 0.5);
        arr.markers.push_back(createArrowMarker(
            center, axis_top, "world", now, 1,
            0.02f, 0.06f, 0.0f,
            0.0f, 1.0f, 0.0f, 1.0f
        ));
        // 中心速度箭头（黄色）
        Eigen::Vector3d velocity = tracker_.getCenterVelocity();
        constexpr double kVelocityScale = 0.5;
        Eigen::Vector3d arrow_end = center + velocity * kVelocityScale;
        arr.markers.push_back(createArrowMarker(
            center, arrow_end, "world", now, 2,
            0.02f, 0.06f, 0.0f,
            1.0f, 1.0f, 0.0f, 1.0f
        ));
        // 观测装甲板（红色）
        arr.markers.push_back(createBoxMarker(
            measured.xyz_world_, measured.q_world_armor_,
            "world", now, 3,
            1.0f, 0.0f, 0.0f, 1.0f
        ));
        if (selected_id >= 0) {
            arr.markers.push_back(createTextMarker(
                measured.xyz_world_, "target id=" + std::to_string(selected_id),
                "world", now, 13,
                0.08f,
                1.0f, 0.2f, 0.2f, 1.0f
            ));
        }
        // 滤波装甲板（绿色）
        arr.markers.push_back(createBoxMarker(
            filtered.xyz_world_, filtered.q_world_armor_,
            "world", now, 4,
            0.0f, 1.0f, 0.0f, 1.0f
        ));
        // 四个预测装甲板（绿色，id 5-8）+ 文字标签（id 9-12）
        if (tracker_.isInitialized()) {
            auto armor_list = tracker_.getTrackerArmorList();
            for (int i = 0; i < 4; ++i) {
                double angle = armor_list[i][3];
                Eigen::Vector3d pos(armor_list[i][0], armor_list[i][1], armor_list[i][2]);
                Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
                bool selected = i == selected_id;
                arr.markers.push_back(createBoxMarker(
                    pos, q,
                    "world", now, 5 + i,
                    selected ? 0.0f : 0.0f,
                    selected ? 0.4f : 1.0f,
                    selected ? 1.0f : 0.0f,
                    1.0f
                ));
                arr.markers.push_back(createTextMarker(
                    pos, selected ? "id=" + std::to_string(i) + " *" : "id=" + std::to_string(i),
                    "world", now, 9 + i,
                    0.08f,
                    selected ? 0.0f : 1.0f,
                    selected ? 0.6f : 1.0f,
                    1.0f, 1.0f
                ));
            }
        }

        marker_array_pub_->publish(arr);
    }
    void publish(const ArmorPlates::SharedPtr armor_plates)
    {
        const auto & measured = tracker_.getMeasuredArmor();
        const auto & filtered = tracker_.getFilterArmor();

        // AimCommand 和 TrackerData 仅在跟踪成功时发送
        if (!tracker_.isLost()) {
            AimCommand aim_command;
            aim_command.delta_pitch = tracker_.getPitch();
            aim_command.delta_yaw = tracker_.getYaw();
            aim_command_pub_->publish(aim_command);

            // RCLCPP_INFO(this->get_logger(),
            //     "delta_yaw=%.4f rad (%.2f deg), delta_pitch=%.4f rad (%.2f deg)",
            //     aim_command.delta_yaw, aim_command.delta_yaw * 180.0 / M_PI,
            //     aim_command.delta_pitch, aim_command.delta_pitch * 180.0 / M_PI);

            TrackerData tracker_data_msg;
            tracker_data_msg.header = armor_plates->header;
            tracker_data_msg.measurement_yaw = measured.ypd_camera_.x();
            tracker_data_msg.measurement_pitch = measured.ypd_camera_.y();
            tracker_data_msg.filter_yaw = filtered.ypd_camera_.x();
            tracker_data_msg.filter_pitch = filtered.ypd_camera_.y();
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

public:
    ArmorPlateTracker() : Node("armor_plate_tracker_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Armor Plate Tracker节点创建成功！");
        init();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateTracker>());
    rclcpp::shutdown();
    return 0;
}
