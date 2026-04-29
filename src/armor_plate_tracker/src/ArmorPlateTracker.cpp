#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include "armor_plate_interfaces/msg/tracker_data.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <mutex>
#include <deque>

using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::GimbalAngle;
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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::Subscription<GimbalAngle>::SharedPtr gimbal_ganle_sub_;
    rclcpp::Publisher<AimCommand>::SharedPtr aim_command_pub_;
    rclcpp::Publisher<TrackerData>::SharedPtr tracker_data_pub_;
    // 数据可视化
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filter_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr measured_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    // ===== 时间相关 ===== //
    double current_time_ = 0.0;
    builtin_interfaces::msg::Time image_stamp_;
    // ===== 绝对角度 ===== //
    std::deque<AngleRecord> angle_buffer_;
    std::mutex angle_buffer_mutex_;

    // ===== DEBUG =====//
    bool debug_;
    rclcpp::Publisher<TrackerDebug>::SharedPtr tracker_debug_pub_;

    void init()
    {
        // ===== 参数获取 ===== //
        max_lost_time_ = this->declare_parameter<double>("max_lost_time", 0.5);
        mutation_yaw_threshold_ = this->declare_parameter<double>("mutation_yaw_threshold", 3.0);
        // ===== ROS 相关 ===== //
        armor_plates_sub_ = this->create_subscription<ArmorPlates>(
            "armor_plates", 
            10,
            std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ArmorPlateTracker::info, this));
        aim_command_pub_ = this->create_publisher<AimCommand>("aim_command", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        gimbal_ganle_sub_ = this->create_subscription<GimbalAngle>(
            "gimbal_angle", 10,
            [this](const GimbalAngle::SharedPtr msg){
                std::lock_guard<std::mutex> lock(angle_buffer_mutex_);
                angle_buffer_.push_back({msg->stamp, msg->yaw_abs, msg->pitch_abs});
                if(angle_buffer_.size() > 50) angle_buffer_.pop_front();
            }
        );
        tracker_data_pub_ = this->create_publisher<TrackerData>("tracker_data", 10);
        filter_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("filter_pose", 10);
        measured_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("measured_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        // ===== DEBUG ===== //
        debug_ = this->declare_parameter<bool>("debug", false);
        if(debug_) {
            tracker_debug_pub_ = this->create_publisher<TrackerDebug>("tracker_debug", 10);
        }
        // ===== 装甲板跟踪器 ===== //
        tracker_.setMaxLostTime(max_lost_time_);
        tracker_.setMutationThreshold(mutation_yaw_threshold_);
        tracker_.Init();
        
        if (debug_) RCLCPP_INFO(this->get_logger(), "启动DEBUG模式");
    }
    double findClosestAngle(const builtin_interfaces::msg::Time& image_stamp, float& output_yaw, float& output_pitch)
    {
        double image_time = image_stamp.sec + image_stamp.nanosec * 1e-9;
        
        // 获取角度缓存
        std::vector<AngleRecord> local_buffer;
        {
            std::lock_guard<std::mutex> lock(angle_buffer_mutex_);
            if (angle_buffer_.empty()) {
                output_yaw = 0.0f;
                output_pitch = 0.0f;
                // RCLCPP_WARN(this->get_logger(), "角度缓存为空，无法对齐，使用 0");
                return 0.0;
            }
            local_buffer.assign(angle_buffer_.begin(), angle_buffer_.end());
        }
        
        auto closest_it = local_buffer.begin();
        double min_diff = std::abs(
            (closest_it->stamp.sec + closest_it->stamp.nanosec * 1e-9) - image_time);
        
        for (auto it = local_buffer.begin() + 1; it != local_buffer.end(); ++it) {
            double record_time = it->stamp.sec + it->stamp.nanosec * 1e-9;
            double diff = std::abs(record_time - image_time);
            if (diff < min_diff) {
                min_diff = diff;
                closest_it = it;
            }
        }
        
        output_yaw = closest_it->yaw_abs;
        output_pitch = closest_it->pitch_abs;
        
        if (min_diff > 0.05) {
            RCLCPP_WARN(this->get_logger(), 
                "角度-图像时间差 %.1f ms，对齐可能不准", min_diff * 1000.0);
        }
        return closest_it->stamp.sec + closest_it->stamp.nanosec * 1e-9 - image_time;
    }
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg)
    {
        // 数据获取
        image_stamp_ = msg->header.stamp;
        current_time_ = image_stamp_.sec + image_stamp_.nanosec * 1e-9;
        const auto& armor_plates = msg->armor_plates;
        float yaw_abs = 0.0;
        float pitch_abs = 0.0;
        findClosestAngle(msg->header.stamp, yaw_abs, pitch_abs);
        tracker_.Update(armor_plates, current_time_, yaw_abs, pitch_abs);
        publish(msg);
    }
    void info()
    {
        float measurement_yaw = tracker_.getMeasuredYaw();
        float measurement_pitch = tracker_.getMeasuredPitch();
        float filter_yaw = tracker_.getYaw();
        float filter_pitch = tracker_.getPitch();
        PoseStamped measured_position_world = tracker_.getMeasuredPositionWorld();
        Eigen::Quaterniond measured_q(
            measured_position_world.pose.orientation.w,
            measured_position_world.pose.orientation.x,
            measured_position_world.pose.orientation.y,
            measured_position_world.pose.orientation.z
        );
        float pose_yaw = calculatePoseYaw(measured_q);
        RCLCPP_INFO(this->get_logger(), "测量数据: yaw = %.4f, pitch = %.4f||滤波数据: yaw = %.4f, pitch = %.4f",
            measurement_yaw, measurement_pitch, filter_yaw, filter_pitch);
        RCLCPP_INFO(this->get_logger(), "装甲板姿态: yaw = %.4f", pose_yaw);
    }
    visualization_msgs::msg::Marker createSphereMarker(
        const geometry_msgs::msg::PoseStamped& pose_stamped,
        int id, float scale, float r, float g, float b, float a)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = pose_stamped.header;
        marker.ns = "tracker_sphere";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_stamped.pose;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        return marker;
    }

    void publish(const ArmorPlates::SharedPtr armor_plates)
    {
        // 发布滤波装甲板数据
        TrackerData debug_msg;
        debug_msg.measurement_yaw = tracker_.getMeasuredYaw();
        debug_msg.measurement_pitch = tracker_.getMeasuredPitch();
        debug_msg.filter_yaw = tracker_.getYaw();
        debug_msg.filter_pitch = tracker_.getPitch();
        if (tracker_data_pub_) {
            tracker_data_pub_->publish(debug_msg);
        }
        // TF 发布到相机坐标系
        int armor_plate_count = 0;
        for (const auto& armor_plate : armor_plates->armor_plates) {
            geometry_msgs::msg::TransformStamped transfrom_stamped;
            transfrom_stamped.header.stamp = this->now();
            transfrom_stamped.header.frame_id = armor_plates->header.frame_id;
            transfrom_stamped.child_frame_id = "armor_plate_" + std::to_string(++armor_plate_count);
            transfrom_stamped.transform.translation.x = armor_plate.pose.position.x;
            transfrom_stamped.transform.translation.y = armor_plate.pose.position.y;
            transfrom_stamped.transform.translation.z = armor_plate.pose.position.z;
            transfrom_stamped.transform.rotation = armor_plate.pose.orientation;
            tf_broadcaster_->sendTransform(transfrom_stamped);
        }
        // 发布目标位姿 世界坐标系
        PoseStamped filter_position_world = tracker_.getFilterPositionWorld();
        PoseStamped measured_position_world = tracker_.getMeasuredPositionWorld();
        auto now = this->now();
        filter_position_world.header.stamp = now;
        measured_position_world.header.stamp = now;
        filter_pose_pub_->publish(filter_position_world);
        measured_pose_pub_->publish(measured_position_world);
        // TF发布到世界坐标系
        geometry_msgs::msg::TransformStamped filter_transform_stamped;
        filter_transform_stamped.header.stamp = now;
        filter_transform_stamped.header.frame_id = "world";
        filter_transform_stamped.child_frame_id = "measured_pose";
        filter_transform_stamped.transform.translation.x = filter_position_world.pose.position.x;
        filter_transform_stamped.transform.translation.y = filter_position_world.pose.position.y;
        filter_transform_stamped.transform.translation.z = filter_position_world.pose.position.z;
        filter_transform_stamped.transform.rotation = filter_position_world.pose.orientation;
        tf_broadcaster_->sendTransform(filter_transform_stamped);
        // 发布球体 Marker 到 RViz
        marker_pub_->publish(createSphereMarker(filter_position_world, 0, 0.15f, 0.0f, 1.0f, 0.0f, 1.0f));
        marker_pub_->publish(createSphereMarker(measured_position_world, 1, 0.20f, 1.0f, 0.0f, 0.0f, 0.5f));
        ////////// DEBUG //////////
        if (debug_) {
            TrackerDebug debug_msg;
            debug_msg.header.stamp = image_stamp_;
            auto eigen2geometry = [](const Eigen::Vector3d& vec) {
                geometry_msgs::msg::Vector3 vec_msg;
                vec_msg.x = vec.x();
                vec_msg.y = vec.y();
                vec_msg.z = vec.z();
                return vec_msg;
            };
            if (!tracker_.isLost()) {
                Eigen::Vector3d measured_cam = tracker_.getMeasuredPositionCamera();
                debug_msg.target_point = eigen2geometry(measured_cam);
                Eigen::Vector3d filtered_cam = tracker_.getFilterPositionCamera();
                debug_msg.filtered_point = eigen2geometry(filtered_cam);
            } else {
                // 丢失时放在相机正前方（光轴上，投影到图像中心）
                geometry_msgs::msg::Vector3 center_point;
                center_point.x = 0.0;
                center_point.y = 0.0;
                center_point.z = 1.0;
                debug_msg.target_point = center_point;
                debug_msg.filtered_point = center_point;
            }

            tracker_debug_pub_->publish(debug_msg);
        }

        // 发送增量角
        if (tracker_.isLost()) {
            RCLCPP_WARN(this->get_logger(), "目标丢失");
            return;
        } else {
            AimCommand aim_command;
            aim_command.delta_pitch = tracker_.getPitch();
            aim_command.delta_yaw = tracker_.getYaw();
            aim_command_pub_->publish(aim_command);
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
