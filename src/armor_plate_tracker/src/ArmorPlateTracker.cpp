#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/debug_tracker.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::DebugTracker;

class ArmorPlateTracker : public rclcpp::Node
{
private:
    // ===== 装甲板跟踪器  ===== //
    Tracker tracker_;
    double max_lost_time_;
    double mutation_yaw_threshold_;
    double mutation_pitch_threshold_;
    // ===== ROS 相关  ===== //
    rclcpp::Subscription<ArmorPlates>::SharedPtr armor_plates_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<AimCommand>::SharedPtr aim_command_pub_;
    // ===== 时间相关 ===== //
    double current_time_ = 0.0;
    // ===== DEBUG =====//
    bool debug_;
    rclcpp::Publisher<DebugTracker>::SharedPtr debug_tracker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_image_sub_;
    // 相机内参
    cv::Mat camera_matrix_;
    // 最新跟踪结果（用于图像叠加）
    TrackingOverlayData overlay_data_;
    std::mutex overlay_mutex_;

    void init()
    {
        // ===== 参数获取 ===== //
        max_lost_time_ = this->declare_parameter<double>("max_lost_time", 0.5);
        mutation_yaw_threshold_ = this->declare_parameter<double>("mutation_yaw_threshold", 3.0);
        mutation_pitch_threshold_ = this->declare_parameter<double>("mutation_pitch_threshold", 2.0);
        // ===== ROS 相关 ===== //
        armor_plates_sub_ = this->create_subscription<ArmorPlates>(
            "armor_plates", 
            10,
            std::bind(&ArmorPlateTracker::ArmorPlatesCallBack, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&ArmorPlateTracker::info, this));
        aim_command_pub_ = this->create_publisher<AimCommand>("aim_command", 10);
        // ===== DEBUG ===== //
        debug_ = this->declare_parameter<bool>("debug", false);
        // ===== 装甲板跟踪器 ===== //
        tracker_.setMaxLostTime(max_lost_time_);
        tracker_.setMutationThreshold(mutation_yaw_threshold_, mutation_pitch_threshold_);
        tracker_.Init();
        
        if (debug_) RCLCPP_INFO(this->get_logger(), "启动DEBUG模式");
        
    }
    void info()
    {
        float measurement_yaw = tracker_.getMeasuredYaw();
        float measurement_pitch = tracker_.getMeasuredPitch();
        float filter_yaw = tracker_.getYaw();
        float filter_pitch = tracker_.getPitch();
        RCLCPP_INFO(this->get_logger(), "测量数据: yaw = %.2f, pitch = %.2f||滤波数据: yaw = %.2f, pitch = %.2f",
            measurement_yaw, measurement_pitch, filter_yaw, filter_pitch);
    }
    void publish()
    {
        AimCommand aim_command;
        aim_command.delta_pitch = tracker_.getPitch();
        aim_command.delta_yaw = tracker_.getYaw();
        aim_command_pub_->publish(aim_command);
    }
///////// DEBUG ///////////////////
    void Debug()
    {
        debug_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "debug_image",
            10,
            std::bind(&ArmorPlateTracker::DebugImageCallBack, this, std::placeholders::_1)
        );
        debug_tracker_pub_ = this->create_publisher<DebugTracker>("debug_tracker", 10); 
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            2374.54248, 0., 698.85288,
            0., 2377.53648, 520.8649,
            0., 0., 1.);
    }
    cv::Point2f reprojection(const geometry_msgs::msg::Point& position)
    {
        // 直接用 tvec 重投影到图像坐标
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);
        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);
        double X = position.x;
        double Y = position.y;
        double Z = position.z;
        if (std::abs(Z) < 1e-6) return cv::Point2f(-1, -1);
        double u = fx * X / Z + cx;
        double v = fy * Y / Z + cy;
        return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    }
    cv::Point2f reprojectionFromYawPitch(float yaw, float pitch, float distance)
    {
        // 从 yaw/pitch/distance 重建 tvec 并重投影
        double yaw_rad = yaw * CV_PI / 180.0;
        double pitch_rad = pitch * CV_PI / 180.0;
        geometry_msgs::msg::Point p;
        p.x = distance * std::sin(yaw_rad) * std::cos(pitch_rad);
        p.y = -distance * std::sin(pitch_rad);
        p.z = distance * std::cos(yaw_rad) * std::cos(pitch_rad);
        return reprojection(p);
    }
    void drawPoint(cv::Mat& img, const cv::Point2f& pt, const cv::Scalar& color, const std::string& label)
    {
        if (pt.x < 0 || pt.y < 0) return;
        cv::circle(img, pt, 8, color, 2);
        cv::circle(img, pt, 3, color, -1);
        int cross_len = 12;
        cv::line(img, cv::Point(pt.x - cross_len, pt.y), cv::Point(pt.x + cross_len, pt.y), color, 2);
        cv::line(img, cv::Point(pt.x, pt.y - cross_len), cv::Point(pt.x, pt.y + cross_len), color, 2);
        if (!label.empty()) {
            cv::putText(img, label, cv::Point(pt.x + 15, pt.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, color, 2);
        }
    }
    void drawYawPitchText(cv::Mat& img, const cv::Point2f& pt, float yaw, float pitch, const cv::Scalar& color)
    {
        if (pt.x < 0 || pt.y < 0) return;
        char buf[64];
        snprintf(buf, sizeof(buf), "yaw:%.2f pitch:%.2f", yaw, pitch);
        cv::putText(img, buf, cv::Point(pt.x + 15, pt.y + 15), cv::FONT_HERSHEY_PLAIN, 1.2, color, 2);
    }
    void DebugImageCallBack(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat overlay = cv_ptr->image;

            // 在这里画图：每来一帧图像，画一次最新的跟踪结果
            std::lock_guard<std::mutex> lock(overlay_mutex_);
            if (overlay_data_.has_result) {
                cv::Point2f measured_pt = reprojection(overlay_data_.measured_position);
                drawPoint(overlay, measured_pt, cv::Scalar(0, 0, 255), "measured");
                drawYawPitchText(overlay, measured_pt, overlay_data_.measured_yaw, overlay_data_.measured_pitch, cv::Scalar(255, 0, 255));

                if (!overlay_data_.is_lost) {
                    cv::Point2f filtered_pt = reprojectionFromYawPitch(
                        overlay_data_.filter_yaw, overlay_data_.filter_pitch, overlay_data_.distance);
                    drawPoint(overlay, filtered_pt, cv::Scalar(0, 255, 0), "filtered");
                    drawYawPitchText(overlay, filtered_pt, overlay_data_.filter_yaw, overlay_data_.filter_pitch, cv::Scalar(0, 255, 0));
                } else {
                    cv::Point2f predicted_pt = reprojectionFromYawPitch(
                        overlay_data_.filter_yaw, overlay_data_.filter_pitch, overlay_data_.distance);
                    drawPoint(overlay, predicted_pt, cv::Scalar(0, 255, 255), "predicted");
                    drawYawPitchText(overlay, predicted_pt, overlay_data_.filter_yaw, overlay_data_.filter_pitch, cv::Scalar(0, 255, 0));
                }
            }

            cv::imshow("Tracker Debug", overlay);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
        }
    }
    void ArmorPlatesCallBack(const ArmorPlates::SharedPtr msg)
    {
        double current_time = this->now().seconds();
        current_time_ = current_time;
        size_t num = msg->armor_plates.size();

        std::vector<geometry_msgs::msg::Point> positions;
        std::vector<float> image_distances;
        positions.reserve(num);
        image_distances.reserve(num);

        for (size_t i = 0; i < num; ++i) {
            positions.push_back(msg->armor_plates[i].pose.position);
            image_distances.push_back(msg->armor_plates[i].image_distance_to_center);
        }

        tracker_.Update(positions, image_distances, current_time);

        if (debug_) {
            DebugTracker debug_msg;
            debug_msg.measurement_yaw = tracker_.getMeasuredYaw();
            debug_msg.measurement_pitch = tracker_.getMeasuredPitch();
            debug_msg.filter_yaw = tracker_.getYaw();
            debug_msg.filter_pitch = tracker_.getPitch();
            if (debug_tracker_pub_) {
                debug_tracker_pub_->publish(debug_msg);
            }
        }

        // 更新最新跟踪结果数据，不在此画图
        if (debug_) {
            auto measured_pos = tracker_.getMeasuredPosition();
            float distance = static_cast<float>(std::sqrt(
                measured_pos.x * measured_pos.x +
                measured_pos.y * measured_pos.y +
                measured_pos.z * measured_pos.z));

            std::lock_guard<std::mutex> lock(overlay_mutex_);
            overlay_data_.has_result = true;
            overlay_data_.is_lost = tracker_.isLost();
            overlay_data_.measured_position = measured_pos;
            overlay_data_.measured_yaw = tracker_.getMeasuredYaw();
            overlay_data_.measured_pitch = tracker_.getMeasuredPitch();
            overlay_data_.filter_yaw = tracker_.getYaw();
            overlay_data_.filter_pitch = tracker_.getPitch();
            overlay_data_.distance = distance;
        }
    }

public:
    ArmorPlateTracker() : Node("armor_plate_tracker_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "Armor Plate Tracker节点创建成功！");
        init();
        publish();
        if (debug_) {
            Debug();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateTracker>());
    rclcpp::shutdown();
    return 0;
}
