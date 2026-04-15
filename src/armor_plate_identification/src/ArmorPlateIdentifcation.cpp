#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/TestFunc.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <cmath>

#include "tf2_ros/transform_broadcaster.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    cv::Mat img_show_;
    PairedLights lights_;
    PoseSolver pose_solver_;
    std::string target_color_; // "RED" 或 "BLUE"
    // ros相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    image_transport::CameraSubscriber camera_sub_;
    // 处理用时（毫秒）
    float process_time_ms_ = 0.0f;
    // 解算结果
    std::vector<ArmorPlate> armor_plates_;
    // debug 
    bool debug_base_ = false;
    bool debug_identification_ = false;
    bool debug_preprocessing_ = false;
    DebugParamController debug_controller_;
    bool camera_info_received_ = false;
    std::vector<cv::Point3f> world_points_;

    void init()
    {
        target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");
        lights_.MAX_ANGLE_DIFF = static_cast<float>(this->declare_parameter<double>("max_angle_diff", 10.0));
        lights_.MIN_LENGTH_RATIO = static_cast<float>(this->declare_parameter<double>("min_length_ratio", 0.6));
        lights_.MIN_X_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("min_x_diff_ratio", 0.75));
        lights_.MAX_Y_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("max_y_diff_ratio", 1.0));
        lights_.MAX_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("max_distance_ratio", 0.8));
        lights_.MIN_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("min_distance_ratio", 0.1));

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 
            std::bind(&ArmorPlateIdentification::info, this));

        world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
        world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
        world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 2
        world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 3

        armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        camera_sub_ = image_transport::create_camera_subscription(
            this, "image_raw",
            std::bind(&ArmorPlateIdentification::imageCallback, this,
                      std::placeholders::_1, std::placeholders::_2),
            "raw");
        
        debug_base_ = this->declare_parameter<bool>("debug_base", false);
        debug_identification_ = this->declare_parameter<bool>("debug_identification", false);
        debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);

        RCLCPP_INFO(this->get_logger(), "识别节点已启动，等待 /image_raw ...");
        RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停");
        
        if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
        if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
        if (debug_identification_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
        if (debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
        
        if (debug_identification_ && debug_base_) {
            RCLCPP_INFO(this->get_logger(), "DEBUG模式：1-6选参数  T/G调值  +/-调速度");
        } else if (debug_identification_) {
            RCLCPP_INFO(this->get_logger(), "DEBUG模式：1-6选参数  T/G调值");
        } else if (debug_base_) {
            RCLCPP_INFO(this->get_logger(), "DEBUG模式：+/-调速度");
        }
    }

    void info()
    {
        if (debug_identification_) {
            RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %.2f, MAX_Y_DIFF_RATIO: %.2f, MIN_DISTANCE_RATIO: %.2f, MAX_DISTANCE_RATIO: %.2f, MIN_LENGTH_RATIO: %.2f, MIN_X_DIFF_RATIO: %.2f",
            lights_.MAX_ANGLE_DIFF, lights_.MAX_Y_DIFF_RATIO, lights_.MIN_DISTANCE_RATIO, lights_.MAX_DISTANCE_RATIO, lights_.MIN_LENGTH_RATIO, lights_.MIN_X_DIFF_RATIO
            );
        }
    }

    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
    {
        // 相机初始化
        if (!camera_info_received_ && info_msg) {
            cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
            for (int i = 0; i < 9; ++i) {
                camera_matrix.at<double>(i / 3, i % 3) = info_msg->k[i];
            }
            for (size_t i = 0; i < info_msg->d.size() && i < 5; ++i) {
                distortion_coefficients.at<double>(0, static_cast<int>(i)) = info_msg->d[i];
            }
            pose_solver_ = PoseSolver(world_points_, camera_matrix, distortion_coefficients);
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "CameraInfo 已接收，内参已加载 (fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f)",
                        info_msg->k[0], info_msg->k[4], info_msg->k[2], info_msg->k[5]);
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto t_start = std::chrono::steady_clock::now();

        cv::Mat frame = cv_ptr->image;
        img_show_ = frame.clone();
        Identification(frame);
        SolvePose();
        ImageShow();
        controlParams();
        Publish();

        auto t_end = std::chrono::steady_clock::now();
        process_time_ms_ = static_cast<float>(
            std::chrono::duration<double, std::milli>(t_end - t_start).count());

        if (debug_base_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
        }
    }

    void Identification(cv::Mat& image)
    {
        // 预处理
        cv::Mat gary_thre;
        cv::cvtColor(image, gary_thre, cv::COLOR_BGR2GRAY);
        cv::Mat img_thre;
        cv::threshold(gary_thre, img_thre, 160, 255, cv::THRESH_BINARY);
        cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::dilate(img_thre, img_thre, kernal);
        // 灯条匹配
        lights_.findPairedLights(img_thre, image);
        lights_.drawPairedLights(img_show_);
////////////////////// DEUBG ////////////////////////
        if (debug_preprocessing_) {
            // 预处理四图拼接显示
            // 绘制目标区域
            cv::Mat img_target;
            cv::bitwise_and(image, image, img_target, img_thre);
            std::vector<cv::Mat> images = {image, gary_thre, img_thre, img_target};
            std::vector<std::string> labels = {"Original", "Grayscale", "Threshold", "Target Region"};
            showMultiImages("PreProcessions-View", images, labels);
        }
        if (debug_identification_) {
            debug_controller_.drawParams(img_show_, lights_, process_time_ms_);
            lights_.drawAllLights(img_show_);
            debug_controller_.drawDebugInfo(img_show_, debug_base_);
        }
    }

    void SolvePose()
    {
        std::vector<ArmorPlate> armor_plates;
        for (const auto& points : lights_.getPairedLightPoints()) {
            pose_solver_.solve(points);
            ArmorPlate armor_plate;
            cv::Mat tvec = pose_solver_.getTvec();
            Eigen::Quaternion q = pose_solver_.getQuaternion();
            geometry_msgs::msg::Pose pose;
            pose.position.x = tvec.at<double>(0) / 1000;
            pose.position.y = tvec.at<double>(1) / 1000;
            pose.position.z = tvec.at<double>(2) / 1000;
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            float image_distance = pose_solver_.getImageDistanceToCenter();
            armor_plate.pose = pose;
            armor_plate.image_distance_to_center = image_distance;
            armor_plates.push_back(armor_plate);
        }
        armor_plates_ = armor_plates;
    }
    
    void Publish()
    {
        int index = 0;
        builtin_interfaces::msg::Time stamp = this->now();

        for (const auto& armor_plate : armor_plates_)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = stamp;
            transformStamped.header.frame_id = "camera_link";
            transformStamped.child_frame_id = "armor_plate_" + std::to_string(index++);
            transformStamped.transform.translation.x = armor_plate.pose.position.x;
            transformStamped.transform.translation.y = armor_plate.pose.position.y;
            transformStamped.transform.translation.z = armor_plate.pose.position.z;
            transformStamped.transform.rotation.x = armor_plate.pose.orientation.x;
            transformStamped.transform.rotation.y = armor_plate.pose.orientation.y;
            transformStamped.transform.rotation.z = armor_plate.pose.orientation.z;
            transformStamped.transform.rotation.w = armor_plate.pose.orientation.w;
            tf_broadcaster_->sendTransform(transformStamped);
        }

        ArmorPlates armor_plates_msg;
        armor_plates_msg.header.stamp = stamp;
        armor_plates_msg.header.frame_id = "camera_link";
        armor_plates_msg.armor_plates = armor_plates_;
        armor_plates_pub_->publish(armor_plates_msg);
        ////////////////// DEBUG ////////////////////////
        if (debug_image_pub_->get_subscription_count() > 0 || debug_image_pub_->get_intra_process_subscription_count() > 0) {
            cv::Mat undistorted = pose_solver_.undistortImage(img_show_);
            cv::Mat resized;
            cv::resize(undistorted, resized, cv::Size(), 0.5, 0.5);
            auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized);
            cv_img.header.stamp = stamp;
            cv_img.header.frame_id = "camera_link";
            debug_image_pub_->publish(*cv_img.toImageMsg());
        }
    }

    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;
        
        if (key == 27) 
        {
            cv::destroyAllWindows();
            rclcpp::shutdown();
            return;
        }
        
        if (key == 'p' || key == 'P') {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
            return;
        }

        if (debug_base_) {
            if (debug_controller_.handleKey(key, lights_, this->get_logger())) {
                return;
            }
        }
    }

    void ImageShow()
    {
        cv::imshow("Detection Result", img_show_);
    }
    
public:
    ArmorPlateIdentification() : Node("armor_plate_identification_node")
    {
        init();
    }
    
    ~ArmorPlateIdentification()
    {
        cv::destroyAllWindows();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateIdentification>());
    rclcpp::shutdown();
    return 0;
}
