#include "armor_plate_identification/CameraDriver.hpp"
#include "armor_plate_identification/Recognize.hpp"
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

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    ArmorCameraCapture camera;
    cv::Mat img_show;
    PairedLights lights;
    PoseSolver pose_solver_;
    // 可调参数
    int exposure_time = 300;  // 初始曝光时间（微秒）
    int gain = 20;             // 初始增益
    std::string target_color_; // "RED" 或 "BLUE"
    // ros相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // 当前时间（秒）
    double current_time_ = 0.0;
    // 处理用时（毫秒）
    float process_time_ms_ = 0.0f;
    // 解算结果
    std::vector<ArmorPlate> armor_plates_;
    // debug 
    bool debug_base_ = false;
    bool debug_identification_ = false;
    bool debug_preprocessing_ = false;
    DebugParamController debug_controller_;

    // 初始化
    void init()
    {
        // 打开相机
        if (!camera.open()) {
            RCLCPP_ERROR(this->get_logger(), "相机打开失败,请检查相机是否连接正确！");
            return;
        }
        
        // 设置为灯条识别优化的低曝光模式
        if (camera.setLowExposureForLightBar(exposure_time, gain)) {
            RCLCPP_INFO(this->get_logger(), "低曝光模式设置成功，当前曝光: %.0f us, 增益: %d", 
                        camera.getExposureTime(), camera.getGain());
        } else {
            RCLCPP_WARN(this->get_logger(), "低曝光模式设置失败");
        }

        // 匹配参数初始化（ROS参数化）
        target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");
        lights.MAX_ANGLE_DIFF = static_cast<float>(this->declare_parameter<double>("max_angle_diff", 10.0));
        lights.MIN_LENGTH_RATIO = static_cast<float>(this->declare_parameter<double>("min_length_ratio", 0.6));
        lights.MIN_X_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("min_x_diff_ratio", 0.75));
        lights.MAX_Y_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("max_y_diff_ratio", 1.0));
        lights.MAX_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("max_distance_ratio", 0.8));
        lights.MIN_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("min_distance_ratio", 0.1));
        // 创建定时器，5秒发布一次信息
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 
            std::bind(&ArmorPlateIdentification::info, this));
        // ===== 初始化PoseSolver ===== //
        std::vector<cv::Point3f> world_points_;
        world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
        world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
        world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 2
        world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 3
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            2374.54248, 0., 698.85288,
            0., 2377.53648, 520.8649,
            0., 0., 1.);
        cv::Mat distortion_coefficients_ = (cv::Mat_<double>(1, 5) <<
            -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
        pose_solver_ = PoseSolver(world_points_, camera_matrix_, distortion_coefficients_);
        // ===== 初始化发布器 ===== //
        armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        current_time_ = 0.0;
        
        // ===== DEBUG ===== //
        debug_base_ = this->declare_parameter<bool>("debug_base", false);
        debug_identification_ = this->declare_parameter<bool>("debug_identification", false);
        debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);

        // 操作说明
        RCLCPP_INFO(this->get_logger(), "相机启动成功");
        RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停  W/S-曝光  A/D-增益");
        
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
    // 定时器回调，发布信息
    void info()
    {
        if (debug_identification_) {
            RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %.2f, MAX_Y_DIFF_RATIO: %.2f, MIN_DISTANCE_RATIO: %.2f, MAX_DISTANCE_RATIO: %.2f, MIN_LENGTH_RATIO: %.2f, MIN_X_DIFF_RATIO: %.2f",
            lights.MAX_ANGLE_DIFF, lights.MAX_Y_DIFF_RATIO, lights.MIN_DISTANCE_RATIO, lights.MAX_DISTANCE_RATIO, lights.MIN_LENGTH_RATIO, lights.MIN_X_DIFF_RATIO
            );
        }
    }
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image, target_color_);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights.findPairedLights(img_thre);
        lights.drawPairedLights(img_show);
        
        // 图像显示逻辑
        if (debug_preprocessing_) {
            // DEBUG_PREPROCESSING 模式：显示预处理四图
            cv::Mat img_target;
            cv::bitwise_and(image, image, img_target, img_thre);
            std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
            std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
            showMultiImages("PreProcessions-View", images, labels);
        }
        if (debug_identification_) {
            debug_controller_.drawParams(img_show, lights, process_time_ms_);
            lights.drawAllLights(img_show);
            debug_controller_.drawDebugInfo(img_show, debug_base_);
        }
    }
    void SolvePose()
    {
        std::vector<ArmorPlate> armor_plates;
        // 每一个匹配好的灯条解算
        for (const auto& points : lights.getPairedLightPoints()) {
            pose_solver_.solve(points);
            // 存储解算结果
            ArmorPlate armor_plate;
            cv::Mat tvec = pose_solver_.getTvec();
            Eigen::Quaternion q = pose_solver_.getQuaternion();
            geometry_msgs::msg::Pose pose;
            // mm -> m
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
            // 发布姿态
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

        // 发布灯条信息
        ArmorPlates armor_plates_msg;
        armor_plates_msg.header.stamp = stamp;
        armor_plates_msg.header.frame_id = "camera_link";
        armor_plates_msg.armor_plates = armor_plates_;  // 直接拷贝赋值
        armor_plates_pub_->publish(armor_plates_msg);
        
        // 发布调试图像
        if (debug_image_pub_->get_subscription_count() > 0 || debug_image_pub_->get_intra_process_subscription_count() > 0) {
            auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_show);
            cv_img.header.stamp = stamp;
            cv_img.header.frame_id = "camera_link";
            debug_image_pub_->publish(*cv_img.toImageMsg());
        }
    }

    // 控制参数函数
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;
        
        // ESC：退出（最高优先级）
        if (key == 27) 
        {
            camera.release();
            cv::destroyAllWindows();
            exit(0);
        }
        
        // P：暂停（通用功能）
        if (key == 'p' || key == 'P') {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
            return;
        }

        if (debug_base_) {
            if (debug_controller_.handleKey(key, lights, this->get_logger())) {
                return;
            }
        }
        // 相机参数控制
        // W/S：曝光时间
        if (key == 'w' || key == 'W') {
            exposure_time = std::min(exposure_time + 100, 10000);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
            return;
        }
        if (key == 's' || key == 'S') {
            exposure_time = std::max(exposure_time - 100, 100);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
            return;
        }
        
        // A/D：增益
        if (key == 'a' || key == 'A') {
            gain = std::min(gain + 10, 400);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
            return;
        }
        if (key == 'd' || key == 'D') {
            gain = std::max(gain - 10, 0);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
            return;
        }
    }

    // 可视化
    void ImageShow()
    {
        // 在 img_show 左下角显示相机参数
        std::string params_text = "Exp: " + std::to_string(exposure_time) + "us  Gain: " + std::to_string(gain);
        // 左下角位置（留 10px 边距）
        cv::Point pos(10, img_show.rows - 10);
        cv::putText(img_show, params_text, pos,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        // 显示带参数的检测结果图
        cv::imshow("Detection Result", img_show);
    }
    
public:
    ArmorPlateIdentification() : Node("armor_plate_identification_node")
    {
        init();
        cv::Mat frame;
        while (true)
        {
            if (!camera.read(frame)) break;
            if (!rclcpp::ok()) break;
            
            // 更新时间
            current_time_ = this->now().seconds();
            
            // 开始计时
            auto t_start = std::chrono::steady_clock::now();
            
            // 图像处理
            img_show = frame.clone();
            Identification(frame);
            
            // 解算
            SolvePose();
            
            // 可视化
            ImageShow();
            controlParams();

            // 让 ROS2 处理定时器等事件
            rclcpp::spin_some(this->get_node_base_interface());
            
            // 发布数据
            Publish();
            
            // 结束计时
            auto t_end = std::chrono::steady_clock::now();
            process_time_ms_ = static_cast<float>(
                std::chrono::duration<double, std::milli>(t_end - t_start).count());

            // 控制速度
            if (debug_base_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
            }
        }
        camera.release();
        cv::destroyAllWindows();
    }
    
    ~ArmorPlateIdentification()
    {
        camera.release();
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
