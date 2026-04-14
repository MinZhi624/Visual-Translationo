// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/TestFunc.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <thread>
#include <chrono>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

class Test : public rclcpp::Node
{
private:
    // 相机相关
    cv::VideoCapture c_;
    cv::Mat img_show_;
    // 灯条匹配相关
    PairedLights lights_;
    PoseSolver pose_solver_;
    // PoseSolver结果
    std::vector<ArmorPlate> armor_plates_;
    std::vector<float> yaw_;
    std::vector<float> pitch_;
    // 发布者
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // 时间和帧数
    rclcpp::TimerBase::SharedPtr timer_;
    double fps_;
    int frame_count_;           // 当前帧数->用于每秒多少fps
    double current_time_;       // 当前时间（秒）= frame_count_ / fps_

    // ROS 参数控制 debug 开关
    bool debug_base_ = false;
    bool debug_identification_ = false;
    bool debug_preprocessing_ = false;
    bool debug_pose_ = false;
    DebugParamController debug_controller_;

    void init(const std::string& video_path)
    {
        // ==== 读取 ROS 参数 ==== //
        this->declare_parameter("debug_base", false);
        this->declare_parameter("debug_identification", false);
        this->declare_parameter("debug_preprocessing", false);
        this->get_parameter("debug_base", debug_base_);
        this->get_parameter("debug_identification", debug_identification_);
        this->get_parameter("debug_preprocessing", debug_preprocessing_);
        // ==== 相机初始化 ==== //
        c_.open(video_path);
        if (!c_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path.c_str());
            rclcpp::shutdown();
            return;
        }
        // 获取视频FPS
        fps_ = c_.get(cv::CAP_PROP_FPS);
        if (fps_ <= 0) {
            fps_ = 50.0;  // 默认50fps
            RCLCPP_WARN(this->get_logger(), "无法获取视频FPS，使用默认值: %.1f", fps_);
        } else {
            RCLCPP_INFO(this->get_logger(), "视频FPS: %.2f", fps_);
        }
        // 初始化时间和帧数
        frame_count_ = 0;
        current_time_ = 0.0;
        // ==== 匹配参数初始化 ==== //
        lights_.MAX_ANGLE_DIFF = 10.0f;
        lights_.MIN_LENGTH_RATIO = 0.70f;
        lights_.MIN_X_DIFF_RATIO = 0.75f;
        lights_.MAX_Y_DIFF_RATIO = 1.0f;
        lights_.MAX_DISTANCE_RATIO = 0.8f;
        lights_.MIN_DISTANCE_RATIO = 0.1f;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),  std::bind(&Test::info, this));
        // ===== 初始化PoseSolver ===== //
        // 装甲板坐标系点左上角是0,顺时针排列
        std::vector<cv::Point3f> world_points_;
        world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
        world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
        world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 2
        world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 3
        // 初始化相机内参
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            2374.54248, 0., 698.85288,
            0., 2377.53648, 520.8649,
            0., 0., 1.);
        // 相机畸变系数
        cv::Mat distortion_coefficients_ = (cv::Mat_<double>(1, 5) <<
            -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
        pose_solver_ = PoseSolver(world_points_,camera_matrix_, distortion_coefficients_);
        // ===== 初始化发布器 ===== //
        armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // ===== DEBUG信息 ===== //
        if (debug_identification_) {
            RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
        }
        if (debug_preprocessing_) {
            RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
        }
        if (debug_pose_) {
            RCLCPP_INFO(this->get_logger(), "姿态估计DEBUG模式开启");
        }
    }
    void info()
    {
        // 计时器 5s发布一次信息
        if (debug_identification_) {
            RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %f, MAX_Y_DIFF_RATIO: %f, MIN_DISTANCE_RATIO: %f, MAX_DISTANCE_RATIO: %f, MIN_LENGTH_RATIO: %f, MIN_X_DIFF_RATIO: %f",
            lights_.MAX_ANGLE_DIFF, lights_.MAX_Y_DIFF_RATIO, lights_.MIN_DISTANCE_RATIO, lights_.MAX_DISTANCE_RATIO, lights_.MIN_LENGTH_RATIO, lights_.MIN_X_DIFF_RATIO
            );
        }
    }
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights_.findPairedLights(img_thre);
        lights_.drawPairedLights(img_show_);
        
        if (debug_preprocessing_) {
            // 预处理四图拼接显示
            // 绘制目标区域
            cv::Mat img_target;
            cv::bitwise_and(image, image, img_target, img_thre);
            std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
            std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
            showMultiImages("PreProcessions-View", images, labels);
        }
        if (debug_identification_) {
            debug_controller_.drawParams(img_show_, lights_);
            lights_.drawAllLights(img_show_);
            debug_controller_.drawDebugInfo(img_show_, debug_base_);
        }
    }
    void SolvePose()
    {
        std::vector<float> yaw;
        std::vector<float> pitch;
        std::vector<ArmorPlate> armor_plates;
        // 每一个匹配好的灯条解算
        for (const auto& points : lights_.getPairedLightPoints()) {
            pose_solver_.solve(points);
            yaw.push_back(pose_solver_.getYaw());
            pitch.push_back(pose_solver_.getPitch());
            // 存储解算结果
            ArmorPlate armor_plate;
            cv::Mat tvec = pose_solver_.getTvec();
            Eigen::Quaternion q = pose_solver_.getQuaternion();
            geometry_msgs::msg::Pose pose;
            //mm -> m
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
            if (debug_pose_) {
                pose_solver_.drawPose(img_show_);            
            }
        }
        yaw_ = yaw;
        pitch_ = pitch;
        armor_plates_ = armor_plates;
    }
    void Publish()
    {
        int index = 0;
        // 使用视频的模拟时间（frame_count / fps）作为消息时间戳
        double sim_time = frame_count_ / fps_;
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(sim_time);
        stamp.nanosec = static_cast<uint32_t>((sim_time - stamp.sec) * 1e9);

        for(const auto& armor_plate : armor_plates_)
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
            auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_show_);
            cv_img.header.stamp = stamp;
            cv_img.header.frame_id = "camera_link";
            debug_image_pub_->publish(*cv_img.toImageMsg());
        }
    }
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;

        // ESC：退出
        if (key == 27) {
            rclcpp::shutdown();
            return;
        }

        // P：暂停
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
public:
    Test(std::string video_path) : Node("test_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
        init(video_path);

        cv::Mat frame;
        while(true)
        {
            c_ >> frame;
            if (frame.empty()) {
                RCLCPP_INFO(this->get_logger(), "视频播放结束");
                return;
            }
            if (!rclcpp::ok()) return;
            
            // 更新帧数
            frame_count_++;
            img_show_ = frame.clone();
            // 图像处理
            Identification(frame);
            // 解算
            SolvePose();
            // （本地 Tracker 已禁用，由 armor_plate_tracker 节点处理）
            // 图片展示
            cv::imshow("img_show_", img_show_);
            // 按键控制
            controlParams();
            // 打印
            rclcpp::spin_some(this->get_node_base_interface());
            // 发布数据
            Publish();
            // 控制速度
            if (debug_base_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
            }
        }
        RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请输入正确的参数，输入视频地址");
        return 1;
    }
    auto node = std::make_shared<Test>(argv[1]);
    rclcpp::shutdown();
    return 0;
}
