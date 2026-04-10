// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/TestFunc.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/Tracker.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <thread>
#include <chrono>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

class Test : public rclcpp::Node
{
private:
    cv::VideoCapture c_;
    cv::Mat img_show_;
    PairedLights lights_;
    PoseSolver pose_solver_;
    Tracker tracker_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> yaw_;
    std::vector<float> pitch_;
    // PoseSolver结果
    std::vector<ArmorPlate> armor_plates_;
    // 发布者
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // 时间和帧数
    double fps_;
    int frame_count_;           // 当前帧数->用于每秒多少fps
    double current_time_;       // 当前时间（秒）= frame_count_ / fps_
    
#ifdef DEBUG_BASE
    int play_delay_ms_ = 0; // 播放延迟，越大越慢
    int x = 10, y = 30, line_h = 25;
#endif
    DebugParamController debug_controller_;

    void init(const std::string& video_path)
    {
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
        lights_.MIN_LENGTH_RATIO = 0.6f;
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
        
        // ===== 初始化Tracker ===== //
        tracker_.setMaxLostTime(0.5);  // 最大丢失0.5秒
        tracker_.setMutationThreshold(3.0f, 2.0f);  // yaw突变3度，pitch突变2度
        tracker_.Init();
        // ===== 初始化发布器 ===== //
        armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
#endif
#ifdef DEBUG_PREPROCESSING
        RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
#endif
#ifdef DEBUG_POSE
        RCLCPP_INFO(this->get_logger(), "姿态估计DEBUG模式开启");
#endif
    }
    void info()
    {
        // 计时器 5s发布一次信息
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %f, MAX_Y_DIFF_RATIO: %f, MIN_DISTANCE_RATIO: %f, MAX_DISTANCE_RATIO: %f, MIN_LENGTH_RATIO: %f, MIN_X_DIFF_RATIO: %f",
        lights_.MAX_ANGLE_DIFF, lights_.MAX_Y_DIFF_RATIO, lights_.MIN_DISTANCE_RATIO, lights_.MAX_DISTANCE_RATIO, lights_.MIN_LENGTH_RATIO, lights_.MIN_X_DIFF_RATIO
        );
#endif
    }
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights_.findPairedLights(img_thre);
        lights_.drawPairedLights(img_show_);
        
#ifdef DEBUG_PREPROCESSING
        // 预处理四图拼接显示
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);
#endif
#ifdef DEBUG_INDENTIFICATION
        debug_controller_.drawParams(img_show_, lights_, x, y, line_h);
        lights_.drawAllLights(img_show_);
        debug_controller_.drawDebugInfo(img_show_, play_delay_ms_, true, x, y, line_h);
#endif
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
#ifdef DEBUG_POSE
            pose_solver_.drawPose(img_show_);            
#endif
        }
        yaw_ = yaw;
        pitch_ = pitch;
        armor_plates_ = armor_plates;
    }
    void Track()
    {
        // 更新时间：frame_count / fps
        current_time_ = frame_count_ / fps_;
        
        // 使用Tracker进行滤波跟踪
        tracker_.Update(yaw_, pitch_, current_time_);
        
        // 获取滤波后的值
        float tracked_yaw = tracker_.getYaw();
        float tracked_pitch = tracker_.getPitch();
        
#ifdef DEBUG_POSE
        // 显示时间和帧数
        cv::putText(img_show_, "Time: " + std::to_string(current_time_) + "s Frame: " + std::to_string(frame_count_), 
                   cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2);
        
        // 在图像上显示跟踪结果
        if (tracker_.isLost()) {
            // 丢失目标时显示预测值（红色）
            cv::putText(img_show_, "Tracked Yaw: " + std::to_string(tracked_yaw), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2);
            cv::putText(img_show_, "Tracked Pitch: " + std::to_string(tracked_pitch), 
                       cv::Point(10, 85), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2);
            cv::putText(img_show_, "Lost: " + std::to_string(tracker_.getLostTime(current_time_)) + "s", 
                       cv::Point(10, 110), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2);
        } else {
            // 正常跟踪时显示滤波值（绿色）
            cv::putText(img_show_, "Tracked Yaw: " + std::to_string(tracked_yaw), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 2);
            cv::putText(img_show_, "Tracked Pitch: " + std::to_string(tracked_pitch), 
                       cv::Point(10, 85), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 2);
        }
        
        // 可视化追踪点：将滤波后的角度反投影到图像
        drawTrackedPoint(tracked_yaw, tracked_pitch);
#endif
    }
    void drawTrackedPoint(float tracked_yaw, float tracked_pitch)
    {
        // 获取当前距离（从最后一次解算）
        float distance = pose_solver_.getDistance();
        if (distance <= 0) return;
        
        // 从 yaw/pitch/distance 重建 tvec
        double yaw_rad = tracked_yaw * CV_PI / 180.0;
        double pitch_rad = tracked_pitch * CV_PI / 180.0;
        
        cv::Point3f tracked_tvec;
        tracked_tvec.x = static_cast<float>(distance * std::sin(yaw_rad) * std::cos(pitch_rad));
        tracked_tvec.y = static_cast<float>(-distance * std::sin(pitch_rad));
        tracked_tvec.z = static_cast<float>(distance * std::cos(yaw_rad) * std::cos(pitch_rad));
        
        // 重投影到图像
        cv::Point2f tracked_point = pose_solver_.reprojection(tracked_tvec);
        
        // 画追踪点（黄色圆圈+十字）
        cv::circle(img_show_, tracked_point, 8, cv::Scalar(0, 255, 255), 2);
        cv::circle(img_show_, tracked_point, 3, cv::Scalar(0, 255, 255), -1);
        int cross_len = 12;
        cv::line(img_show_, 
                 cv::Point(tracked_point.x - cross_len, tracked_point.y),
                 cv::Point(tracked_point.x + cross_len, tracked_point.y),
                 cv::Scalar(0, 255, 255), 2);
        cv::line(img_show_, 
                 cv::Point(tracked_point.x, tracked_point.y - cross_len),
                 cv::Point(tracked_point.x, tracked_point.y + cross_len),
                 cv::Scalar(0, 255, 255), 2);
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

#ifdef DEBUG_BASE
        if (debug_controller_.handleKey(key, lights_, play_delay_ms_, this->get_logger())) {
            return;
        }
#endif
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
            // 跟踪滤波
            Track();
            // 图片展示
            cv::imshow("img_show_", img_show_);
            // 按键控制
            controlParams();
            // 打印
            rclcpp::spin_some(this->get_node_base_interface());
            // 发布数据
            Publish();
            // 控制速度
#ifdef DEBUG_BASE
            std::this_thread::sleep_for(std::chrono::milliseconds(play_delay_ms_));
#endif
        }
        RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请输入正确的参数，输入视频地址");
        return 1;
    }
    auto node = std::make_shared<Test>(argv[1]);
    rclcpp::shutdown();
    return 0;
}
