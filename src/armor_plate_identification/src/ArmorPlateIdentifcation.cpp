#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/TestFunc.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"
#include "armor_plate_identification/CameraDriver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    cv::Mat img_show_;
    Detector lights_;
    PoseSolver pose_solver_;
    CameraDriver camera_driver_;
    std::string target_color_; // "RED" 或 "BLUE"
    std::string camera_type_;  // "mindvision" 或 "galaxy"
    std::string camera_frame_id_ = "camera_link";
    // ros相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    // 处理用时（毫秒）
    float process_time_ms_ = 0.0f;
    // 检测结果缓存
    std::vector<Armor> armors_;
    // 解算结果
    std::vector<ArmorPlate> armor_plates_;
    // 数字识别
    NumberClassifier number_classifier_;
    // debug 
    bool debug_base_ = false;
    bool debug_identification_ = false;
    bool debug_preprocessing_ = false;
    bool debug_number_classification_ = false;
    DebugParamController debug_controller_;
    std::vector<cv::Point3f> world_points_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    void init()
    {
        target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");
        lights_.MAX_ANGLE_DIFF = static_cast<float>(this->declare_parameter<double>("max_angle_diff", 10.0));
        lights_.MIN_LENGTH_RATIO = static_cast<float>(this->declare_parameter<double>("min_length_ratio", 0.7));
        lights_.MIN_X_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("min_x_diff_ratio", 0.75));
        lights_.MAX_Y_DIFF_RATIO = static_cast<float>(this->declare_parameter<double>("max_y_diff_ratio", 1.0));
        lights_.MAX_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("max_distance_ratio", 0.8));
        lights_.MIN_DISTANCE_RATIO = static_cast<float>(this->declare_parameter<double>("min_distance_ratio", 0.1));
        lights_.TARGET_COLOR = target_color_;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 
            std::bind(&ArmorPlateIdentification::info, this));

        world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
        world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
        world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 2
        world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 3

        // ===== 初始化数字识别 ===== //
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
        std::string model_relative_path = this->declare_parameter<std::string>("model_path", "");
        std::string label_relative_path = this->declare_parameter<std::string>("label_path", "");
        std::string model_path = package_share_dir + "/" + model_relative_path;
        std::string label_path = package_share_dir + "/" + label_relative_path;
        double number_threshold = this->declare_parameter<double>("number_threshold", 0.15);
        std::vector<std::string> empty;
        std::vector<std::string> ignore_labels = this->declare_parameter<std::vector<std::string>>("ignore_labels", empty);
        number_classifier_ = NumberClassifier(model_path, label_path, number_threshold, ignore_labels);

        armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
        camera_type_ = this->declare_parameter<std::string>("camera_type", "galaxy");
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        if (camera_type_ == "galaxy") {
            camera_frame_id_ = "camera_optical_frame";
        } else {
            camera_frame_id_ = "camera_link";
        }
        double exposure_time = this->declare_parameter<double>("exposure_time", 3500.0);
        double gain = this->declare_parameter<double>("gain", 1.0);
        if (!camera_driver_.init(camera_type_, exposure_time, gain)) {
            RCLCPP_FATAL(this->get_logger(), "相机初始化失败，程序退出");
            rclcpp::shutdown();
            return;
        }
        // ===== PoseSolver ===== //
        camera_info_msg_ = camera_driver_.getCameraInfo();
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
        cv::Mat projection_matrix = cv::Mat::zeros(3, 4, CV_64F);
        for (int i = 0; i < 9; ++i) {
            camera_matrix.at<double>(i / 3, i % 3) = camera_info_msg_.k[i];
        }
        for (size_t i = 0; i < camera_info_msg_.d.size() && i < 5; ++i) {
            distortion_coefficients.at<double>(0, static_cast<int>(i)) = camera_info_msg_.d[i];
        }
        for (int i = 0; i < 12; ++i) {
            projection_matrix.at<double>(i / 4, i % 4) = camera_info_msg_.p[i];
        }
        pose_solver_ = PoseSolver(world_points_, camera_matrix, distortion_coefficients, projection_matrix);
        // ===== DEUBG ===== //
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
        debug_base_ = this->declare_parameter<bool>("debug_base", false);
        debug_identification_ = this->declare_parameter<bool>("debug_identification", false);
        debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
        debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
        // ====== 打印信息 ====== //
         RCLCPP_INFO(this->get_logger(), "识别节点已启动，相机类型: %s, frame_id: %s", camera_type_.c_str(), camera_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "识别节点已启动，相机类型: %s, frame_id: %s", camera_type_.c_str(), camera_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停");
        if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
        if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
        if (debug_identification_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
        if (debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
        if (debug_number_classification_) RCLCPP_INFO(this->get_logger(), "数字识别DEBUG模式开启");
        
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
        lights_.detectArmors(img_thre, image);
        lights_.drawArmors(img_show_);
        armors_ = lights_.getArmors();
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
            debug_controller_.drawParams(img_show_, lights_);
            lights_.drawAllLights(img_show_);
            debug_controller_.drawDebugInfo(img_show_, debug_base_);
        }
        if (debug_number_classification_) {
            lights_.showNumberBinaryROI();
            lights_.showNumberROI();
        }
    }

    void SolvePose()
    {
        std::vector<ArmorPlate> armor_plates;
        for (const auto& armor : armors_) {
            pose_solver_.solve(armor.points_);
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

    void NumberClassify()
    {
        number_classifier_.classify(armors_);
        // 保存数据
        for (size_t i = 0; i < armors_.size() && i < armor_plates_.size(); i++) {
            armor_plates_[i].number = armors_[i].number_;
        }
        ////////// DEBUG /////////
        if (debug_number_classification_) { 
            drawAllNumberTest(img_show_, armors_);
        }
    }
    
    void Publish()
    {
        builtin_interfaces::msg::Time stamp = this->now();

        // 发布装甲板数据
        ArmorPlates armor_plates_msg;
        armor_plates_msg.header.stamp = stamp;
        armor_plates_msg.header.frame_id = camera_frame_id_;
        armor_plates_msg.armor_plates = armor_plates_;
        armor_plates_pub_->publish(armor_plates_msg);
        // 发布 camera_info
        camera_info_msg_.header.stamp = stamp;
        camera_info_msg_.header.frame_id = camera_frame_id_;
        camera_info_pub_->publish(camera_info_msg_);
        ////////////////// DEBUG ////////////////////////
        if (debug_image_pub_->get_subscription_count() > 0 || debug_image_pub_->get_intra_process_subscription_count() > 0) {
            cv::Mat undistorted = pose_solver_.undistortImage(img_show_);
            cv::Mat resized;
            cv::resize(undistorted, resized, cv::Size(), 0.5, 0.5);
            auto cv_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized);
            cv_img.header.stamp = stamp;
            cv_img.header.frame_id = camera_frame_id_;
            debug_image_pub_->publish(*cv_img.toImageMsg());
        }
    }

    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;
        if (key == 27 || key == 'q' || key == 'Q') 
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
        if (key == 's' || key == 'S') {
            lights_.setSave(true);
        }
        if (debug_base_) {
            if (debug_controller_.handleKey(key, lights_, this->get_logger())) {
                return;
            }
        }
    }
    void ImageShow()
    {
        debug_controller_.drawProcessTime(img_show_, process_time_ms_);
        cv::imshow("Identifacation", img_show_);
    }
    void Save()
    {
        ////////// DEBUG ///////////
        if(debug_number_classification_) {
            lights_.saveNumberRoi();
        }        
    }
public:
    ArmorPlateIdentification() : Node("armor_plate_identification_node"), camera_driver_(this)
    {
        init();
    }
    
    ~ArmorPlateIdentification()
    {
        camera_driver_.close();
        cv::destroyAllWindows();
    }
    
    void run()
    {
        RCLCPP_INFO(this->get_logger(), "开始图像处理循环");
        int fail_count = 0;
        while (rclcpp::ok()) {
            cv::Mat frame = camera_driver_.Read();
            if (frame.empty()) {
                fail_count++;
                if (fail_count > 5) {
                    RCLCPP_FATAL(this->get_logger(), "Camera read failed!");
                    rclcpp::shutdown();
                }
                continue;
            }
            fail_count = 0;
            
            auto t_start = std::chrono::steady_clock::now();
            img_show_ = frame.clone();
            Identification(frame);
            SolvePose();
            NumberClassify();
            Publish();
            Save();
            auto t_end = std::chrono::steady_clock::now();
            process_time_ms_ = static_cast<float>(std::chrono::duration<double, std::milli>(t_end - t_start).count());
            ImageShow();
            controlParams();
            if (debug_base_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorPlateIdentification>();
    std::thread spin_thread([&]() { rclcpp::spin(node); });
    node->run();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 0;
}
