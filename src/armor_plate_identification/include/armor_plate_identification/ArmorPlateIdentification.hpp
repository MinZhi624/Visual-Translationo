#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/DebugBase.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"
#include "armor_plate_identification/CameraDriver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <deque>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

struct GimbalData {
    float yaw_abs = 0.0f;
    float pitch_abs = 0.0f;
};

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    cv::Mat img_show_;
    Detector lights_;
    PoseSolver pose_solver_;
    CameraDriver camera_driver_;
    std::string target_color_;
    std::string camera_type_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Subscription<GimbalAngle>::SharedPtr gimbal_angle_sub_;
    GimbalData gimbal_data_;
    std::mutex gimbal_mutex_;
    builtin_interfaces::msg::Time read_stamp_;
    std::vector<DetectorArmor> armors_;
    std::vector<ArmorPlate> armor_plates_;
    DebugBase debug_base_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    std::mutex tracker_debug_mutex_;
    std::deque<ImageSave> img_buffs_;

    void init();
    void identification(cv::Mat& img_bgr);
    void solvePose();
    void publish();
    void save();
    void show();
    void trackerDebugCallBack(const TrackerDebug::SharedPtr msg);

    void initDebug();
    void initDetector();
    void initPoseSolver();

public:
    ArmorPlateIdentification();
    ~ArmorPlateIdentification();
    void run();
};
