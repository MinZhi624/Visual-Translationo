#pragma once
#include "armor_plate_identification/Armor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/DebugIdentifaction.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/NumberClassifier.hpp"
#include "armor_plate_identification/CameraDriver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

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

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    cv::Mat img_show_;
    Detector lights_;
    PoseSolver pose_solver_;
    CameraDriver camera_driver_;
    std::string target_color_;
    std::string camera_type_;
    std::string camera_frame_id_ = "camera_link";
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    builtin_interfaces::msg::Time read_stamp_;
    float process_time_ms_ = 0.0f;
    std::vector<Armor> armors_;
    std::vector<ArmorPlate> armor_plates_;
    bool debug_base_ = false;
    bool debug_identification_ = false;
    bool debug_preprocessing_ = false;
    bool debug_number_classification_ = false;
    DebugParamController debug_controller_;
    NumberRoiCollector roi_collector_;
    PreprocessDebug preprocess_debug_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    float process_time_sum_ = 0.0f;
    float id_time_sum_ = 0.0f;
    float id_split_sum_ = 0.0f;
    float id_detect_sum_ = 0.0f;
    int process_time_count_ = 0;
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    std::mutex tracker_debug_mutex_;
    std::deque<ImageSave> img_buffs_;

    void init();
    void Identification(cv::Mat& img_bgr);
    void SolvePose();
    void NumberClassify();
    void Publish();
    void controlParams();
    void ImageShow();
    void Save();
    void TrackerDebugCallBack(const TrackerDebug::SharedPtr msg);

public:
    ArmorPlateIdentification();
    ~ArmorPlateIdentification();
    void run();
};
