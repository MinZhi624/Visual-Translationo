#pragma once
#include "armor_plate_identification/Armor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/DebugIdentifaction.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <deque>
#include <thread>
#include <chrono>
#include <mutex>
#include <filesystem>
#include <algorithm>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackerDebug;

class Test : public rclcpp::Node
{
private:
    cv::VideoCapture c_;
    cv::Mat img_show_;
    std::string target_color_;
    Detector lights_;
    PoseSolver pose_solver_;
    std::vector<Armor> armors_;
    std::vector<ArmorPlate> armor_plates_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    // DEBUG //
    float process_time_ms_ = 0.0f;
    float process_time_sum_ = 0.0f;
    float id_time_sum_ = 0.0f;
    float id_split_sum_ = 0.0f;
    float id_detect_sum_ = 0.0f;
    int process_time_count_ = 0;
    double aabb_time_sum_us_ = 0.0;
    int roi_count_ = 0;
    int frame_count_ = 0;
    double fps_ = 50.0;
    bool debug_base_;
    bool debug_identification_;
    bool debug_preprocessing_;
    bool debug_number_classification_;
    bool debug_frame_;
    int debug_frame_count_;
    DebugParamController debug_controller_;
    NumberRoiCollector roi_collector_;
    PreprocessDebug preprocess_debug_;
    bool headless_;
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    std::mutex tracker_debug_mutex_;
    std::deque<ImageSave> img_buffs_;
    std::string test_name_;

    void init(const std::string& video_path);
    void Identification(cv::Mat& img_bgr);
    void SolvePose();
    void NumberClassify();
    void Publish();
    void controlParams();
    void imageShow();
    void Save();
    void TrackerDebugCallBack(const TrackerDebug::SharedPtr msg);

public:
    Test(std::string video_path);
    void run();
};
