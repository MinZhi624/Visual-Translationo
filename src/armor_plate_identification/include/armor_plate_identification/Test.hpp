#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/DebugTest.hpp"
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
    std::vector<DetectorArmor> armors_;
    std::vector<ArmorPlate> armor_plates_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;

    // Test 特有
    double fps_ = 50.0;
    DebugTest debug_test_;
    std::string test_name_;

    // TrackerDebug
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    std::mutex tracker_debug_mutex_;
    std::deque<ImageSave> img_buffs_;
    int tracker_debug_count_ = 0;

    void init(const std::string& video_path);
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
    Test(std::string video_path);
    void run();
    void closeTrackerDebugFile();
};
