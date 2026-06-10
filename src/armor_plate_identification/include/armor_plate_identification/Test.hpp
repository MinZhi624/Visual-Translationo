#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/GuiWorker.hpp"
#include "armor_plate_identification/debug/DebugTest.hpp"
#include "armor_plate_identification/debug/DebugTracker.hpp"

#include <armor_plate_interfaces/GimbalData.hpp>
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_plate_common/thread_safe_queue.hpp"

#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>


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
    builtin_interfaces::msg::Time read_stamp_;
    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    GimbalData test_gimbal_;

    // Test 特有
    double fps_ = 50.0;
    DebugTest debug_test_;
    std::string test_name_;

    // TrackerDebug
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    ThreadSafeQueue<Record, true> img_queue_{50};
    int tracker_debug_count_ = 0;
    // TrackerDebug线程
    ThreadSafeQueue<TrackerDebug::SharedPtr, true> tracker_debug_queue_{1};
    std::thread tracker_debug_thread_;
    bool tracker_debug_worker_running_ = false;
    // GUI
    GuiWorker gui_worker_;
    DebugTracker debug_tracker_{&pose_solver_, &gui_worker_};
    bool headless_ = false;

    void init(const std::string& video_path);
    void identification(cv::Mat& img_bgr);
    void solvePose();
    void publish();
    void save();
    void show();
    bool control(const KeyEvent& event);
    
    void trackerDebugCallBack(const TrackerDebug::SharedPtr msg);
    void processTrackerDebug(const TrackerDebug::SharedPtr msg);
    void trackerDebugWorker();
    void stopTrackerDebugWorker();

    void initDebug();
    void initDetector();
    void initPoseSolver();
public:
    Test(std::string video_path);
    ~Test();
    void run();
    void closeTrackerDebugFile();
};
