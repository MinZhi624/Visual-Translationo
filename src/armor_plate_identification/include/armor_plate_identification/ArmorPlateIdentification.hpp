#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/DebugIdentification.hpp"
#include "armor_plate_identification/DebugTracker.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "armor_plate_identification/CameraDriver.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "armor_plate_identification/GuiWorker.hpp"

#include <mutex>
#include <deque>
#include <condition_variable>
#include <thread>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

struct GimbalRecord {
    builtin_interfaces::msg::Time stamp;
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
    GimbalRecord gimbal_data_;
    std::deque<GimbalRecord> gimbal_history_;
    std::mutex gimbal_mutex_;
    GimbalData matched_gimbal_;
    builtin_interfaces::msg::Time read_stamp_;
    std::vector<DetectorArmor> armors_;
    DebugIdentification debug_base_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    std::mutex tracker_debug_mutex_;
    std::deque<Record> img_buffs_;

    std::mutex tracker_debug_queue_mutex_;
    std::condition_variable tracker_debug_cv_;
    std::deque<TrackerDebug::SharedPtr> tracker_debug_msgs_;
    std::thread tracker_debug_thread_;
    bool tracker_debug_worker_running_ = false;

    GuiWorker gui_worker_;
    DebugTracker debug_tracker_{&pose_solver_, &gui_worker_};
    bool headless_ = false;
    bool diagnostic_log_ = false;
    double last_gimbal_match_diff_ms_ = 0.0;
    size_t last_gimbal_history_size_ = 0;

    void init();
    void identification(cv::Mat& img_bgr);
    void solvePose();
    void publish();
    void save();
    void show();
    void trackerDebugCallBack(const TrackerDebug::SharedPtr msg);
    void processTrackerDebug(const TrackerDebug::SharedPtr msg);
    void trackerDebugWorker();
    void stopTrackerDebugWorker();
    bool control(const KeyEvent& event);

    void initDebug();
    void initDetector();
    void initPoseSolver();

public:
    ArmorPlateIdentification();
    ~ArmorPlateIdentification();
    void run();
};

