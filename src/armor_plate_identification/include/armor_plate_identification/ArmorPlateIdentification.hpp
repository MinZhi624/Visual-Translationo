#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/debug/DebugIdentification.hpp"
#include "armor_plate_identification/debug/DebugTracker.hpp"
#include "armor_plate_identification/PoseSolver.hpp"

#include "armor_plate_identification/camera/Camera.hpp"
#include "armor_plate_common/thread_safe_queue.hpp"

#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "armor_plate_identification/GuiWorker.hpp"

#include <mutex>
#include <deque>
#include <condition_variable>
#include <thread>
#include <atomic>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

struct GimbalRecord {
    builtin_interfaces::msg::Time stamp;
    GimbalData data;
};

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    cv::Mat img_show_;
    Detector lights_;
    PoseSolver pose_solver_;
    Camera camera_;
    builtin_interfaces::msg::Time read_stamp_;
    std::vector<DetectorArmor> armors_;
    std::string target_color_;
    DebugIdentification debug_base_;

    rclcpp::Publisher<ArmorPlates>::SharedPtr armor_plates_pub_;
    rclcpp::Subscription<GimbalAngle>::SharedPtr gimbal_angle_sub_;
    
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    // 图像队列
    ThreadSafeQueue<Record, true> img_queue_{50};
    // gimbal队列
    ThreadSafeQueue<GimbalRecord, false> gimbal_queue_{200};
    GimbalRecord gimbal_ahead_;
    GimbalRecord gimbal_behind_;
    GimbalData matched_gimbal_;
    bool gimbal_has_data_ = false;
    // GUI
    GuiWorker gui_worker_;
    DebugTracker debug_tracker_{&pose_solver_, &gui_worker_};
    bool headless_ = false;
    // Tracker Debug线程
    ThreadSafeQueue<TrackerDebug::SharedPtr, true> tracker_debug_queue_{1};
    std::thread tracker_debug_thread_;
    bool tracker_debug_worker_running_ = false;
    // Camera线程
    ThreadSafeQueue<Frame, true> frame_queue_{1};
    std::thread camera_capture_thread_;
    std::atomic<bool> camera_capture_running_{false};

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
    void cameraCaptureWorker();
    void stopCameraCaptureWorker();
    bool control(const KeyEvent& event);

    void initDebug();
    void initDetector();
    void initPoseSolver();

    builtin_interfaces::msg::Time convertSteadyToRosTime(
        const std::chrono::steady_clock::time_point& steady_stamp);

public:
    ArmorPlateIdentification();
    ~ArmorPlateIdentification();
    void run();
};

