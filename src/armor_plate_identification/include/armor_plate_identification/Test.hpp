#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/GuiWorker.hpp"
#include "armor_plate_identification/KeyFrameCache.hpp"
#include "armor_plate_identification/debug/DebugTest.hpp"
#include "armor_plate_identification/debug/DebugTracker.hpp"

#include <armor_plate_interfaces/GimbalData.hpp>
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/tracked_targets.hpp"
#include "armor_plate_interfaces/msg/planner_debug.hpp"
#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_plate_common/thread_safe_queue.hpp"

#include <atomic>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>


using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using armor_plate_interfaces::msg::TrackerDebug;
using armor_plate_interfaces::msg::PlannerDebug;
using armor_plate_identification::KeyFrameCache;
using armor_plate_identification::KeyFrame;
using armor_plate_identification::KeyFrameRecord;

namespace armor_plate_identification {
namespace debug {
class YawSearchBenchmark;
}  // namespace debug
}  // namespace armor_plate_identification

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
    rclcpp::Publisher<armor_plate_interfaces::msg::GimbalAngle>::SharedPtr gimbal_angle_pub_;
    GimbalData test_gimbal_;

    // Test 特有
    double fps_ = 50.0;
    DebugTest debug_test_;
    std::string test_name_;

    // KeyFrameCache（同步帧、TrackerDebug、PlannerDebug）
    std::unique_ptr<KeyFrameCache> keyframe_cache_;

    // TrackerDebug
    rclcpp::Subscription<TrackerDebug>::SharedPtr tracker_debug_sub_;
    int tracker_debug_count_ = 0;

    // ===== 消息统计（auto_test 验证用） =====
    struct MsgStats {
        int tracked_targets_count = 0;
        int tracker_debug_count = 0;
        int planner_debug_count = 0;
        int aim_command_count = 0;
        int aim_command_invalid_count = 0;
        builtin_interfaces::msg::Time last_tracked_targets_stamp;
        builtin_interfaces::msg::Time last_planner_debug_stamp;
        builtin_interfaces::msg::Time last_aim_command_stamp;
        double max_prediction_time_abs = 0.0;
        int last_selected_track_id = -1;
        int last_selected_armor_id = -1;
    } stats_;

    rclcpp::Subscription<armor_plate_interfaces::msg::TrackedTargets>::SharedPtr tracked_targets_sub_;
    rclcpp::Subscription<armor_plate_interfaces::msg::PlannerDebug>::SharedPtr planner_debug_sub_;
    rclcpp::Subscription<armor_plate_interfaces::msg::AimCommand>::SharedPtr aim_command_sub_;
    // TrackerDebug线程
    ThreadSafeQueue<TrackerDebug::SharedPtr, true> tracker_debug_queue_{1};
    std::thread tracker_debug_thread_;
    std::atomic<bool> tracker_debug_worker_running_{false};
    std::atomic<bool> should_exit_{false};
    // PlannerDebug线程
    ThreadSafeQueue<PlannerDebug::SharedPtr, true> planner_debug_queue_{1};
    std::thread planner_debug_thread_;
    std::atomic<bool> planner_debug_worker_running_{false};
    // GUI
    GuiWorker gui_worker_;
    DebugTracker debug_tracker_{&pose_solver_, &gui_worker_};
    bool headless_ = false;

    // Yaw 搜索 benchmark（仅 Test target 链接）
    std::unique_ptr<armor_plate_identification::debug::YawSearchBenchmark> yaw_benchmark_;
    std::size_t raw_frame_index_ = 0;

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

    void plannerDebugCallBack(const PlannerDebug::SharedPtr msg);
    void processPlannerDebug(const PlannerDebug::SharedPtr msg);
    void plannerDebugWorker();

    void compositeDebugOverlay(std::unique_ptr<KeyFrameRecord> record);

    void initDebug();
    void initDetector();
    void initPoseSolver();
    void initYawBenchmark();
    void initStatsSubscriptions();

public:
    Test(std::string video_path);
    ~Test();
    void run();
    int runWithStatus();
    bool finalizeYawBenchmark();
    void closeTrackerDebugFile();
    void stopTrackerDebugWorker();
    void stopPlannerDebugWorker();
    bool printSummary();
};
