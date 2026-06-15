#include "armor_plate_identification/debug/DebugTest.hpp"

#include <fstream>
#include <filesystem>
#include <iomanip>

namespace {
    std::ofstream& getTrackerLogFile() {
        static std::ofstream file;
        return file;
    }
    std::string& getCurrentLogDir() {
        static std::string dir;
        return dir;
    }
}

DebugTest::DebugTest(const DebugBaseParams& base_params, const DebugTestParams& test_params)
    : DebugIdentification(base_params), test_params_(test_params) {}

void DebugTest::saveTrackerDebug(const std::string& log_dir,
                                 const armor_plate_interfaces::msg::TrackerDebug& msg)
{
    auto& tracker_log_file = getTrackerLogFile();
    auto& current_log_dir = getCurrentLogDir();

    if (!tracker_log_file.is_open() && current_log_dir != log_dir) {
        current_log_dir = log_dir;
    }

    if (!tracker_log_file.is_open()) {
        std::filesystem::create_directories(current_log_dir);
        std::string log_path = current_log_dir + "/tracker_log.txt";
        tracker_log_file.open(log_path, std::ios::out);
        if (tracker_log_file.is_open()) {
            tracker_log_file << "sec nanosec log"
                << std::endl;
            RCLCPP_INFO(rclcpp::get_logger("TRACKER_DEBUG"), "日志保存到: %s", log_path.c_str());
        }
    }

    if (tracker_log_file.is_open()) {
        tracker_log_file << msg.header.stamp.sec << " " << msg.header.stamp.nanosec << " "
            << "滤波:("
            << std::fixed << std::setprecision(4) << msg.filtered_point_world.x << ","
            << std::fixed << std::setprecision(4) << msg.filtered_point_world.y << ","
            << std::fixed << std::setprecision(4) << msg.filtered_point_world.z << ","
            << std::fixed << std::setprecision(4) << msg.filter_yaw << "),"
            << "EKF状态: 中心点("
            << std::fixed << std::setprecision(4) << msg.center_world.x << ","
            << std::fixed << std::setprecision(4) << msg.center_world.y << ","
            << std::fixed << std::setprecision(4) << msg.center_world.z << ") "
            << "中心速度("
            << std::fixed << std::setprecision(4) << msg.center_velocity.x << ","
            << std::fixed << std::setprecision(4) << msg.center_velocity.y << ","
            << std::fixed << std::setprecision(4) << msg.center_velocity.z << "), "
            << "r = " << std::fixed << std::setprecision(4) << msg.center_r << ", "
            << "l = " << std::fixed << std::setprecision(4) << msg.center_l << ", "
            << "h = " << std::fixed << std::setprecision(4) << msg.center_h << ", "
            << "是否丢失:" << (msg.is_lost ? 1 : 0)
            << std::endl;
    }
}

void DebugTest::closeTrackerDebugFile()
{
    auto& tracker_log_file = getTrackerLogFile();
    if (tracker_log_file.is_open()) {
        tracker_log_file.close();
    }
    getCurrentLogDir().clear();
}
