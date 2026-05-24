#include "armor_plate_identification/DebugTest.hpp"

#include <fstream>
#include <filesystem>

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
    : DebugBase(base_params), test_params_(test_params) {}

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
            tracker_log_file << "sec nanosec "
                << "target_world_x target_world_y target_world_z "
                << "filtered_world_x filtered_world_y filtered_world_z "
                << "raw_yaw filter_yaw "
                << "center_x center_y center_r center_vx center_vy "
                << "method solve_ok time_cost" << std::endl;
            RCLCPP_INFO(rclcpp::get_logger("TRACKER_DEBUG"), "日志保存到: %s", log_path.c_str());
        }
    }

    if (tracker_log_file.is_open()) {
        tracker_log_file << msg.header.stamp.sec << " " << msg.header.stamp.nanosec << " "
            << msg.target_point_world.x << " " << msg.target_point_world.y << " " << msg.target_point_world.z << " "
            << msg.filtered_point_world.x << " " << msg.filtered_point_world.y << " " << msg.filtered_point_world.z << " "
            << msg.raw_yaw << " " << msg.filter_yaw << " "
            << msg.center_x << " " << msg.center_y << " " << msg.center_r << " "
            << msg.center_v_x << " " << msg.center_v_y << " "
            << msg.method << " " << msg.solve_ok << " " << msg.time_cost << std::endl;
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
