#include "armor_plate_identification/DebugIdentifaction.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>

void showMultiImages(const std::string& window_name,
                     const std::vector<cv::Mat>& imgs,
                     const std::vector<std::string>& labels)
{
    if (imgs.empty()) return;
    
    // 固定输出窗口大小
    const int grid_cols = 2;  // 2列
    const int grid_rows = 2;  // 2行
    const int cell_width = 640;
    const int cell_height = 480;
    const int text_height = 30;  // 标签高度
    
    // 创建画布（2x2 网格）
    cv::Mat canvas((cell_height + text_height) * grid_rows, 
                   cell_width * grid_cols, 
                   CV_8UC3, cv::Scalar(0, 0, 0));
    
    for (size_t i = 0; i < imgs.size() && i < 4; ++i) {
        int row = i / grid_cols;
        int col = i % grid_cols;
        
        // 转换单通道为三通道
        cv::Mat img_display;
        if (imgs[i].channels() == 1) {
            cv::cvtColor(imgs[i], img_display, cv::COLOR_GRAY2BGR);
        } else {
            img_display = imgs[i].clone();
        }
        
        // resize 到固定大小
        cv::Mat img_resized;
        cv::resize(img_display, img_resized, cv::Size(cell_width, cell_height));
        
        // 计算放置位置
        int x = col * cell_width;
        int y = row * (cell_height + text_height);
        
        // 复制图像到画布
        cv::Rect roi(x, y + text_height, cell_width, cell_height);
        img_resized.copyTo(canvas(roi));
        
        // 绘制标签
        std::string label = (i < labels.size()) ? labels[i] : ("Image " + std::to_string(i + 1));
        cv::putText(canvas, label, cv::Point(x + 10, y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
    
    cv::imshow(window_name, canvas);
}

void drawNumberTest(cv::Mat& img, const Armor& armor)
{
    if (armor.name_ == ArmorName::NONE) return;
    std::string text = armorNameToString(armor.name_)
                     + " (" + std::to_string(static_cast<int>(armor.confidence_ * 100)) + "%)";
    cv::putText(img, text, (armor.points_[0] + armor.points_[1]) / 2,
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
}

void drawAllNumberTest(cv::Mat& img, const std::vector<Armor>& armors)
{
    for (const auto& armor : armors) {
        drawNumberTest(img, armor);
    }
}

/////////////////// 绘制函数 //////////////////////////

void drawArmors(cv::Mat& img, const std::vector<Armor>& armors)
{
    for (const auto& armor : armors) {
        // 画交叉线（紫色）
        cv::line(img, armor.points_[0], armor.points_[2], cv::Scalar(255, 0, 255), 2);
        cv::line(img, armor.points_[1], armor.points_[3], cv::Scalar(255, 0, 255), 2);
    }
}

void drawAllLights(cv::Mat& img, const std::vector<Light>& lights)
{
    for (const auto& light : lights) {
        drawRotatedRect(img, light.rect_); // 默认是蓝色
    }
}

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
}

void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color, int thickness)
{
    cv::line(img, p1, p2, color, thickness);
    cv::line(img, p2, p3, color, thickness);
    cv::line(img, p3, p4, color, thickness);
    cv::line(img, p4, p1, color, thickness);
}

/////////////////// DebugParamController //////////////////////////

bool DebugParamController::handleKey(int key, const rclcpp::Logger& logger)
{
    // +/-：速度控制
    if (key == '+' || key == '=') {
        play_delay_ms_ = std::max(play_delay_ms_ - 10, 0);
        RCLCPP_INFO(logger, "播放延迟: %d ms", play_delay_ms_);
        return true;
    }
    if (key == '-' || key == '_') {
        play_delay_ms_ += 10;
        RCLCPP_INFO(logger, "播放延迟: %d ms", play_delay_ms_);
        return true;
    }

    return false;
}

void DebugParamController::drawProcessTime(cv::Mat& img, float process_time_ms)
{
    if (process_time_ms < 0.0f) return;
    std::string time_text = "Process: " + std::to_string(process_time_ms) + " ms";
    time_text = time_text.substr(0, time_text.find('.') + 3);
    cv::putText(img, time_text, cv::Point(x_, y_ + 0 * line_h_),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
}

void DebugParamController::drawDelay(cv::Mat& img)
{
    std::string delay_text = "Delay: " + std::to_string(play_delay_ms_) + " ms";
    cv::putText(img, delay_text, cv::Point(x_, y_ + 1 * line_h_),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

void DebugParamController::drawDebugInfo(cv::Mat& img, bool show_speed_control)
{
    int base_row = 2;  // 1 行 process_time + 1 行 delay
    if (show_speed_control) {
        cv::putText(img, "+/-:speed  P:pause  ESC:exit",
                    cv::Point(x_, y_ + base_row * line_h_ + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    } else {
        cv::putText(img, "P:pause  ESC:exit",
                    cv::Point(x_, y_ + base_row * line_h_ + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
}

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

void saveTrackerDebugToFile(const std::string& log_dir,
                            const armor_plate_interfaces::msg::TrackerDebug& msg)
{
    auto& tracker_log_file = getTrackerLogFile();
    auto& current_log_dir = getCurrentLogDir();

    // 如果目录变化，关闭旧文件
    if (current_log_dir != log_dir) {
        if (tracker_log_file.is_open()) {
            tracker_log_file.close();
        }
        current_log_dir = log_dir;
    }

    // 首次调用时创建目录并打开文件
    if (!tracker_log_file.is_open()) {
        std::filesystem::create_directories(log_dir);
        std::string log_path = log_dir + "/tracker_log.txt";
        tracker_log_file.open(log_path, std::ios::out | std::ios::app);
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

void closeTrackerDebugFile()
{
    auto& tracker_log_file = getTrackerLogFile();
    if (tracker_log_file.is_open()) {
        tracker_log_file.close();
    }
    getCurrentLogDir().clear();
}

/////////////////// infoTrackerDebugMsg //////////////////////////

void infoTrackerDebugMsg(const rclcpp::Logger& logger,
                         const armor_plate_interfaces::msg::TrackerDebug& msg)
{
    RCLCPP_INFO(logger,
        "cam:(%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f) "
        "world:(%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f) "
        "yaw:%.4f->%.4f "
        "center:(%.4f,%.4f) r:%.4f v:(%.4f,%.4f) "
        "%s solve:%d %.1fms",
        msg.target_point.x, msg.target_point.y, msg.target_point.z,
        msg.filtered_point.x, msg.filtered_point.y, msg.filtered_point.z,
        msg.target_point_world.x, msg.target_point_world.y, msg.target_point_world.z,
        msg.filtered_point_world.x, msg.filtered_point_world.y, msg.filtered_point_world.z,
        msg.raw_yaw, msg.filter_yaw,
        msg.center_x, msg.center_y, msg.center_r,
        msg.center_v_x, msg.center_v_y,
        msg.method.c_str(), msg.solve_ok, msg.time_cost);
}

/////////////////// NumberRoiCollector //////////////////////////

int NumberRoiCollector::global_counter_ = 0;
int NumberRoiCollector::batch_counter_ = 0;

void NumberRoiCollector::toggleRecording()
{
    if (recording_) {
        stopRecording();
    } else {
        startRecording();
    }
}

void NumberRoiCollector::startRecording()
{
    recording_ = true;
    collected_.clear();
    RCLCPP_INFO(rclcpp::get_logger("NumberRoiCollector"), "开始录制 rejected ROI");
}

void NumberRoiCollector::stopRecording()
{
    recording_ = false;
    if (!collected_.empty()) {
        std::string dir = "./Debug/NumberROI/rejected/batch_" + std::to_string(++batch_counter_);
        save(dir);
    }
    RCLCPP_INFO(rclcpp::get_logger("NumberRoiCollector"),
        "停止录制，共收集 %lu 张 rejected ROI", collected_.size());
}

int NumberRoiCollector::feedRejected(const std::vector<cv::Mat>& rois)
{
    if (!recording_ || rois.empty()) return 0;
    int before = static_cast<int>(collected_.size());
    collected_.insert(collected_.end(), rois.begin(), rois.end());
    int added = static_cast<int>(collected_.size()) - before;
    if (added > 0) {
        RCLCPP_WARN(rclcpp::get_logger("NumberRoiCollector"),
            "录制中: 新增 %d 张 rejected ROI (总计 %lu)", added, collected_.size());
    }
    return added;
}

void NumberRoiCollector::save(const std::string& dir)
{
    if (collected_.empty()) return;
    for (const auto& roi : collected_) {
        std::string path = dir + "/roi_" + std::to_string(++global_counter_) + ".png";
        if (!cv::imwrite(path, roi)) {
            RCLCPP_WARN(rclcpp::get_logger("NumberRoiCollector"),
                "写入失败: %s", path.c_str());
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("NumberRoiCollector"),
        "已保存 %lu 张 ROI 到 %s", collected_.size(), dir.c_str());
    collected_.clear();
}

void NumberRoiCollector::show()
{
    if (collected_.empty()) return;
    cv::Mat concat_img;
    cv::hconcat(collected_, concat_img);
    cv::imshow("Number ROIs", concat_img);
}
