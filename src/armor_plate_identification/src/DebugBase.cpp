#include "armor_plate_identification/DebugBase.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <sstream>
#include <iomanip>

int DebugBase::global_counter_ = 0;
int DebugBase::batch_counter_ = 0;

DebugBase::DebugBase(const DebugBaseParams& params) : params_(params) {}

void DebugBase::onFrameStart()
{
    frame_count_++;
    start_mark_ = std::chrono::steady_clock::now();
    last_mark_ = start_mark_;
}

void DebugBase::mark(const std::string& label)
{
    auto t = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float, std::milli>(t - last_mark_).count();
    mark_sums_[label] += dt;
    last_mark_ = t;
}

void DebugBase::onFrameEnd()
{
    auto t = std::chrono::steady_clock::now();
    last_process_time_ms_ = std::chrono::duration<float, std::milli>(t - start_mark_).count();
    process_time_sum_ += last_process_time_ms_;

    if (frame_count_ > 0 && frame_count_ % params_.stats_interval == 0) {
        printStats();
        process_time_sum_ = 0.0f;
        for (auto& [k, v] : mark_sums_) v = 0.0f;
    }
}

void DebugBase::printStats()
{
    float avg_total = process_time_sum_ / params_.stats_interval;

    // 识别时间
    float avg_id = 0.0f;
    if (mark_sums_.contains("preprocess")) {
        avg_id += mark_sums_.at("preprocess") / params_.stats_interval;
    }
    if (mark_sums_.contains("detectArmors")) {
        avg_id += mark_sums_.at("detectArmors") / params_.stats_interval;
    }

    // 构建路标字符串
    std::stringstream marks_ss;
    for (const auto& [name, sum] : mark_sums_) {
        float avg = sum / params_.stats_interval;
        marks_ss << name << ":" << std::fixed << std::setprecision(1) << avg << "ms ";
    }
    std::string marks_str = marks_ss.str();
    if (!marks_str.empty()) marks_str.pop_back();  // 去掉末尾空格

    if (avg_id > 0.0f) {
        RCLCPP_INFO(rclcpp::get_logger("DEBUG_BASE"),
            "【平均用时】总:%.1fms  识别:%.1fms [%s]",
            avg_total, avg_id, marks_str.c_str());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("DEBUG_BASE"),
            "【平均用时】总:%.1fms [%s]", avg_total, marks_str.c_str());
    }
}

void DebugBase::debugLights(const std::vector<Light>& lights)
{
    if (params_.debug_lights_) {
        cached_lights_ = lights;
    }
}

void DebugBase::debugNumberClassification(const std::vector<DetectorArmor>& armors)
{
    if (params_.debug_number_classification_) {
        cached_armors_ = armors;
    }
}

void DebugBase::debugPreprocessing(const cv::Mat& img_bgr, const PreprocessDebug& prep)
{
    if (params_.debug_preprocessing_) {
        cached_img_bgr_ = img_bgr.clone();
        preprocess_debug_ = prep;
    }
}

void DebugBase::draw(cv::Mat& target_img)
{
    if (params_.debug_lights_) {
        drawLights(target_img);
    }
    if (params_.debug_number_classification_) {
        drawNumbers(target_img);
    }

    drawProcessTime(target_img);
    if (params_.debug_timecontrol_) {
        drawDelayTime(target_img);
    }
}

void DebugBase::show()
{
    if (!shouldShow()) return;
    if (params_.debug_preprocessing_) {
        showPreprocessWindow();
    }
    if (params_.debug_number_classification_) {
        showRoiCollector();
    }
}

KeyAction DebugBase::handleKey(int key)
{
    if (key == 27 || key == 'q' || key == 'Q') {
        cv::destroyAllWindows();
        return KeyAction::Exit;
    }
    if (key == 'p' || key == 'P') {
        return KeyAction::Pause;
    }
    if (key == 's' || key == 'S') {
        recording_ = !recording_;
        if (recording_) {
            collected_.clear();
            RCLCPP_INFO(rclcpp::get_logger("DEBUG_NUMBER"), "开始录制 rejected ROI");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("DEBUG_NUMBER"), "停止录制，共收集 %lu 张", collected_.size());
        }
        return KeyAction::Processed;
    }
    if (params_.debug_timecontrol_) {
        if (key == '+' || key == '=') {
            params_.delay_time = std::max(params_.delay_time - 10, 0);
            RCLCPP_INFO(rclcpp::get_logger("DEBUG_TIME_CONTROL"), "播放延迟: %d ms", params_.delay_time);
            return KeyAction::Processed;
        }
        if (key == '-' || key == '_') {
            params_.delay_time += 10;
            RCLCPP_INFO(rclcpp::get_logger("DEBUG_TIME_CONTROL"), "播放延迟: %d ms", params_.delay_time);
            return KeyAction::Processed;
        }
    }
    return KeyAction::None;
}

void DebugBase::save()
{
    if (!recording_ && !collected_.empty() && !params_.log_dir.empty()) {
        std::string dir = params_.log_dir + "/rejected/batch_" + std::to_string(++batch_counter_);
        std::filesystem::create_directories(dir);
        for (const auto& roi : collected_) {
            std::string path = dir + "/roi_" + std::to_string(++global_counter_) + ".png";
            if (!cv::imwrite(path, roi)) {
                RCLCPP_WARN(rclcpp::get_logger("DEBUG_NUMBER"), "写入失败: %s", path.c_str());
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("DEBUG_NUMBER"), "已保存 %lu 张 ROI 到 %s", collected_.size(), dir.c_str());
        collected_.clear();
    }
}

void DebugBase::feedRejected(const std::vector<cv::Mat>& rois)
{
    if (recording_ && !rois.empty()) {
        collected_.insert(collected_.end(), rois.begin(), rois.end());
    }
}

// ========== 私有绘制辅助 ==========

void DebugBase::drawLights(cv::Mat& img)
{
    for (const auto& light : cached_lights_) {
        drawRotatedRect(img, light.rect_);
    }
}

void DebugBase::drawNumbers(cv::Mat& img)
{
    for (const auto& armor : cached_armors_) {
        if (armor.name_ == ArmorName::NONE) continue;
        std::string text = armorNameToString(armor.name_)
                         + " (" + std::to_string(static_cast<int>(armor.confidence_ * 100)) + "%)";
        cv::putText(img, text, (armor.points_[0] + armor.points_[1]) / 2,
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
    }
}

void DebugBase::drawProcessTime(cv::Mat& img) 
{
    if (last_process_time_ms_ < 0.0f) return;
    std::string text = "Process: " + std::to_string(last_process_time_ms_) + " ms";
    text = text.substr(0, text.find('.') + 3);
    cv::putText(img, text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
}

void DebugBase::drawDelayTime(cv::Mat& img)
{
    std::string text = "Delay: " + std::to_string(params_.delay_time) + " ms";
    cv::putText(img, text, cv::Point(10, 55),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
}

void DebugBase::showPreprocessWindow()
{
    if (cached_img_bgr_.empty() || preprocess_debug_.blue_dim_thre.empty()) return;

    cv::Mat blue_debug;
    cv::cvtColor(preprocess_debug_.blue_dim_thre, blue_debug, cv::COLOR_GRAY2BGR);
    for (const auto& [rect, count] : preprocess_debug_.fragment_info) {
        std::string num = std::to_string(count);
        cv::Scalar color = (count == 1) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(blue_debug, num, cv::Point(rect.x + 5, rect.y + 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        cv::rectangle(blue_debug, rect, color, 1);
    }

    std::vector<cv::Mat> images = {
        cached_img_bgr_, blue_debug,
        preprocess_debug_.gray_thre, preprocess_debug_.merged_thre
    };
    std::vector<std::string> labels = {"Original", "BLUE_dim (fragments)", "GRAY_thre", "Merged"};
    // showMultiImages 内联实现
    {
        const int grid_cols = 2;
        const int grid_rows = 2;
        const int cell_width = 640;
        const int cell_height = 480;
        const int text_height = 30;

        cv::Mat canvas((cell_height + text_height) * grid_rows,
                       cell_width * grid_cols,
                       CV_8UC3, cv::Scalar(0, 0, 0));

        for (size_t i = 0; i < images.size() && i < 4; ++i) {
            int row = static_cast<int>(i / grid_cols);
            int col = static_cast<int>(i % grid_cols);

            cv::Mat img_display;
            if (images[i].channels() == 1) {
                cv::cvtColor(images[i], img_display, cv::COLOR_GRAY2BGR);
            } else {
                img_display = images[i].clone();
            }

            cv::Mat img_resized;
            cv::resize(img_display, img_resized, cv::Size(cell_width, cell_height));

            int x = col * cell_width;
            int y = row * (cell_height + text_height);

            cv::Rect roi(x, y + text_height, cell_width, cell_height);
            img_resized.copyTo(canvas(roi));

            std::string label = (i < labels.size()) ? labels[i] : ("Image " + std::to_string(i + 1));
            cv::putText(canvas, label, cv::Point(x + 10, y + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("PreProcessions-View", canvas);
    }
}

void DebugBase::showRoiCollector()
{
    if (collected_.empty()) return;
    cv::Mat concat_img;
    cv::hconcat(collected_, concat_img);
    cv::imshow("Number ROIs", concat_img);
}

/////////////////// 全局绘制函数 //////////////////////////

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

void drawArmors(cv::Mat& img, const std::vector<DetectorArmor>& armors)
{
    for (const auto& armor : armors) {
        cv::line(img, armor.points_[0], armor.points_[2], cv::Scalar(255, 0, 255), 2);
        cv::line(img, armor.points_[1], armor.points_[3], cv::Scalar(255, 0, 255), 2);
    }
}

void infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug& msg)
{
    RCLCPP_INFO(rclcpp::get_logger("DEBUG_TRACKER"),
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
