#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include "armor_plate_identification/GuiWorker.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include <rclcpp/logging.hpp>
#include <opencv2/core.hpp>
#include <map>
#include <chrono>
#include <vector>

void drawArmors(cv::Mat& img, const std::vector<DetectorArmor>& armors);
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);

void infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug& msg);

struct DebugBaseParams
{
    bool debug_timecontrol_ = false;
    bool debug_lights_ = false;
    bool debug_preprocessing_ = false;
    bool debug_number_classification_ = false;
    int delay_time = 20;
    int stats_interval = 50;
};


class DebugBase
{
public:
    DebugBase() = default;
    DebugBase(const DebugBaseParams& params);

    // 生命周期
    void onFrameStart();
    void mark(const std::string& label);
    void onFrameEnd();
    virtual bool shouldShow() const { return true; }

    // 信息收集（无条件调，内部判开关）
    void debugLights(const std::vector<Light>& lights);
    void debugNumberClassification(const std::vector<DetectorArmor>& armors);
    void debugPreprocessing(const cv::Mat& img_bgr, const PreprocessDebug& prep);

    void draw(cv::Mat& target_img);
    void show();

    // 取走本帧准备好的显示图像，调用后内部缓存清空
    std::vector<std::pair<std::string, cv::Mat>> getDisplayFrames();

    void save();

    // 按键处理（debug 相关：+/- 调延迟、S 录制 ROI）
    void control(const KeyEvent& event);

    // 查询
    bool isDebugTimeControl() const { return params_.debug_timecontrol_; }
    bool isDebugLights() const { return params_.debug_lights_; }
    bool isDebugPreprocessing() const { return params_.debug_preprocessing_; }
    bool isDebugNumberClassification() const { return params_.debug_number_classification_; }
    int getDelayTimeMs() const { return params_.delay_time; }
    int getFrameCount() const { return frame_count_; }

    // ROI 录制
    void feedRejected(const std::vector<cv::Mat>& rois);
    bool isRecordingRois() const { return recording_; }

protected:
    DebugBaseParams params_;
    int frame_count_ = 0;

    // 延迟绘制缓存
    std::vector<Light> cached_lights_;
    std::vector<DetectorArmor> cached_armors_;
    cv::Mat cached_img_bgr_;
    PreprocessDebug preprocess_debug_;

    // ROI 录制
    bool recording_ = false;
    std::vector<cv::Mat> collected_;
    static int global_counter_;
    static int batch_counter_;

    // 计时
    std::chrono::steady_clock::time_point start_mark_;
    std::chrono::steady_clock::time_point last_mark_;
    float last_process_time_ms_ = 0.0f;

    // 路标累计
    std::map<std::string, float> mark_sums_;

    // 性能统计
    float process_time_sum_ = 0.0f;

    // 本帧准备好的显示图像缓存
    std::vector<std::pair<std::string, cv::Mat>> display_frames_;

private:
    void printStats();
    void drawLights(cv::Mat& img);
    void drawNumbers(cv::Mat& img);
    void drawProcessTime(cv::Mat& img);
    void drawDelayTime(cv::Mat& img);
    void showPreprocessWindow();
    void showRoiCollector();
    void showNumberRois();
};
