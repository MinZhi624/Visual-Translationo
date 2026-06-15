#pragma once

#include "armor_plate_identification/KeyFrame.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace DebugWindow {
    constexpr const char* IDENTIFICATION = "identification";
    constexpr const char* TRACKER_DEBUG  = "tracker_debug";
    constexpr const char* PREPROCESS     = "preprocess";
    constexpr const char* REJECTED_ROIS  = "rejected_rois";
    constexpr const char* NUMBER_ROIS    = "number_rois";
}

enum class KeyAction { None, Processed, Pause, Exit };

struct KeyEvent {
    KeyAction action = KeyAction::None;
    int raw_key = -1;
};

/**
 * @brief 将所有 OpenCV GUI 操作（imshow + waitKey）集中到单一独立线程。
 *        同时负责原始按键 → KeyAction 的映射，主进程通过 consumeKey() 获取高层事件。
 */
class GuiWorker {
private:
    struct DisplaySlot {
        cv::Mat display;
        std::unique_ptr<KeyFrame> key_frame;
    };

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<KeyAction> last_action_{KeyAction::None};
    std::atomic<int> last_raw_key_{-1};

    std::unordered_map<std::string, DisplaySlot> frames_;
    std::mutex frames_mutex_;

    void loop();
public:
    GuiWorker() = default;
    ~GuiWorker() { stop(); }

    void start();
    void stop();

    void pushFrame(const std::string& window_name, const cv::Mat& img);
    std::unique_ptr<KeyFrame> exchangeKeyFrame(
        const std::string& window_name, std::unique_ptr<KeyFrame> frame,
        double display_scale = 1.0);
    std::unique_ptr<KeyFrame> takeKeyFrame(const std::string& window_name);
    KeyEvent consumeKey();

    bool isRunning() const { return running_.load(); }

    // 绘制工具
    static void drawArmors(cv::Mat& img, const std::vector<struct DetectorArmor>& armors);
    static void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
    static void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
};
