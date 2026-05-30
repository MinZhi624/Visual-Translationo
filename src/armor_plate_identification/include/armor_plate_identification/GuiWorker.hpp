#pragma once

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
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<KeyAction> last_action_{KeyAction::None};
    std::atomic<int> last_raw_key_{-1};

    std::unordered_map<std::string, cv::Mat> frames_;
    std::mutex frames_mutex_;

    void loop();
public:
    GuiWorker() = default;
    ~GuiWorker() { stop(); }

    void start();
    void stop();

    void pushFrame(const std::string& window_name, const cv::Mat& img);
    KeyEvent consumeKey();

    bool isRunning() const { return running_.load(); }


};
