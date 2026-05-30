#include "armor_plate_identification/GuiWorker.hpp"

void GuiWorker::start()
{
    if (running_.load()) return;
    running_ = true;
    last_action_ = KeyAction::None;
    last_raw_key_ = -1;
    thread_ = std::thread(&GuiWorker::loop, this);
}

void GuiWorker::stop()
{
    if (!running_.load()) return;
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void GuiWorker::pushFrame(const std::string& window_name, const cv::Mat& img)
{
    std::lock_guard<std::mutex> lock(frames_mutex_);
    if (img.empty()) return;
    frames_[window_name] = img.clone();
}

KeyEvent GuiWorker::consumeKey()
{
    KeyAction action = last_action_.exchange(KeyAction::None);
    int raw = last_raw_key_.exchange(-1);
    return KeyEvent{action, raw};
}

void GuiWorker::loop()
{
    while (running_.load()) {
        {
            std::lock_guard<std::mutex> lock(frames_mutex_);
            for (const auto& [name, img] : frames_) {
                if (!img.empty()) {
                    cv::imshow(name, img);
                }
            }
        }

        int key = cv::waitKey(1);
        if (key != -1) {
            if (key == 27 || key == 'q' || key == 'Q') {
                last_action_ = KeyAction::Exit;
            } else if (key == 'p' || key == 'P') {
                last_action_ = KeyAction::Pause;
            } else if (key == '+' || key == '-' || key == '_' || key == '=' ||
                       key == 's' || key == 'S') {
                last_action_ = KeyAction::Processed;
            } else {
                last_action_ = KeyAction::None;
            }
            last_raw_key_ = key;
        }
    }
    cv::destroyAllWindows();
}
