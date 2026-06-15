#include "armor_plate_identification/GuiWorker.hpp"
#include "armor_plate_identification/DetectorArmor.hpp"

#include <cmath>
#include <opencv2/imgproc.hpp>

void GuiWorker::drawArmors(cv::Mat& img, const std::vector<DetectorArmor>& armors)
{
    for (const auto& armor : armors) {
        cv::line(img, armor.image_points_[0], armor.image_points_[2], cv::Scalar(255, 0, 255), 2);
        cv::line(img, armor.image_points_[1], armor.image_points_[3], cv::Scalar(255, 0, 255), 2);
    }
}

void GuiWorker::drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], color, thickness);
    }
}

void GuiWorker::drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color, int thickness)
{
    cv::line(img, p1, p2, color, thickness);
    cv::line(img, p2, p3, color, thickness);
    cv::line(img, p3, p4, color, thickness);
    cv::line(img, p4, p1, color, thickness);
}

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
    if (img.empty()) return;

    DisplaySlot next;
    next.display = img.clone();

    std::lock_guard<std::mutex> lock(frames_mutex_);
    frames_[window_name] = std::move(next);
}

std::unique_ptr<KeyFrame> GuiWorker::exchangeKeyFrame(
    const std::string& window_name, std::unique_ptr<KeyFrame> frame, double display_scale)
{
    if (!frame || frame->image.empty()) return nullptr;

    DisplaySlot next;
    next.key_frame = std::move(frame);
    if (display_scale > 0.0 && std::abs(display_scale - 1.0) > 1e-6) {
        cv::resize(next.key_frame->image, next.display, cv::Size(), display_scale, display_scale);
    } else {
        next.display = next.key_frame->image;
    }

    std::lock_guard<std::mutex> lock(frames_mutex_);
    auto & current = frames_[window_name];
    auto previous = std::move(current.key_frame);
    current = std::move(next);
    return previous;
}

std::unique_ptr<KeyFrame> GuiWorker::takeKeyFrame(const std::string& window_name)
{
    std::lock_guard<std::mutex> lock(frames_mutex_);
    auto it = frames_.find(window_name);
    if (it == frames_.end()) return nullptr;

    auto frame = std::move(it->second.key_frame);
    frames_.erase(it);
    return frame;
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
            for (const auto& [name, slot] : frames_) {
                if (!slot.display.empty()) {
                    cv::imshow(name, slot.display);
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
