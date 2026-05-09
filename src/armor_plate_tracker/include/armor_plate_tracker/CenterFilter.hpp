#pragma once
#include <Eigen/Core>
#include <deque>
#include <vector>

// 带时间戳的装甲板观测
struct ArmorPlateStamped {
    double timestamp;
    Eigen::Vector3d position_world;  // 世界坐标系位置
    double yaw_world;                // 世界坐标系 yaw
};

// 一帧数据：多块同车板 + 时间戳
struct Frame {
    double timestamp;
    std::vector<ArmorPlateStamped> plates;
};

class CenterFilter
{
public:
    CenterFilter();

    // 添加一帧观测（1-2 块同车板）
    void addFrame(const std::vector<ArmorPlateStamped>& plates, double timestamp);

    // 最小二乘求解，返回是否成功
    bool solve();

    // 重置历史数据
    void reset();

    // 设置窗口大小（帧数）
    void setWindowSize(size_t size) { window_size_ = size; }

    // 获取结果
    Eigen::Vector2d getCenter() const { return center_; }
    Eigen::Vector2d getVelocity() const { return velocity_; }
    double getR() const { return r_; }
    bool isReady() const { return frame_history_.size() >= min_frames_; }

    // 用默认 r 反算旋转中心（数据不足时）
    static Eigen::Vector2d backCalculateCenter(
        const Eigen::Vector3d& position_world, double yaw_world, double r_default = 0.26);

private:
    size_t window_size_;
    size_t min_frames_;
    std::deque<Frame> frame_history_;

    // 求解结果
    Eigen::Vector2d center_;
    Eigen::Vector2d velocity_;
    double r_;
};
