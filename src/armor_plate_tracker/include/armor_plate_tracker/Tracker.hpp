#pragma once
#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"
#include "armor_plate_tracker/CoordinateTransformer.hpp"

#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

struct AngleRecord {
    builtin_interfaces::msg::Time stamp;
    float yaw_abs;
    float pitch_abs;
};

class Tracker
{
private:
    // EKF 滤波器
    MyExtendedKalmanFilter ekf_;

    // 坐标变换器
    CoordinateTransformer transformer_;

    // 装甲板数据（观测 / 滤波）
    TrackerArmor measured_armor_;
    TrackerArmor filter_armor_;

    // EKF 中心点状态
    Eigen::Vector3d center_point_world_{0, 0, 0};
    Eigen::Vector3d center_velocity_{0, 0, 0};
    float center_r_ = 0.0f;

    // 时间相关
    double last_update_time_ = 0.0;
    double last_detection_time_ = 0.0;

    // 跟踪状态
    bool initialized_ = false;
    double max_lost_time_ = 0.1;
    bool is_lost_ = true;

    // 突变检测
    float yaw_mutation_threshold_ = 0.05f;
    float last_armor_pose_yaw_world_ = 0.0f;
    int32_t last_armor_number_ = 0;

    // 调试信息
    float time_cost_ = 0.0f;
    bool solve_ok_ = false;

    void selectBestMatch(const std::vector<TrackerArmor> & armors, TrackerArmor & target);
    bool checkYawMutation(float armor_pose_yaw);
    bool isLostTooLong(double current_time) const;
    double calculateDt(double current_time);

    void updateMeasurement(const TrackerArmor & armor, double current_time);
    void updateFilteredValue(const TrackerArmor & armor);

public:
    Tracker();

    void reset();
    void init(const TrackerArmor & armor, double current_time);

    void setMaxLostTime(double seconds) { max_lost_time_ = seconds; }
    void setMutationThreshold(float yaw_thresh) { yaw_mutation_threshold_ = yaw_thresh; }

    void Update(const std::vector<ArmorPlate> & armor_plates,
                double current_time,
                float yaw_abs, float pitch_abs);

    // 获取装甲板数据
    const TrackerArmor & getMeasuredArmor() const { return measured_armor_; }
    const TrackerArmor & getFilterArmor() const { return filter_armor_; }

    // 增量角（从 filter_armor_ 的 ypd_camera_ 获取）
    float getYaw() const { return filter_armor_.ypd_camera_.x(); }
    float getPitch() const { return filter_armor_.ypd_camera_.y(); }

    bool isLost() const { return is_lost_; }
    bool isInitialized() const { return initialized_; }
    double getLastUpdateTime() const { return last_update_time_; }

    // EKF 中心点
    Eigen::Vector3d getCenterPointWorld() const { return center_point_world_; }
    Eigen::Vector3d getCenterVelocity() const { return center_velocity_; }

    armor_plate_interfaces::msg::TrackerDebug CreatedebugMsg(const builtin_interfaces::msg::Time & stamp) const;
};
