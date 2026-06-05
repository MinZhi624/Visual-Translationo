#pragma once
#include "armor_plate_tracker/Target.hpp"
#include "armor_plate_tracker/CoordinateTransformer.hpp"

#include <armor_plate_interfaces/ArmorPose.hpp>
#include <armor_plate_interfaces/ArmorTypes.hpp>
#include <armor_plate_interfaces/GimbalData.hpp>
#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;

enum class TrackerState{LOST, TEMP_LOST, DETECTING, TRACKING};

/** @brief 状态管理 + 目标选择 */
class Tracker
{
private:
    static constexpr float MIN_VALID_ARMOR_PITCH_WORLD = -0.05f;
    // 目标跟踪器
    Target target_;
    // 坐标变换器
    CoordinateTransformer transformer_;
    // 观测量
    TrackerArmor measured_armor_;
    TrackerArmor filter_armor_;
    int selected_armor_id_ = -1;
    Eigen::Vector3d center_point_world_{0, 0, 0};
    Eigen::Vector3d center_velocity_{0, 0, 0};
    float center_r_ = 0.0f;
    // 时间相关
    double last_update_time_ = 0.0;
    double last_detection_time_ = 0.0;
    // 跟踪状态
    TrackerState state_ = TrackerState::LOST;
    int detect_count_ = 0;
    double max_lost_time_ = 0.1;

    ArmorName last_armor_name_ = ArmorName::NONE;

    bool isLostTooLong(double current_time) const;
    double calculateDt(double current_time);

    TrackerArmor ArmorPlateToTrackerArmor(const ArmorPlate & armor_plate);
    std::vector<TrackerArmor> ArmorPlateToTrackerArmor(const std::vector<ArmorPlate> & armor_plates);

    static std::map<ArmorName, std::vector<TrackerArmor>> groupByArmorName(const std::vector<TrackerArmor> & armors);
    static TrackerArmor selectRepresentative(const std::vector<TrackerArmor> & armors);

    void updateMeasurement(const TrackerArmor & armor, double current_time);
    void updateFilteredValue(const TrackerArmor & armor);
    void extractFilteredResult();

    void updateState(bool is_found, double current_time);
public:
    Tracker();

    void reset();
    void init(const TrackerArmor & armor, double current_time);
    void setMaxLostTime(double seconds) { max_lost_time_ = seconds; }
    void Update(const std::vector<ArmorPlate> & armor_plates,
                double current_time,
                const GimbalData & gimbal);

    // 获取装甲板数据
    const TrackerArmor & getMeasuredArmor() const { return measured_armor_; }
    const TrackerArmor & getFilterArmor() const { return filter_armor_; }
    const std::array<ArmorPose, 4> & getTrackerArmorList() const { return target_.getTargetArmorList(); }
    // 增量角（从 filter_armor_ 的 ypd_gimbal_ 获取）
    float getYaw() const { return filter_armor_.ypd_gimbal_.x(); }
    float getPitch() const { return filter_armor_.ypd_gimbal_.y(); }

    bool isSend() const {return state_ == TrackerState::TRACKING; }
    bool isLost() const { return state_ == TrackerState::LOST; }
    double getLastUpdateTime() const { return last_update_time_; }

    // EKF 中心点
    Eigen::Vector3d getCenterPointWorld() const { return center_point_world_; }
    Eigen::Vector3d getCenterVelocity() const { return center_velocity_; }
    int getSelectedArmorId() const { return selected_armor_id_; }

    armor_plate_interfaces::msg::TrackerDebug CreatedebugMsg(const builtin_interfaces::msg::Time & stamp) const;
};
