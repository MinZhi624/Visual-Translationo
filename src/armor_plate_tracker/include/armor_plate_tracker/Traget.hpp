#pragma once
#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"
#include "armor_plate_tracker/TrackerArmor.hpp"

#include <array>
#include <vector>

/** @brief 用于跟踪装甲板状态的目标类 */
class Traget
{
private:
    MyExtendedKalmanFilter ekf_;
    std::array<Eigen::Vector<double, 4>, 4> armor_list_;

    bool initialized_ = false;
    size_t selected_armor_id_ = 0;

    void updateArmorList();
    size_t findArmorIdx(const TrackerArmor & armor);

    static double normalizeRadAngle(double rad);

public:
    Traget() = default;
    // 核心函数
    void predict(double dt);
    void update(const TrackerArmor & armor);
    void update(const std::vector<TrackerArmor> & armors);

    void reset();
    void init(const TrackerArmor & armor);

    bool isInitialized() const { return initialized_; }
    size_t getSelectedArmorId() const { return selected_armor_id_; }

    std::array<Eigen::Vector<double, 4>, 4> getTrackerArmorList() const { return armor_list_; }
    Eigen::Vector<double, 11> getEKFState() const { return ekf_.getStatePost(); }
    Eigen::Vector<double, 4> getFilteredObservation() const { return ekf_.getFilteredObservation(); }

    // 便捷接口：从 EKF 状态中提取常用量
    Eigen::Vector3d getCenterPointWorld() const;
    Eigen::Vector3d getCenterVelocity() const;
    double getRadius() const;
};
