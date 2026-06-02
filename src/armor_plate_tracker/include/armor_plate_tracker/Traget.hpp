#pragma once
#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"
#include "armor_plate_tracker/TrackerArmor.hpp"

#include <array>

/** @brief 用于跟踪装甲板状态的目标类 */
class Traget
{
private:
    MyExtendedKalmanFilter ekf_;
    std::array<Eigen::Vector<double, 4>, 4> armor_list_;

    bool is_initialized_;
    bool is_divergent_;
    bool is_converged_;
    size_t selected_armor_id_;

    bool checkDivergence();
    bool checkConverge();
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

    bool isDivergent() const { return is_divergent_; }
    bool isConverged() const { return is_converged_; }
    bool isInitialized() const { return is_initialized_; }
    size_t getSelectedArmorId() const { return selected_armor_id_; }

    std::array<Eigen::Vector<double, 4>, 4> getTrackerArmorList() const { return armor_list_; }
    Eigen::Vector<double, 11> getEKFState() const { return ekf_.getStatePost(); }
    Eigen::Vector<double, 4> getFilteredObservation() const { return ekf_.getFilteredObservation(); }

    // 便捷接口：从 EKF 状态中提取常用量
    Eigen::Vector3d getCenterPointWorld() const;
    Eigen::Vector3d getCenterVelocity() const;
    double getRadius() const;
};
