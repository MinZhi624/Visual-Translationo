#pragma once

#include "armor_plate_planner/ArmorSelector.hpp"
#include <armor_plate_interfaces/TrackerTypes.hpp>
#include <Eigen/Core>
#include <vector>
#include <cstdint>

struct TargetState {
    uint32_t track_id = 0;
    armor_plate_interfaces::TrackerState tracking_state = armor_plate_interfaces::TrackerState::LOST;
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
    double radius = 0.0;
    double radius_offset = 0.0;
    double height_offset = 0.0;
    std::vector<PlannerArmor> armors;
};

class TargetPredictor
{
public:
    TargetPredictor() = default;

    TargetState predict(
        const TargetState& state,
        double dt) const;
};
