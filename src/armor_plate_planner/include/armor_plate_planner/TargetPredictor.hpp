#pragma once

#include <armor_plate_interfaces/msg/tracked_target.hpp>

class TargetPredictor
{
public:
    TargetPredictor() = default;

    armor_plate_interfaces::msg::TrackedTarget predict(
        const armor_plate_interfaces::msg::TrackedTarget & state,
        double dt) const;
};
