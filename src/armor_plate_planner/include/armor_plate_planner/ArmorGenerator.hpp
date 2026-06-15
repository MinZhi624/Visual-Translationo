#pragma once

#include <armor_plate_interfaces/msg/tracked_target.hpp>
#include <armor_plate_interfaces/msg/tracked_armor.hpp>
#include <vector>

class ArmorGenerator
{
public:
    ArmorGenerator() = default;

    std::vector<armor_plate_interfaces::msg::TrackedArmor> generate(
        const armor_plate_interfaces::msg::TrackedTarget & state) const;
};
