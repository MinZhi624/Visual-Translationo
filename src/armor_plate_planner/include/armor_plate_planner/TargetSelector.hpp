#pragma once

#include <armor_plate_interfaces/msg/tracked_targets.hpp>
#include <optional>
#include <cstdint>

class TargetSelector
{
private:
    size_t last_selected_index_ = 0;

public:
    TargetSelector() = default;

    std::optional<size_t> selectIndex(const armor_plate_interfaces::msg::TrackedTargets & targets);
    void reset() { last_selected_index_ = 0; }
};
