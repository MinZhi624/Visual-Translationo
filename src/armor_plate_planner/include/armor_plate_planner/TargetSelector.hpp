#pragma once

#include "armor_plate_planner/TargetPredictor.hpp"
#include <optional>
#include <vector>
#include <cstddef>

class TargetSelector
{
private:
    size_t last_selected_index_ = 0;

public:
    TargetSelector() = default;

    std::optional<size_t> selectIndex(const std::vector<TargetState>& targets);
    void reset() { last_selected_index_ = 0; }
};
