#include "armor_plate_planner/TargetSelector.hpp"

using armor_plate_interfaces::TrackerState;

std::optional<size_t> TargetSelector::selectIndex(
    const std::vector<TargetState>& targets)
{
    // 第一版只接受 TRACKING 状态
    for (size_t i = 0; i < targets.size(); ++i) {
        if (targets[i].tracking_state == TrackerState::TRACKING) {
            last_selected_index_ = i;
            return last_selected_index_;
        }
    }

    return std::nullopt;
}
