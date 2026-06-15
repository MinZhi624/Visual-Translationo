#include "armor_plate_planner/TargetSelector.hpp"
#include <armor_plate_interfaces/TrackerTypes.hpp>

using armor_plate_interfaces::uint8ToTrackerState;
using armor_plate_interfaces::TrackerState;

std::optional<size_t> TargetSelector::selectIndex(
    const armor_plate_interfaces::msg::TrackedTargets & targets)
{
    // 第一版只接受 TRACKING 状态
    for (size_t i = 0; i < targets.targets.size(); ++i) {
        if (uint8ToTrackerState(targets.targets[i].tracking_state) == TrackerState::TRACKING) {
            last_selected_index_ = i;
            return last_selected_index_;
        }
    }

    return std::nullopt;
}
