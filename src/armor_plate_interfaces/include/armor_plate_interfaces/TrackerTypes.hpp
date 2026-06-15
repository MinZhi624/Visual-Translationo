#pragma once

#include <cstdint>

namespace armor_plate_interfaces
{

enum class TrackerState : uint8_t
{
  LOST = 0,
  DETECTING = 1,
  TRACKING = 2,
  TEMP_LOST = 3
};

inline uint8_t trackerStateToUint8(TrackerState state)
{
  return static_cast<uint8_t>(state);
}

inline TrackerState uint8ToTrackerState(uint8_t value)
{
  switch (value) {
    case 0:
      return TrackerState::LOST;
    case 1:
      return TrackerState::DETECTING;
    case 2:
      return TrackerState::TRACKING;
    case 3:
      return TrackerState::TEMP_LOST;
    default:
      return TrackerState::LOST;
  }
}

}  // namespace armor_plate_interfaces