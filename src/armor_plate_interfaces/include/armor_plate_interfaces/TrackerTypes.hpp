#pragma once

#include <cstdint>

namespace armor_plate_interfaces
{

enum class TrackerState : uint8_t
{
  LOST = 0,
  TEMP_LOST = 1,
  DETECTING = 2,
  TRACKING = 3
};

inline uint8_t trackerStateToUint8(TrackerState state)
{
  return static_cast<uint8_t>(state);
}

inline TrackerState uint8ToTrackerState(uint8_t value)
{
  return static_cast<TrackerState>(value);
}

}  // namespace armor_plate_interfaces