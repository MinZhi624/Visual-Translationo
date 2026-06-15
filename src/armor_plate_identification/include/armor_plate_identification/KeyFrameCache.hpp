#ifndef ARMOR_PLATE_IDENTIFICATION__KEYFRAMECACHE_HPP_
#define ARMOR_PLATE_IDENTIFICATION__KEYFRAMECACHE_HPP_

#include <unordered_map>
#include <memory>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/planner_debug.hpp"
#include "armor_plate_interfaces/GimbalData.hpp"

namespace armor_plate_identification
{

struct KeyFrame
{
  cv::Mat image;
  GimbalData gimbal;
};

struct KeyFrameRecord
{
  std::unique_ptr<KeyFrame> frame;
  armor_plate_interfaces::msg::TrackerDebug::ConstSharedPtr tracker_debug;
  armor_plate_interfaces::msg::PlannerDebug::ConstSharedPtr planner_debug;
};

class KeyFrameCache
{
public:
  explicit KeyFrameCache(size_t max_size = 100);

  /// Submit a frame image. Returns ready record if all three items are present, nullptr otherwise.
  std::unique_ptr<KeyFrameRecord> submitFrame(int64_t timestamp_ns, std::unique_ptr<KeyFrame> frame);

  /// Submit tracker debug. Returns ready record if all three items are present, nullptr otherwise.
  std::unique_ptr<KeyFrameRecord> submitTrackerDebug(int64_t timestamp_ns,
    armor_plate_interfaces::msg::TrackerDebug::ConstSharedPtr debug);

  /// Submit planner debug. Returns ready record if all three items are present, nullptr otherwise.
  std::unique_ptr<KeyFrameRecord> submitPlannerDebug(int64_t timestamp_ns,
    armor_plate_interfaces::msg::PlannerDebug::ConstSharedPtr debug);

  size_t size() const;

private:
  // Bitmask for tracking which items have arrived
  static constexpr uint8_t HAS_FRAME = 0x01;
  static constexpr uint8_t HAS_TRACKER = 0x02;
  static constexpr uint8_t HAS_PLANNER = 0x04;
  static constexpr uint8_t ALL_PRESENT = HAS_FRAME | HAS_TRACKER | HAS_PLANNER;

  struct CacheEntry
  {
    std::unique_ptr<KeyFrameRecord> record;
    uint8_t mask = 0;
    bool tracker_dup = false;
    bool planner_dup = false;
  };

  std::unordered_map<int64_t, CacheEntry> cache_;
  mutable std::mutex mutex_;
  size_t max_size_;

  void cleanup();
  std::unique_ptr<KeyFrameRecord> checkAndTake(int64_t timestamp_ns, CacheEntry & entry);
};

}  // namespace armor_plate_identification

#endif  // ARMOR_PLATE_IDENTIFICATION__KEYFRAMECACHE_HPP_
