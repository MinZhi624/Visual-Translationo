#include "armor_plate_identification/KeyFrameCache.hpp"
#include <algorithm>

namespace armor_plate_identification
{

KeyFrameCache::KeyFrameCache(size_t max_size)
: max_size_(max_size)
{
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::submitFrame(
    int64_t timestamp_ns, std::unique_ptr<KeyFrame> frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = cache_[timestamp_ns];
  if (!entry.record) {
    entry.record = std::make_unique<KeyFrameRecord>();
  }

  entry.record->frame = std::move(frame);
  entry.mask |= HAS_FRAME;

  return checkAndTake(timestamp_ns, entry);
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::submitTrackerDebug(
    int64_t timestamp_ns,
    armor_plate_interfaces::msg::TrackerDebug::ConstSharedPtr debug)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = cache_[timestamp_ns];
  if (!entry.record) {
    entry.record = std::make_unique<KeyFrameRecord>();
  }

  if (entry.mask & HAS_TRACKER) {
    entry.tracker_dup = true;
  }
  entry.record->tracker_debug = std::move(debug);
  entry.mask |= HAS_TRACKER;

  return checkAndTake(timestamp_ns, entry);
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::submitPlannerDebug(
    int64_t timestamp_ns,
    armor_plate_interfaces::msg::PlannerDebug::ConstSharedPtr debug)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = cache_[timestamp_ns];
  if (!entry.record) {
    entry.record = std::make_unique<KeyFrameRecord>();
  }

  if (entry.mask & HAS_PLANNER) {
    entry.planner_dup = true;
  }
  entry.record->planner_debug = std::move(debug);
  entry.mask |= HAS_PLANNER;

  return checkAndTake(timestamp_ns, entry);
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::checkAndTake(
    int64_t timestamp_ns, CacheEntry & entry)
{
  if (entry.mask == ALL_PRESENT) {
    auto result = std::move(entry.record);
    cache_.erase(timestamp_ns);
    return result;
  }
  cleanup();
  return nullptr;
}

size_t KeyFrameCache::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

void KeyFrameCache::cleanup()
{
  while (cache_.size() > max_size_) {
    auto oldest = std::min_element(cache_.begin(), cache_.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    cache_.erase(oldest);
  }
}

}  // namespace armor_plate_identification
