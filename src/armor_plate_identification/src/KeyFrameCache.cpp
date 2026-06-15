#include "armor_plate_identification/KeyFrameCache.hpp"
#include <algorithm>

KeyFrameCache::KeyFrameCache(size_t max_size)
: max_size_(max_size)
{
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::submitFrame(std::unique_ptr<KeyFrame> frame)
{
  if (!frame) return nullptr;

  const int64_t timestamp_ns = frame->timestamp_ns;
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = cache_[timestamp_ns];
  if (!entry.record) {
    entry.record = std::make_unique<KeyFrameRecord>();
  }

  entry.record->frame = std::move(frame);
  updateMask(entry.mask, Arrival::FRAME);

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

  entry.record->tracker_debug = std::move(debug);
  updateMask(entry.mask, Arrival::TRACKER);

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

  entry.record->planner_debug = std::move(debug);
  updateMask(entry.mask, Arrival::PLANNER);

  return checkAndTake(timestamp_ns, entry);
}

std::unique_ptr<KeyFrameRecord> KeyFrameCache::checkAndTake(
    int64_t timestamp_ns, CacheEntry & entry)
{
  if (entry.mask == Arrival::ALL_PRESENT) {
    auto result = std::move(entry.record);
    cache_.erase(timestamp_ns);
    return result;
  }
  evict();
  return nullptr;
}

size_t KeyFrameCache::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

void KeyFrameCache::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cache_.clear();
}

void KeyFrameCache::evict()
{
  while (cache_.size() > max_size_) {
    auto oldest = std::min_element(cache_.begin(), cache_.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    cache_.erase(oldest);
  }
}

void KeyFrameCache::updateMask(Arrival & mask, Arrival item)
{
  mask = mask | item;
}
