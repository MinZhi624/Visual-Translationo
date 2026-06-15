#pragma once

#include <unordered_map>
#include <memory>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "armor_plate_interfaces/msg/planner_debug.hpp"
#include "armor_plate_interfaces/GimbalData.hpp"

/** @brief 关键帧，包含图像和云台数据 */
struct KeyFrame
{
  cv::Mat image;
  GimbalData gimbal;
};

/** @brief 关键帧完整记录，包含图像帧、Tracker 调试数据和 Planner 调试数据 */
struct KeyFrameRecord
{
  std::unique_ptr<KeyFrame> frame;
  armor_plate_interfaces::msg::TrackerDebug::ConstSharedPtr tracker_debug;
  armor_plate_interfaces::msg::PlannerDebug::ConstSharedPtr planner_debug;
};

/**
 * @brief 关键帧缓存，按时间戳聚合图像帧、TrackerDebug、PlannerDebug
 *
 * 三者到齐后立即返回完整的 KeyFrameRecord，否则返回 nullptr。
 * 内部维护一个带容量上限的 LRU 缓存，自动淘汰最旧的未完成条目。
 */
class KeyFrameCache
{
private:
  /** @brief 缓存条目的到达状态 */
  enum class Arrival : uint8_t {
    NONE        = 0x00,
    FRAME       = 0x01,
    TRACKER     = 0x02,
    PLANNER     = 0x04,
    ALL_PRESENT = 0x07,
  };

  friend Arrival operator|(Arrival a, Arrival b)
  {
    return static_cast<Arrival>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }

  /** @brief 更新到达掩码 */
  static void updateMask(Arrival & mask, Arrival item);

  /** @brief 缓存条目，包含记录和到达状态 */
  struct CacheEntry
  {
    std::unique_ptr<KeyFrameRecord> record;
    Arrival mask = Arrival::NONE;
  };

  std::unordered_map<int64_t, CacheEntry> cache_;
  
  // mutable 保证了const函数可以使用
  mutable std::mutex mutex_;
  size_t max_size_;

  /** @brief 淘汰最旧的未完成条目，保持缓存容量不超限 */
  void evict();

  /** @brief 检查三者是否到齐，到齐则取出并移除缓存条目 */
  std::unique_ptr<KeyFrameRecord> checkAndTake(int64_t timestamp_ns, CacheEntry & entry);
public:
  explicit KeyFrameCache(size_t max_size = 100);

  /**
   * @brief 提交图像帧
   * @param timestamp_ns 时间戳（纳秒）
   * @param frame 图像帧（所有权转移）
   * @return 三者到齐时返回完整记录，否则返回 nullptr
   */
  std::unique_ptr<KeyFrameRecord> submitFrame(int64_t timestamp_ns, std::unique_ptr<KeyFrame> frame);

  /**
   * @brief 提交 Tracker 调试数据
   * @param timestamp_ns 时间戳（纳秒）
   * @param debug TrackerDebug 消息（共享指针）
   * @return 三者到齐时返回完整记录，否则返回 nullptr
   */
  std::unique_ptr<KeyFrameRecord> submitTrackerDebug(int64_t timestamp_ns,
    armor_plate_interfaces::msg::TrackerDebug::ConstSharedPtr debug);

  /**
   * @brief 提交 Planner 调试数据
   * @param timestamp_ns 时间戳（纳秒）
   * @param debug PlannerDebug 消息（共享指针）
   * @return 三者到齐时返回完整记录，否则返回 nullptr
   */
  std::unique_ptr<KeyFrameRecord> submitPlannerDebug(int64_t timestamp_ns,
    armor_plate_interfaces::msg::PlannerDebug::ConstSharedPtr debug);

  /** @brief 当前缓存中的条目数 */
  size_t size() const;
};
