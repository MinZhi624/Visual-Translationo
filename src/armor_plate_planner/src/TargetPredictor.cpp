#include "armor_plate_planner/TargetPredictor.hpp"

armor_plate_interfaces::msg::TrackedTarget TargetPredictor::predict(
    const armor_plate_interfaces::msg::TrackedTarget & state,
    double /*dt*/) const
{
    // TODO: 第一版不实现预测，直接返回原始状态
    return state;
}
