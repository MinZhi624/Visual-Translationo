#include "armor_plate_planner/TargetPredictor.hpp"

TargetState TargetPredictor::predict(
    const TargetState& state,
    double /*dt*/) const
{
    // TODO: 第一版不实现预测，直接返回原始状态
    return state;
}
