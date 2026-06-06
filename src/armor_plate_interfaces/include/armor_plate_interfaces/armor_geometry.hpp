#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <vector>
#include <cmath>

namespace armor_plate_interfaces
{

// 装甲板完整尺寸 (m)
static constexpr double SMALL_ARMOR_WIDTH  = 0.135;  // 135mm
static constexpr double SMALL_ARMOR_HEIGHT = 0.055;  // 55mm
static constexpr double LARGE_ARMOR_WIDTH  = 0.225;  // 225mm
static constexpr double LARGE_ARMOR_HEIGHT = 0.055;  // 55mm

// 装甲板半尺寸 (m) - 用于 PnP 解算
static constexpr double SMALL_HALF_WIDTH  = SMALL_ARMOR_WIDTH / 2.0;
static constexpr double SMALL_HALF_HEIGHT = SMALL_ARMOR_HEIGHT / 2.0;
static constexpr double LARGE_HALF_WIDTH  = LARGE_ARMOR_WIDTH / 2.0;
static constexpr double LARGE_HALF_HEIGHT = LARGE_ARMOR_HEIGHT / 2.0;

// pitch 倾角
static constexpr double ARMOR_PITCH_DEG = 15.0;
static constexpr double ARMOR_PITCH_RAD = ARMOR_PITCH_DEG * M_PI / 180.0;

// 3D 角点 (Eigen) - 坐标系: X轴法向量，y左，z上，顺时针：左上→右上→右下→左下
static const Eigen::Vector3d SMALL_ARMOR_POINTS_EIGEN[4] = {
    {0, SMALL_HALF_WIDTH,  SMALL_HALF_HEIGHT},
    {0, -SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT},
    {0, -SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT},
    {0, SMALL_HALF_WIDTH,  -SMALL_HALF_HEIGHT}
};

static const Eigen::Vector3d LARGE_ARMOR_POINTS_EIGEN[4] = {
    {0, LARGE_HALF_WIDTH,  LARGE_HALF_HEIGHT},
    {0, -LARGE_HALF_WIDTH, LARGE_HALF_HEIGHT},
    {0, -LARGE_HALF_WIDTH, -LARGE_HALF_HEIGHT},
    {0, LARGE_HALF_WIDTH,  -LARGE_HALF_HEIGHT}
};

// 3D 角点 (cv::Point3f)
static const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
    {0, SMALL_HALF_WIDTH,  SMALL_HALF_HEIGHT},
    {0, -SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT},
    {0, -SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT},
    {0, SMALL_HALF_WIDTH,  -SMALL_HALF_HEIGHT}
};

static const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
    {0, LARGE_HALF_WIDTH,  LARGE_HALF_HEIGHT},
    {0, -LARGE_HALF_WIDTH, LARGE_HALF_HEIGHT},
    {0, -LARGE_HALF_WIDTH, -LARGE_HALF_HEIGHT},
    {0, LARGE_HALF_WIDTH,  -LARGE_HALF_HEIGHT}
};

} // namespace armor_plate_interfaces
