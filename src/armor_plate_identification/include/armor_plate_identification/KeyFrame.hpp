#pragma once

#include "armor_plate_interfaces/GimbalData.hpp"

#include <cstdint>
#include <opencv2/core.hpp>

struct KeyFrame
{
    int64_t timestamp_ns = 0;
    cv::Mat image;
    GimbalData gimbal;
};
