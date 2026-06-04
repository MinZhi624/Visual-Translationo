#pragma once

#include <cmath>

namespace armor_plate_common
{

constexpr double normalizeRadAngle(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

constexpr double shortestAngularDistance(double from, double to)
{
    return normalizeRadAngle(to - from);
}

constexpr double degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

constexpr double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

}  // namespace armor_plate_common
