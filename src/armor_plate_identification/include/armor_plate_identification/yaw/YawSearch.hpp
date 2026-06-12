#pragma once

#include <cstddef>
#include <functional>
#include <string>

namespace armor_plate_identification
{
namespace yaw
{

enum class YawSearchStatus {
    Ok,
    InvalidConfig,
    NoFiniteEvaluation,
    RefinementFailed
};

struct YawSearchConfig {
    double search_range_rad;
    double enumeration_step_rad;
    double local_range_rad;
    int ternary_iterations;
};

struct YawSearchResult {
    double center_yaw = 0.0;
    double coarse_yaw = 0.0;
    double refined_yaw = 0.0;
    double coarse_error = 0.0;
    std::size_t evaluation_count = 0;
    bool coarse_at_boundary = false;
    YawSearchStatus status = YawSearchStatus::Ok;
};

using YawErrorFunction = std::function<double(double)>;

YawSearchResult runYawSearch(
    double center_yaw,
    const YawSearchConfig& config,
    const YawErrorFunction& calculate_error);

YawSearchConfig defaultYawSearchConfig();
bool isValidYawSearchConfig(const YawSearchConfig& config, std::string* reason = nullptr);

}  // namespace yaw
}  // namespace armor_plate_identification
