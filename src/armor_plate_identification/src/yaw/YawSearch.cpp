#include "armor_plate_identification/yaw/YawSearch.hpp"

#include "armor_plate_common/angle.hpp"

#include <cmath>
#include <limits>

namespace apc = armor_plate_common;

namespace armor_plate_identification
{
namespace yaw
{

namespace
{

constexpr double kBoundaryEpsilon = 1e-12;

bool isFinite(double value)
{
    return std::isfinite(value);
}

}  // namespace

/*
    修改参数入口
*/
YawSearchConfig defaultYawSearchConfig()
{
    return YawSearchConfig{
        apc::degToRad(30.0),
        apc::degToRad(4.0),
        apc::degToRad(3.0),
        8
    };
}

bool isValidYawSearchConfig(const YawSearchConfig& config, std::string* reason)
{
    auto fail = [&](const char* message) {
        if (reason != nullptr) {
            *reason = message;
        }
        return false;
    };

    if (!isFinite(config.search_range_rad) ||
        !isFinite(config.enumeration_step_rad) ||
        !isFinite(config.local_range_rad)) {
        return fail("config contains non-finite value");
    }

    if (!(config.search_range_rad > 0.0)) {
        return fail("search_range_rad must be > 0");
    }
    if (!(config.search_range_rad <= M_PI)) {
        return fail("search_range_rad must be <= PI");
    }

    if (!(config.enumeration_step_rad > 0.0)) {
        return fail("enumeration_step_rad must be > 0");
    }
    if (!(config.enumeration_step_rad <= 2.0 * config.search_range_rad)) {
        return fail("enumeration_step_rad must be <= 2 * search_range_rad");
    }

    if (!(config.local_range_rad > 0.0)) {
        return fail("local_range_rad must be > 0");
    }
    if (!(config.local_range_rad <= config.search_range_rad)) {
        return fail("local_range_rad must be <= search_range_rad");
    }

    if (!(config.ternary_iterations >= 0)) {
        return fail("ternary_iterations must be >= 0");
    }

    return true;
}

YawSearchResult runYawSearch(
    double center_yaw,
    const YawSearchConfig& config,
    const YawErrorFunction& calculate_error)
{
    YawSearchResult result;
    result.center_yaw = center_yaw;
    result.coarse_yaw = std::numeric_limits<double>::quiet_NaN();
    result.refined_yaw = std::numeric_limits<double>::quiet_NaN();
    result.coarse_error = std::numeric_limits<double>::quiet_NaN();
    result.evaluation_count = 0;
    result.coarse_at_boundary = false;

    std::string reason;
    if (!isValidYawSearchConfig(config, &reason)) {
        result.status = YawSearchStatus::InvalidConfig;
        return result;
    }

    if (!isFinite(center_yaw)) {
        result.status = YawSearchStatus::NoFiniteEvaluation;
        return result;
    }

    // 枚举阶段：保持旧公式 floor(2*range/step)+1
    const double start_yaw = center_yaw - config.search_range_rad;
    const int steps = static_cast<int>(2.0 * config.search_range_rad / config.enumeration_step_rad);

    double best_yaw = std::numeric_limits<double>::quiet_NaN();
    double min_error = std::numeric_limits<double>::infinity();
    double first_yaw = std::numeric_limits<double>::quiet_NaN();
    double last_yaw = std::numeric_limits<double>::quiet_NaN();
    bool has_finite_evaluation = false;

    for (int i = 0; i <= steps; ++i) {
        const double raw_yaw = start_yaw + static_cast<double>(i) * config.enumeration_step_rad;
        if (!isFinite(raw_yaw)) {
            continue;
        }
        const double yaw = apc::normalizeRadAngle(raw_yaw);
        if (i == 0) {
            first_yaw = yaw;
        }
        if (i == steps) {
            last_yaw = yaw;
        }

        const double error = calculate_error(yaw);
        ++result.evaluation_count;

        if (!isFinite(error)) {
            continue;
        }

        has_finite_evaluation = true;
        // 严格 <，保留首个最小值
        if (error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }

    if (!has_finite_evaluation) {
        result.status = YawSearchStatus::NoFiniteEvaluation;
        return result;
    }

    result.coarse_yaw = best_yaw;
    result.coarse_error = min_error;

    // 判断粗搜结果是否落在枚举边界
    if (isFinite(first_yaw) && isFinite(last_yaw) && isFinite(best_yaw)) {
        const double diff_first = std::abs(apc::shortestAngularDistance(best_yaw, first_yaw));
        const double diff_last = std::abs(apc::shortestAngularDistance(best_yaw, last_yaw));
        result.coarse_at_boundary = (diff_first < kBoundaryEpsilon) || (diff_last < kBoundaryEpsilon);
    }

    // 局部三分搜索：在连续未归一化区间上操作
    double l = best_yaw - config.local_range_rad;
    double r = best_yaw + config.local_range_rad;
    int iterations_remaining = config.ternary_iterations;

    while (iterations_remaining > 0) {
        const double m1 = l + (r - l) / 3.0;
        const double m2 = r - (r - l) / 3.0;

        double e1 = std::numeric_limits<double>::infinity();
        double e2 = std::numeric_limits<double>::infinity();

        if (isFinite(m1)) {
            e1 = calculate_error(apc::normalizeRadAngle(m1));
            ++result.evaluation_count;
        }
        if (isFinite(m2)) {
            e2 = calculate_error(apc::normalizeRadAngle(m2));
            ++result.evaluation_count;
        }

        const bool finite1 = isFinite(e1);
        const bool finite2 = isFinite(e2);

        if (!finite1 && !finite2) {
            result.refined_yaw = best_yaw;
            result.status = YawSearchStatus::RefinementFailed;
            return result;
        }

        // 非有限视为 +inf，选择有限侧
        if (!finite1) {
            // e1 = +inf, e2 有限 -> e2 更优，保留右侧 [m1, r]
            l = m1;
        } else if (!finite2) {
            // e2 = +inf, e1 有限 -> e1 更优，保留左侧 [l, m2]
            r = m2;
        } else if (e1 < e2) {
            // e1 更优，保留左侧 [l, m2]
            r = m2;
        } else {
            // e2 更优或相等，保留右侧 [m1, r]
            l = m1;
        }

        --iterations_remaining;
    }

    const double mid = (l + r) / 2.0;
    if (isFinite(mid)) {
        result.refined_yaw = apc::normalizeRadAngle(mid);
    } else {
        result.refined_yaw = best_yaw;
    }
    result.status = YawSearchStatus::Ok;
    return result;
}

}  // namespace yaw
}  // namespace armor_plate_identification
