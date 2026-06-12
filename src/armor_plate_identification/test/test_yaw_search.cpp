#include "armor_plate_identification/yaw/YawSearch.hpp"

#include "armor_plate_common/angle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <string>

namespace api = armor_plate_identification;
namespace apc = armor_plate_common;

namespace
{

// 从旧 PoseSolver.cpp 复制并适配为独立误差函数的参考实现
double referenceSearchYawByEnumeration(
    double center_yaw,
    double range_rad,
    double step_rad,
    const api::yaw::YawErrorFunction& calculate_error)
{
    double best_yaw = center_yaw;
    double min_error = std::numeric_limits<double>::max();
    double start_yaw = center_yaw - range_rad;
    int steps = static_cast<int>(2.0 * range_rad / step_rad);
    for (int i = 0; i <= steps; ++i) {
        double yaw = apc::normalizeRadAngle(start_yaw + i * step_rad);
        double error = calculate_error(yaw);
        if (error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }
    return best_yaw;
}

double referenceSearchYawByTernary(
    double left_yaw,
    double right_yaw,
    int iterations,
    const api::yaw::YawErrorFunction& calculate_error)
{
    double l = left_yaw;
    double r = right_yaw;
    while (iterations--) {
        double m1 = l + (r - l) / 3.0;
        double m2 = r - (r - l) / 3.0;
        double e1 = calculate_error(apc::normalizeRadAngle(m1));
        double e2 = calculate_error(apc::normalizeRadAngle(m2));
        if (e1 < e2) {
            r = m2;
        } else {
            l = m1;
        }
    }
    return apc::normalizeRadAngle((l + r) / 2.0);
}

double referenceOptimizeYaw(
    double center_yaw,
    const api::yaw::YawSearchConfig& config,
    const api::yaw::YawErrorFunction& calculate_error)
{
    double coarse_yaw = referenceSearchYawByEnumeration(
        center_yaw,
        config.search_range_rad,
        config.enumeration_step_rad,
        calculate_error);
    double refined_yaw = referenceSearchYawByTernary(
        coarse_yaw - config.local_range_rad,
        coarse_yaw + config.local_range_rad,
        config.ternary_iterations,
        calculate_error);
    return refined_yaw;
}

}  // namespace

TEST(YawSearch, DefaultConfigEvaluationCount)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    const api::yaw::YawErrorFunction error = [](double yaw) {
        return (yaw - 0.5) * (yaw - 0.5);
    };
    const auto result = api::yaw::runYawSearch(0.0, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    EXPECT_EQ(result.evaluation_count, 90u);
}

TEST(YawSearch, KnownQuadraticMinimum)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    const double target = 0.0;
    const api::yaw::YawErrorFunction error = [target](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, target);
        return d * d;
    };
    const double center = 0.5;
    const auto result = api::yaw::runYawSearch(center, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    EXPECT_NEAR(result.refined_yaw, target, 1e-4);
    EXPECT_LT(result.coarse_error, 1e-3);
    EXPECT_FALSE(result.coarse_at_boundary);
}

TEST(YawSearch, OptimumCrossingPiBoundary)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    // 最小值在 -PI + 0.1 处，跨越 PI/-PI 边界
    const double target = -M_PI + 0.1;
    const api::yaw::YawErrorFunction error = [target](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, target);
        return d * d;
    };
    const double center = M_PI - 0.05;
    const auto result = api::yaw::runYawSearch(center, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    const double yaw_error = std::abs(apc::shortestAngularDistance(result.refined_yaw, target));
    EXPECT_NEAR(yaw_error, 0.0, 1e-4);
}

TEST(YawSearch, OptimumAtBoundary)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    // 最小值在枚举左边界 -search_range 处
    const double target = -config.search_range_rad;
    const api::yaw::YawErrorFunction error = [target](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, target);
        return d * d;
    };
    const auto result = api::yaw::runYawSearch(0.0, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    EXPECT_TRUE(result.coarse_at_boundary);
    const double yaw_error = std::abs(apc::shortestAngularDistance(result.refined_yaw, target));
    EXPECT_NEAR(yaw_error, 0.0, 1e-4);
}

TEST(YawSearch, ZeroTernaryIterations)
{
    auto config = api::yaw::defaultYawSearchConfig();
    config.ternary_iterations = 0;
    const api::yaw::YawErrorFunction error = [](double yaw) {
        return (yaw - 0.5) * (yaw - 0.5);
    };
    const auto result = api::yaw::runYawSearch(0.0, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    EXPECT_EQ(result.evaluation_count, 60u);
    EXPECT_DOUBLE_EQ(result.refined_yaw, result.coarse_yaw);
}

TEST(YawSearch, InvalidConfig)
{
    const api::yaw::YawErrorFunction error = [](double yaw) {
        return yaw * yaw;
    };

    {
        auto config = api::yaw::defaultYawSearchConfig();
        config.search_range_rad = 0.0;
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::InvalidConfig);
        EXPECT_EQ(result.evaluation_count, 0u);
        EXPECT_TRUE(std::isnan(result.coarse_yaw));
        EXPECT_TRUE(std::isnan(result.refined_yaw));
    }

    {
        auto config = api::yaw::defaultYawSearchConfig();
        config.search_range_rad = M_PI + 0.1;
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::InvalidConfig);
    }

    {
        auto config = api::yaw::defaultYawSearchConfig();
        config.enumeration_step_rad = 2.0 * config.search_range_rad + 0.1;
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::InvalidConfig);
    }

    {
        auto config = api::yaw::defaultYawSearchConfig();
        config.local_range_rad = config.search_range_rad + 0.1;
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::InvalidConfig);
    }

    {
        auto config = api::yaw::defaultYawSearchConfig();
        config.ternary_iterations = -1;
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::InvalidConfig);
    }

    {
        std::string reason;
        auto config = api::yaw::defaultYawSearchConfig();
        config.search_range_rad = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(api::yaw::isValidYawSearchConfig(config, &reason));
        EXPECT_FALSE(reason.empty());
    }
}

TEST(YawSearch, SingleNonFiniteEnumerationError)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    // 在 yaw == 0 处返回 NaN，其余为二次误差；枚举包含 yaw == 0 点
    const api::yaw::YawErrorFunction error = [](double yaw) {
        if (std::abs(yaw) < 1e-12) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return (yaw - 0.5) * (yaw - 0.5);
    };
    const auto result = api::yaw::runYawSearch(0.0, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
    EXPECT_NEAR(result.refined_yaw, 0.5, 1e-4);
}

TEST(YawSearch, AllNonFiniteEnumerationErrors)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    const api::yaw::YawErrorFunction error = [](double) {
        return std::numeric_limits<double>::quiet_NaN();
    };
    const auto result = api::yaw::runYawSearch(0.0, config, error);
    EXPECT_EQ(result.status, api::yaw::YawSearchStatus::NoFiniteEvaluation);
    EXPECT_EQ(result.evaluation_count, 60u);
    EXPECT_TRUE(std::isnan(result.coarse_yaw));
}

TEST(YawSearch, NonFiniteTernaryErrors)
{
    const auto config = api::yaw::defaultYawSearchConfig();

    // 一个非有限：局部区间跨越 NaN 边界，右侧正常
    {
        const double threshold = 0.1;
        const double target = 0.15;
        const api::yaw::YawErrorFunction error = [threshold, target](double yaw) {
            if (yaw < threshold) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            return (yaw - target) * (yaw - target);
        };
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok);
        EXPECT_NEAR(result.refined_yaw, target, 1e-4);
    }

    // 两个都非有限：枚举阶段即全部 NaN
    {
        const api::yaw::YawErrorFunction error = [](double) {
            return std::numeric_limits<double>::quiet_NaN();
        };
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::NoFiniteEvaluation);
    }

    // 粗搜有有限值，但三分局部区间内几乎全为非有限
    {
        const api::yaw::YawErrorFunction error = [](double yaw) {
            const double deg = apc::radToDeg(yaw);
            const double nearest = std::round(deg);
            // 仅在整数度附近有限，三分搜索几乎总会探测到非有限值
            if (std::abs(deg - nearest) < 1e-3) {
                return (yaw - 0.5) * (yaw - 0.5);
            }
            return std::numeric_limits<double>::quiet_NaN();
        };
        const auto result = api::yaw::runYawSearch(0.0, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::RefinementFailed);
        EXPECT_TRUE(std::isfinite(result.coarse_yaw));
        EXPECT_DOUBLE_EQ(result.refined_yaw, result.coarse_yaw);
    }
}

TEST(YawSearch, OldVsNewEquivalence)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> center_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> target_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> scale_dist(0.1, 10.0);
    std::uniform_real_distribution<double> offset_dist(-5.0, 5.0);

    auto run_case = [&](const api::yaw::YawErrorFunction& error, double center) {
        const double expected = referenceOptimizeYaw(center, config, error);
        const auto result = api::yaw::runYawSearch(center, config, error);
        EXPECT_EQ(result.status, api::yaw::YawSearchStatus::Ok)
            << "center=" << center;
        const double yaw_diff = std::abs(apc::shortestAngularDistance(result.refined_yaw, expected));
        EXPECT_NEAR(yaw_diff, 0.0, 1e-12)
            << "center=" << center << " expected=" << expected
            << " actual=" << result.refined_yaw;
        const double expected_error = error(expected);
        const double actual_error = error(result.refined_yaw);
        EXPECT_NEAR(actual_error, expected_error, 1e-12)
            << "center=" << center;
    };

    // 固定边界与跨 PI 用例
    run_case([&config](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, -config.search_range_rad);
        return d * d;
    }, 0.0);

    run_case([](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, -M_PI + 0.1);
        return d * d;
    }, M_PI - 0.05);

    run_case([](double yaw) {
        const double d = apc::shortestAngularDistance(yaw, M_PI - 0.05);
        return d * d;
    }, -M_PI + 0.1);

    run_case([](double yaw) {
        return (yaw - 0.123456789) * (yaw - 0.123456789);
    }, 0.0);

    // 随机二次型函数
    for (int i = 0; i < 200; ++i) {
        const double center = center_dist(rng);
        const double target = target_dist(rng);
        const double scale = scale_dist(rng);
        const double offset = offset_dist(rng);
        run_case([scale, offset, target](double yaw) {
            const double d = apc::shortestAngularDistance(yaw, target);
            return scale * d * d + offset;
        }, center);
    }
}

TEST(YawSearch, RepeatedRunsIdentical)
{
    const auto config = api::yaw::defaultYawSearchConfig();
    const api::yaw::YawErrorFunction error = [](double yaw) {
        return std::sin(3.0 * yaw) * std::sin(3.0 * yaw) + 0.1 * yaw * yaw;
    };
    const auto r1 = api::yaw::runYawSearch(0.5, config, error);
    const auto r2 = api::yaw::runYawSearch(0.5, config, error);
    EXPECT_EQ(r1.status, r2.status);
    EXPECT_DOUBLE_EQ(r1.coarse_yaw, r2.coarse_yaw);
    EXPECT_DOUBLE_EQ(r1.refined_yaw, r2.refined_yaw);
    EXPECT_DOUBLE_EQ(r1.coarse_error, r2.coarse_error);
    EXPECT_EQ(r1.evaluation_count, r2.evaluation_count);
    EXPECT_EQ(r1.coarse_at_boundary, r2.coarse_at_boundary);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
