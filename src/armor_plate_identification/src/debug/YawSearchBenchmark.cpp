#include "armor_plate_identification/debug/YawSearchBenchmark.hpp"
#include "armor_plate_identification/DetectorArmor.hpp"

#include "armor_plate_common/angle.hpp"
#include "armor_plate_interfaces/ArmorTypes.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>

namespace apc = armor_plate_common;

namespace armor_plate_identification
{
namespace debug
{

namespace
{

constexpr double kReferenceSearchRangeDeg = 30.0;
constexpr double kReferenceEnumerationStepDeg = 0.05;
constexpr double kReferenceLocalRangeDeg = 0.1;
constexpr int kReferenceTernaryIterations = 25;

constexpr double kCandidateSearchRangeDeg = 30.0;
constexpr double kCandidateLocalRangeDeg = 3.0;

constexpr double kObservableDeltaDeg = 0.5;
constexpr double kObservableThresholdPxPerCorner = 0.01;
constexpr double kLocalBasinThresholdDeg = 3.0;

bool isFinite(double value)
{
    return std::isfinite(value);
}

}  // namespace

YawSearchBenchmark::YawSearchBenchmark(rclcpp::Node* node, const YawSearchBenchmarkParams& params)
    : node_(node), params_(params)
{
    generateParamCombos();
    openCsv();
}

YawSearchBenchmark::~YawSearchBenchmark()
{
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}

void YawSearchBenchmark::generateParamCombos()
{
    param_combos_.reserve(params_.steps_deg.size() * params_.iterations.size());
    for (double step : params_.steps_deg) {
        for (int iter : params_.iterations) {
            param_combos_.emplace_back(step, iter);
        }
    }
}

void YawSearchBenchmark::openCsv()
{
    partial_path_ = params_.output_csv + ".partial";
    final_path_ = params_.output_csv;

    std::filesystem::path output_path(final_path_);
    std::filesystem::create_directories(output_path.parent_path());

    csv_file_.open(partial_path_);
    if (!csv_file_.is_open()) {
        throw std::runtime_error("无法打开 benchmark 临时 CSV: " + partial_path_);
    }
    csv_file_.imbue(std::locale::classic());
    csv_file_ << kSchemaHeader << "\n";
    csv_open_ = true;
}

void YawSearchBenchmark::onFrameStart(std::size_t raw_frame_index)
{
    raw_frame_index_ = raw_frame_index;

    if (params_.max_samples > 0 && sample_count_ >= static_cast<std::size_t>(params_.max_samples)) {
        should_stop_ = true;
        is_sample_frame_ = false;
        return;
    }

    if (raw_frame_index_ % static_cast<std::size_t>(params_.sample_stride) == 0) {
        is_sample_frame_ = true;
        current_sample_id_ = sample_count_++;
        current_frame_index_ = raw_frame_index_;
    } else {
        is_sample_frame_ = false;
    }
}

void YawSearchBenchmark::onYawSearch(
    const DetectorArmor& armor,
    double center_yaw,
    const armor_plate_identification::yaw::YawSearchConfig& /*config*/,
    const armor_plate_identification::yaw::YawSearchResult& /*result*/,
    const YawErrorFunction& calculate_error,
    std::size_t armor_index)
{
    if (!is_sample_frame_) {
        return;
    }

    runReferenceAndSweep(armor, center_yaw, calculate_error, armor_index);
}

void YawSearchBenchmark::runReferenceAndSweep(
    const DetectorArmor& armor,
    double center_yaw,
    const YawErrorFunction& calculate_error,
    std::size_t armor_index)
{
    using armor_plate_identification::yaw::runYawSearch;
    using armor_plate_identification::yaw::YawSearchConfig;
    using armor_plate_identification::yaw::YawSearchStatus;

    // 稠密参考搜索，不改变生产路径
    YawSearchConfig reference_config;
    reference_config.search_range_rad = apc::degToRad(kReferenceSearchRangeDeg);
    reference_config.enumeration_step_rad = apc::degToRad(kReferenceEnumerationStepDeg);
    reference_config.local_range_rad = apc::degToRad(kReferenceLocalRangeDeg);
    reference_config.ternary_iterations = kReferenceTernaryIterations;

    const auto reference_result = runYawSearch(center_yaw, reference_config, calculate_error);

    double reference_yaw = std::numeric_limits<double>::quiet_NaN();
    double reference_error = std::numeric_limits<double>::quiet_NaN();
    bool reference_boundary = false;
    bool reference_valid = false;

    if (reference_result.status == YawSearchStatus::Ok) {
        reference_yaw = reference_result.refined_yaw;
        reference_error = calculate_error(reference_yaw);
        reference_boundary = reference_result.coarse_at_boundary;
        reference_valid = true;
    } else if (reference_result.status == YawSearchStatus::RefinementFailed) {
        reference_yaw = reference_result.coarse_yaw;
        reference_error = reference_result.coarse_error;
        reference_boundary = reference_result.coarse_at_boundary;
        reference_valid = true;
    }

    // 参考误差非有限、装甲板在相机后方、投影点非有限均视为无效
    const bool armor_behind_camera = !isFinite(armor.xyz_camera_.z()) || armor.xyz_camera_.z() <= 0.0;
    const bool reference_invalid = !reference_valid || !isFinite(reference_yaw) || !isFinite(reference_error);
    const bool armor_invalid = reference_invalid || armor_behind_camera;

    // 可观测性：参考解两侧 0.5° 的每角点误差增量均 >= 0.01 px
    bool observable = false;
    if (!armor_invalid && isFinite(reference_yaw)) {
        const double delta = apc::degToRad(kObservableDeltaDeg);
        const double e_plus = calculate_error(apc::normalizeRadAngle(reference_yaw + delta));
        const double e_minus = calculate_error(apc::normalizeRadAngle(reference_yaw - delta));
        if (isFinite(e_plus) && isFinite(e_minus) && isFinite(reference_error)) {
            const double per_corner_ref = reference_error / 4.0;
            const double inc_plus = e_plus / 4.0 - per_corner_ref;
            const double inc_minus = e_minus / 4.0 - per_corner_ref;
            observable = (inc_plus >= kObservableThresholdPxPerCorner) &&
                         (inc_minus >= kObservableThresholdPxPerCorner);
        }
    }

    // 参数组合按 sample id 循环移位，减少系统性的时序偏置
    auto combos = param_combos_;
    if (!combos.empty()) {
        const std::size_t offset = current_sample_id_ % combos.size();
        std::rotate(combos.begin(), combos.begin() + offset, combos.end());
    }

    const bool timing_valid = static_cast<int>(current_sample_id_) >= params_.warmup_samples;

    for (const auto& combo : combos) {
        const double step_deg = combo.first;
        const int iterations = combo.second;

        YawSearchConfig candidate_config;
        candidate_config.search_range_rad = apc::degToRad(kCandidateSearchRangeDeg);
        candidate_config.enumeration_step_rad = apc::degToRad(step_deg);
        candidate_config.local_range_rad = apc::degToRad(kCandidateLocalRangeDeg);
        candidate_config.ternary_iterations = iterations;

        const auto t0 = std::chrono::steady_clock::now();
        const auto candidate_result = runYawSearch(center_yaw, candidate_config, calculate_error);
        const auto t1 = std::chrono::steady_clock::now();
        const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

        double coarse_yaw = candidate_result.coarse_yaw;
        double refined_yaw = std::numeric_limits<double>::quiet_NaN();
        double refined_error = std::numeric_limits<double>::quiet_NaN();

        if (candidate_result.status == YawSearchStatus::Ok) {
            refined_yaw = candidate_result.refined_yaw;
            refined_error = calculate_error(refined_yaw);
        } else if (candidate_result.status == YawSearchStatus::RefinementFailed) {
            refined_yaw = candidate_result.coarse_yaw;
            refined_error = candidate_result.coarse_error;
        }

        const bool row_invalid = armor_invalid || !isFinite(refined_error);

        double yaw_error_deg = std::numeric_limits<double>::quiet_NaN();
        double error_regret_sum_px = std::numeric_limits<double>::quiet_NaN();
        double error_regret_per_corner_px = std::numeric_limits<double>::quiet_NaN();
        bool coarse_in_local_basin = false;

        if (!row_invalid && isFinite(reference_yaw) && isFinite(refined_yaw)) {
            yaw_error_deg = std::abs(apc::shortestAngularDistance(refined_yaw, reference_yaw)) * 180.0 / M_PI;
            error_regret_sum_px = refined_error - reference_error;
            error_regret_per_corner_px = error_regret_sum_px / 4.0;
        }
        if (isFinite(reference_yaw) && isFinite(coarse_yaw)) {
            coarse_in_local_basin = std::abs(apc::shortestAngularDistance(coarse_yaw, reference_yaw)) <=
                                    apc::degToRad(kLocalBasinThresholdDeg);
        }

        writeRow(
            armor,
            current_sample_id_,
            current_frame_index_,
            armor_index,
            center_yaw,
            reference_yaw,
            reference_error,
            reference_boundary,
            observable,
            row_invalid,
            step_deg,
            iterations,
            coarse_yaw,
            refined_yaw,
            candidate_result.coarse_error,
            refined_error,
            yaw_error_deg,
            error_regret_sum_px,
            error_regret_per_corner_px,
            coarse_in_local_basin,
            candidate_result.evaluation_count,
            elapsed_ns,
            timing_valid);

        if (csv_file_.fail()) {
            RCLCPP_ERROR(node_->get_logger(), "YawSearchBenchmark CSV 写入失败");
            csv_open_ = false;
            should_stop_ = true;
            return;
        }
    }
}

void YawSearchBenchmark::writeRow(
    const DetectorArmor& armor,
    std::size_t sample_id,
    std::size_t frame_index,
    std::size_t armor_index,
    double init_yaw_rad,
    double reference_yaw_rad,
    double reference_error_sum_px,
    bool reference_boundary,
    bool observable,
    bool invalid,
    double step_deg,
    int iterations,
    double coarse_yaw_rad,
    double refined_yaw_rad,
    double coarse_error_sum_px,
    double refined_error_sum_px,
    double yaw_error_deg,
    double error_regret_sum_px,
    double error_regret_per_corner_px,
    bool coarse_in_local_basin,
    std::size_t evaluation_count,
    std::int64_t elapsed_ns,
    bool timing_valid)
{
    if (!csv_open_) {
        return;
    }

    auto write_double = [this](double value) {
        if (std::isnan(value)) {
            csv_file_ << "nan";
        } else if (std::isinf(value)) {
            csv_file_ << (value > 0.0 ? "inf" : "-inf");
        } else {
            csv_file_ << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        }
    };

    csv_file_ << kSchemaVersion << ",";
    csv_file_ << params_.video_name << ",";
    csv_file_ << sample_id << ",";
    csv_file_ << frame_index << ",";
    csv_file_ << armor_index << ",";
    csv_file_ << armorNameToString(armor.name_) << ",";
    csv_file_ << (armor.type_ == ArmorType::LARGE ? "LARGE" : "SMALL") << ",";
    write_double(armor.xyz_camera_.norm());
    csv_file_ << ",";
    write_double(armor.image_distance_to_center_);
    csv_file_ << ",";
    write_double(init_yaw_rad);
    csv_file_ << ",";
    write_double(reference_yaw_rad);
    csv_file_ << ",";
    write_double(reference_error_sum_px);
    csv_file_ << ",";
    csv_file_ << (reference_boundary ? "1" : "0") << ",";
    csv_file_ << (observable ? "1" : "0") << ",";
    csv_file_ << (invalid ? "1" : "0") << ",";
    write_double(step_deg);
    csv_file_ << ",";
    csv_file_ << iterations << ",";
    write_double(coarse_yaw_rad);
    csv_file_ << ",";
    write_double(refined_yaw_rad);
    csv_file_ << ",";
    write_double(coarse_error_sum_px);
    csv_file_ << ",";
    write_double(refined_error_sum_px);
    csv_file_ << ",";
    write_double(yaw_error_deg);
    csv_file_ << ",";
    write_double(error_regret_sum_px);
    csv_file_ << ",";
    write_double(error_regret_per_corner_px);
    csv_file_ << ",";
    csv_file_ << (coarse_in_local_basin ? "1" : "0") << ",";
    csv_file_ << evaluation_count << ",";
    csv_file_ << elapsed_ns << ",";
    csv_file_ << (timing_valid ? "1" : "0") << "\n";
}

bool YawSearchBenchmark::finalize()
{
    if (!csv_open_) {
        return true;
    }

    csv_file_.close();
    csv_open_ = false;

    try {
        std::filesystem::rename(partial_path_, final_path_);
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "YawSearchBenchmark CSV 重命名失败: %s", e.what());
        return false;
    }
}

}  // namespace debug
}  // namespace armor_plate_identification
