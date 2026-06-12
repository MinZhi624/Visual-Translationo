#pragma once

#include "armor_plate_identification/yaw/IYawSearchObserver.hpp"
#include "armor_plate_identification/yaw/YawSearch.hpp"

#include <rclcpp/node.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace armor_plate_identification
{
namespace debug
{

struct YawSearchBenchmarkParams {
    bool enabled = false;
    std::vector<double> steps_deg;
    std::vector<int> iterations;
    int sample_stride = 5;
    int max_samples = 2000;
    int warmup_samples = 50;
    std::string output_csv;
    std::string video_name;
};

class YawSearchBenchmark : public IYawSearchObserver {
public:
    YawSearchBenchmark(rclcpp::Node* node, const YawSearchBenchmarkParams& params);
    ~YawSearchBenchmark() override;

    void onFrameStart(std::size_t raw_frame_index);
    bool shouldStop() const { return should_stop_.load(); }
    bool finalize();

private:
    void onYawSearch(
        const DetectorArmor& armor,
        double center_yaw,
        const armor_plate_identification::yaw::YawSearchConfig& config,
        const armor_plate_identification::yaw::YawSearchResult& result,
        const YawErrorFunction& calculate_error,
        std::size_t armor_index) override;

    void generateParamCombos();
    void openCsv();
    void runReferenceAndSweep(
        const DetectorArmor& armor,
        double center_yaw,
        const YawErrorFunction& calculate_error,
        std::size_t armor_index);
    void writeRow(
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
        bool timing_valid);

    rclcpp::Node* node_;
    YawSearchBenchmarkParams params_;
    std::vector<std::pair<double, int>> param_combos_;

    std::size_t raw_frame_index_ = 0;
    std::size_t sample_count_ = 0;
    std::size_t current_sample_id_ = 0;
    std::size_t current_frame_index_ = 0;
    bool is_sample_frame_ = false;
    std::atomic<bool> should_stop_{false};

    std::ofstream csv_file_;
    std::string partial_path_;
    std::string final_path_;
    bool csv_open_ = false;

    static constexpr int kSchemaVersion = 1;
    static constexpr const char* kSchemaHeader =
        "schema_version,video,sample_id,frame_index,armor_index,armor_name,armor_type,"
        "distance_m,image_center_distance_px,init_yaw_rad,reference_yaw_rad,reference_error_sum_px,"
        "reference_boundary,observable,invalid,step_deg,iterations,coarse_yaw_rad,refined_yaw_rad,"
        "coarse_error_sum_px,refined_error_sum_px,yaw_error_deg,error_regret_sum_px,"
        "error_regret_per_corner_px,coarse_in_local_basin,evaluation_count,elapsed_ns,timing_valid";
};

}  // namespace debug
}  // namespace armor_plate_identification
