#pragma once
#include "armor_plate_identification/DebugBase.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

struct DebugTestParams
{
    bool headless = false;
    bool debug_frame = false;
    int debug_frame_count = 100;
};

class DebugTest : public DebugBase
{
public:
    DebugTest() = default;
    DebugTest(const DebugBaseParams& base_params, const DebugTestParams& test_params);

    bool shouldShow() const override { return !test_params_.headless; }
    bool shouldExit() const { return test_params_.debug_frame && frame_count_ >= test_params_.debug_frame_count; }
    bool isDebugFrameMode() const { return test_params_.debug_frame; }
    int getDebugFrameCount() const { return test_params_.debug_frame_count; }

    void saveTrackerDebug(const std::string& log_dir, const armor_plate_interfaces::msg::TrackerDebug& msg);
    void closeTrackerDebugFile();

private:
    DebugTestParams test_params_;
};
