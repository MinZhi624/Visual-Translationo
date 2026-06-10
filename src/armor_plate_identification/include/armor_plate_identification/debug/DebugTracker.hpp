#pragma once
#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_identification/GuiWorker.hpp"
#include "armor_plate_interfaces/GimbalData.hpp"
#include "armor_plate_common/angle.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

#include <opencv2/core.hpp>


class DebugTracker
{
private:
    static constexpr double ARMOR_PITCH = armor_plate_common::degToRad(15.0);
    PoseSolver* pose_solver_;
    GuiWorker* gui_worker_;
public:
    DebugTracker(PoseSolver* pose_solver, GuiWorker* gui_worker);

    static void drawCross(
        cv::Mat& img,
        const cv::Point2f& center,
        const cv::Scalar& color,
        int radius = 12);

    static void drawPredictedArmorPoints(
        cv::Mat& img,
        const armor_plate_interfaces::msg::TrackerDebug& msg,
        const PoseSolver& pose_solver,
        const GimbalData& gimbal);

    void drawTragetPoints(
        cv::Mat& img,
        const armor_plate_interfaces::msg::TrackerDebug& msg,
        const GimbalData& gimbal) const;

    void drawPredictedCar(
        cv::Mat& img,
        const armor_plate_interfaces::msg::TrackerDebug& msg,
        const GimbalData& gimbal) const;

    void pushTrackerDebugFrame(const cv::Mat& debug_img) const;

    static void infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug& msg);
    static void infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug::SharedPtr msg);
};
