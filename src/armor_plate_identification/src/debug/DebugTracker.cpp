#include "armor_plate_identification/debug/DebugTracker.hpp"

#include <armor_plate_interfaces/ArmorPose.hpp>
#include <armor_plate_interfaces/ArmorTypes.hpp>

#include <rclcpp/logging.hpp>
#include <opencv2/imgproc.hpp>

DebugTracker::DebugTracker(PoseSolver* pose_solver, GuiWorker* gui_worker)
    : pose_solver_(pose_solver), gui_worker_(gui_worker)
{
}

namespace {
    void drawArmorRect(
        cv::Mat & img,
        const std::array<cv::Point2f, 4> & points,
        const cv::Scalar & color,
        int id)
    {
        for (int i = 0; i < 4; ++i) {
            cv::line(img, points[i], points[(i + 1) % 4], color, 2, cv::LINE_AA);
        }

        cv::Point2f center = (points[0] + points[2]) * 0.5f;
        cv::putText(
            img, std::to_string(id), center,
            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
}


void DebugTracker::drawCross(
    cv::Mat& img,
    const cv::Point2f& center,
    const cv::Scalar& color,
    int radius)
{
    cv::circle(img, center, radius, color, 2, cv::LINE_AA);
    cv::line(img, center + cv::Point2f(-radius, 0), center + cv::Point2f(radius, 0), color, 2, cv::LINE_AA);
    cv::line(img, center + cv::Point2f(0, -radius), center + cv::Point2f(0, radius), color, 2, cv::LINE_AA);
}


void DebugTracker::drawTragetPoints(
    cv::Mat& img,
    const armor_plate_interfaces::msg::TrackerDebug& msg,
    const GimbalData& gimbal) const
{
    if (!pose_solver_) return;

    // filtered：未丢失时绿色，丢失时用 EKF 预测点画黄色
    Eigen::Vector3d filtered_world(msg.filtered_point_world.x, msg.filtered_point_world.y, msg.filtered_point_world.z);
    cv::Point2f filtered_px = pose_solver_->xyzWorldToPixel(filtered_world, gimbal);
    if (filtered_px.x >= 0) {
        cv::Scalar color = msg.is_lost ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
        drawCross(img, filtered_px, color, 12);
    }
}


void DebugTracker::drawPredictedCar(
    cv::Mat& img,
    const armor_plate_interfaces::msg::TrackerDebug& msg,
    const GimbalData& gimbal) const
{
    if (!pose_solver_) return;
    drawPredictedArmorPoints(img, msg, *pose_solver_, gimbal);
}


void DebugTracker::pushTrackerDebugFrame(const cv::Mat& debug_img) const
{
    if (!gui_worker_) return;
    cv::Mat show_img;
    cv::resize(debug_img, show_img, cv::Size(), 0.5, 0.5);
    gui_worker_->pushFrame(DebugWindow::TRACKER_DEBUG, show_img);
}


void DebugTracker::infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug& msg)
{
    RCLCPP_INFO(rclcpp::get_logger("DEBUG_TRACKER"),
        "滤波:(%.4f,%.4f,%.4f,%.4f),"
        "EKF状态: 中心点(%.4f,%.4f,%.4f) 中心速度(%.4f,%.4f,%.4f), "
        "r = %.4f, l = %.4f, h = %.4f, 是否丢失:%d",
        msg.filtered_point_world.x, msg.filtered_point_world.y, msg.filtered_point_world.z, msg.filter_yaw,
        msg.center_world.x, msg.center_world.y, msg.center_world.z,
        msg.center_velocity.x, msg.center_velocity.y, msg.center_velocity.z,
        msg.center_r, msg.center_l, msg.center_h,
        msg.is_lost ? 1 : 0);
}

void DebugTracker::infoTrackerDebugMsg(const armor_plate_interfaces::msg::TrackerDebug::SharedPtr msg)
{
    if (msg) infoTrackerDebugMsg(*msg);
}

void DebugTracker::drawPredictedArmorPoints(
    cv::Mat & img,
    const armor_plate_interfaces::msg::TrackerDebug & msg,
    const PoseSolver & pose_solver,
    const GimbalData & gimbal)
{
    const auto & armor_points = msg.predicted_armor_points_world;
    const auto & armor_yaws = msg.predicted_armor_yaws_world;

    if (armor_points.size() < 4 || armor_yaws.size() < 4) return;

    ArmorType armor_type = armorNameToType(static_cast<ArmorName>(msg.armor_name));

    for (int i = 0; i < 4; i++) {
        ArmorPose armor_pose;
        armor_pose.xyz_world = Eigen::Vector3d(armor_points[i].x, armor_points[i].y, armor_points[i].z);
        armor_pose.yaw = armor_yaws[i];
        armor_pose.type = armor_type;

        auto projected_vec = pose_solver.reprojectArmor(armor_pose, gimbal);
        std::array<cv::Point2f, 4> projected_points = {
            projected_vec[0], projected_vec[1], projected_vec[2], projected_vec[3]
        };

        bool selected = msg.selected_armor_id == i;
        cv::Scalar color = selected ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 255, 255);
        drawArmorRect(img, projected_points, color, i);
    }

    // 绘制车中心点
    Eigen::Vector3d car_center_world(msg.center_world.x, msg.center_world.y, msg.center_world.z);
    cv::Point2f car_center_px = pose_solver.xyzWorldToPixel(car_center_world, gimbal);
    if (car_center_px.x >= 0) {
        cv::circle(img, car_center_px, 8, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }
}
