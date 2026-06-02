#include "armor_plate_identification/DebugTracker.hpp"

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

    // target：未丢失时画红色，丢失时不画
    if (!msg.is_lost) {
        Eigen::Vector3d target_world(msg.target_point_world.x, msg.target_point_world.y, msg.target_point_world.z);
        cv::Point2f target_px = pose_solver_->xyzWorldToPixel(target_world, gimbal);
        if (target_px.x >= 0) {
            drawCross(img, target_px, cv::Scalar(0, 0, 255), 12);
        }
    }

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
        "world:(%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f) "
        "yaw:%.4f->%.4f "
        "center:(%.4f,%.4f,%.4f) r:%.4f v:(%.4f,%.4f) "
        "lost:%d",
        msg.target_point_world.x, msg.target_point_world.y, msg.target_point_world.z,
        msg.filtered_point_world.x, msg.filtered_point_world.y, msg.filtered_point_world.z,
        msg.raw_yaw, msg.filter_yaw,
        msg.center_x, msg.center_y, msg.center_z, msg.center_r,
        msg.center_v_x, msg.center_v_y,
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
    // 建立本地坐标系装甲板
    const std::array<Eigen::Vector3d, 4> local_armor_points = {
        Eigen::Vector3d(0.0, SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT),
        Eigen::Vector3d(0.0, -SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT),
        Eigen::Vector3d(0.0, -SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT),
        Eigen::Vector3d(0.0, SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT)
    };

    for(int i = 0; i < 4; i++) {
        const auto & armor_point = armor_points[i];
        const auto & armor_yaw = armor_yaws[i];
        Eigen::Vector3d armor_center = {armor_point.x, armor_point.y, armor_point.z};

        Eigen::Matrix3d Rz = Eigen::AngleAxisd(armor_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(ARMOR_PITCH, Eigen::Vector3d::UnitY()).toRotationMatrix();
        Eigen::Matrix3d R_world_armor = Rz * Ry;

        std::array<cv::Point2f, 4> projected_points;
        for(int j = 0; j < 4; j++) {
            Eigen::Vector3d projected_point = armor_center + R_world_armor * local_armor_points[j];
            projected_points[j] = pose_solver.xyzWorldToPixel(projected_point, gimbal);
        }
        // 绘制装甲板
        bool selected = msg.selected_armor_id == i;
        // 目标选中的是蓝色
        cv::Scalar color = selected ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 255, 255);
        drawArmorRect(img, projected_points, color, i);
        
    }
    // 绘制车中心点
    Eigen::Vector3d car_center_world(msg.center_x, msg.center_y, msg.center_z);
    cv::Point2f car_center_px = pose_solver.xyzWorldToPixel(car_center_world, gimbal);
    if (car_center_px.x >= 0) {
        cv::circle(img, car_center_px, 8, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }
}
