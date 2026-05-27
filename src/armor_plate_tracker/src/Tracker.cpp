#include "armor_plate_tracker/Tracker.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "rclcpp/logging.hpp"
#include <chrono>
#include <limits>
#include <rclcpp/logger.hpp>
#include <vector>

using armor_plate_interfaces::msg::TrackerDebug;

static float normalizeRadAngle(float rad)
{
    while (rad > M_PI) rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

static constexpr float MIN_VALID_ARMOR_PITCH_WORLD = -0.05f;

static double getArmorDist(const Eigen::Vector<double, 4> & armor, const TrackerArmor & target)
{
    double dx = armor.x() - target.xyz_world_.x();
    double dy = armor.y() - target.xyz_world_.y();
    return std::sqrt(dx * dx + dy * dy);
}

static double getArmorAngleDiff(const Eigen::Vector<double, 4> & armor, float yaw)
{
    return std::abs(normalizeRadAngle(yaw - armor.w()));
}

static size_t pickArmorIdx(const std::vector<Eigen::Vector<double, 4>> & list,
                           const TrackerArmor & target,
                           float yaw,
                           size_t & far_idx)
{
    far_idx = 0;
    double max_dist = getArmorDist(list[0], target);
    for (size_t i = 1; i < list.size(); ++i) {
        double dist = getArmorDist(list[i], target);
        if (dist > max_dist) {
            max_dist = dist;
            far_idx = i;
        }
    }

    size_t best_idx = far_idx == 0 ? 1 : 0;
    double min_ang = getArmorAngleDiff(list[best_idx], yaw);
    for (size_t i = 0; i < list.size(); ++i) {
        if (i == far_idx) continue;
        double ang = getArmorAngleDiff(list[i], yaw);
        if (ang < min_ang) {
            min_ang = ang;
            best_idx = i;
        }
    }
    return best_idx;
}

// ========== Tracker ==========

Tracker::Tracker() = default;

void Tracker::reset()
{
    RCLCPP_WARN(rclcpp::get_logger("TRACKER"), "reset tracker");
    Eigen::Vector<double, 9> zero_state = Eigen::Vector<double, 9>::Zero();
    Eigen::Matrix<double, 9, 9> identity_P = Eigen::Matrix<double, 9, 9>::Identity();
    ekf_.initialize(zero_state, identity_P);

    initialized_ = false;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_armor_pose_yaw_world_ = 0.0f;
    last_armor_number_ = 0;
    selected_armor_id_ = -1;
    center_r_ = 0.0f;
    measured_armor_ = TrackerArmor();
    filter_armor_ = TrackerArmor();
}

void Tracker::init(const TrackerArmor & armor, double current_time)
{
    float armor_pose_yaw_world = armor.ypr_world_.x();
    const Eigen::Vector3d & xyz_world = armor.xyz_world_;

    const double r_init = 0.26;
    double x_c0 = xyz_world.x() + r_init * std::cos(armor_pose_yaw_world);
    double y_c0 = xyz_world.y() + r_init * std::sin(armor_pose_yaw_world);

    Eigen::Vector<double, 9> init_state;
    init_state << x_c0, y_c0, xyz_world.z(), 0.0, 0.0, 0.0, r_init, armor_pose_yaw_world, 0.0;

    Eigen::Matrix<double, 9, 9> init_P = Eigen::Matrix<double, 9, 9>::Identity();
    init_P.diagonal() << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 0.01, 0.1, 1.0;

    ekf_.initialize(init_state, init_P);

    updateMeasurement(armor, current_time);
    updateFilteredValue(armor);

    initialized_ = true;
}

void Tracker::selectBestMatch(const std::vector<TrackerArmor> & armors, TrackerArmor & target)
{
    size_t num = armors.size();
    if (num == 1) {
        target = armors[0];
        return;
    }

    if (!initialized_) {
        size_t min_idx = 0;
        float min_dist = armors[0].image_distance_to_center;
        for (size_t i = 1; i < num; i++) {
            if (armors[i].image_distance_to_center < min_dist) {
                min_dist = armors[i].image_distance_to_center;
                min_idx = i;
            }
        }
        target = armors[min_idx];
        return;
    }

    // 用 EKF 预测装甲板世界位置
    Eigen::Vector<double, 9> state = ekf_.getStatePost();
    double r = state[6];
    double yaw = state[7];
    Eigen::Vector3d xyz_pred_world(
        state[0] - r * std::cos(yaw),
        state[1] - r * std::sin(yaw),
        state[2]);

    float min_dist = std::numeric_limits<float>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < num; ++i) {
        float dist = (armors[i].xyz_world_ - xyz_pred_world).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = i;
        }
    }
    target = armors[best_idx];
}

bool Tracker::checkYawMutation(float armor_pose_yaw)
{
    if (!initialized_) return false;
    float dy = armor_pose_yaw - last_armor_pose_yaw_world_;
    dy = normalizeRadAngle(dy);
    return std::abs(dy) > yaw_mutation_threshold_;
}

bool Tracker::isLostTooLong(double current_time) const
{
    return (current_time - last_detection_time_) > max_lost_time_;
}

double Tracker::calculateDt(double current_time)
{
    double dt = 0.01;
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        std::min(dt, 1.0);
    }
    last_update_time_ = current_time;
    return dt;
}

void Tracker::Update(const std::vector<ArmorPlate> & armor_plates,
                     double current_time,
                     float yaw_abs, float pitch_abs)
{
    auto t_start = std::chrono::steady_clock::now();
    solve_ok_ = false;

    double dt = calculateDt(current_time);
    transformer_.update(yaw_abs, pitch_abs);
    ekf_.updateStateTransitionMatrix(dt);

    // 没有目标
    if (armor_plates.empty()) {
        is_lost_ = true;
        if (isLostTooLong(current_time))
            reset();
        else if (initialized_)
            ekf_.predict();
        return;
    }

    // ArmorPlate → TrackerArmor（一进来就转换，后续不再用 ArmorPlate）
    std::vector<TrackerArmor> armors;
    armors.reserve(armor_plates.size());
    for (const auto & plate : armor_plates) {
        TrackerArmor a(
            Eigen::Vector3d(plate.pose.position.x, plate.pose.position.y, plate.pose.position.z),
            Eigen::Quaterniond(plate.pose.orientation.w, plate.pose.orientation.x,
                               plate.pose.orientation.y, plate.pose.orientation.z));
        a.id = plate.number;
        a.image_distance_to_center = plate.image_distance_to_center;
        transformer_.updateTrackerArmor(a);
        armors.push_back(a);
    }

    // 选择最佳匹配
    TrackerArmor target;
    selectBestMatch(armors, target);

    if (target.ypr_world_.y() < MIN_VALID_ARMOR_PITCH_WORLD) {
        RCLCPP_WARN(rclcpp::get_logger("TRACKER"),
            "reject mirrored PnP: ypr in camera: (%.4f, %.4f, %.4f), ypr in world: (%.4f, %.4f, %.4f)",
            target.ypr_camera_.x(), target.ypr_camera_.y(), target.ypr_camera_.z(),
            target.ypr_world_.x(), target.ypr_world_.y(), target.ypr_world_.z()
        );
        is_lost_ = true;
        if (initialized_) ekf_.predict();
        return;
    }

    float armor_pose_yaw_world = target.ypr_world_.x();
    
    // if (checkYawMutation(armor_pose_yaw_world)) reset();

    if (!initialized_) {
        init(target, current_time);
        return;
    }

    // ===== TEST =====
    // bool yaw_jump = checkYawMutation(armor_pose_yaw_world);
    // if (yaw_jump) {
    //     RCLCPP_INFO(rclcpp::get_logger("TRACKER_ID"), "突变");
    // }

    auto armor_list = getTrackerArmorList();
    size_t far_idx = 0;
    size_t best_idx = pickArmorIdx(armor_list, target, armor_pose_yaw_world, far_idx);
    selected_armor_id_ = static_cast<int>(best_idx);

    // EKF Predict
    ekf_.predict();

    // EKF Correct
    Eigen::Vector<double, 4> measurement;
    measurement << target.xyz_world_.x(), target.xyz_world_.y(),
                   target.xyz_world_.z(), armor_pose_yaw_world;

    Eigen::Vector<double, 4> filtered_obs = ekf_.correct(measurement, selected_armor_id_);
    Eigen::Vector<double, 9> state = ekf_.getStatePost();
    solve_ok_ = true;

    // 滤波结果
    Eigen::Vector3d xyz_world_filtered = filtered_obs.head<3>();
    TrackerArmor filtered(xyz_world_filtered, filtered_obs[3]);
    transformer_.updateTrackerArmor(filtered);
    updateMeasurement(target, current_time);
    updateFilteredValue(filtered);

    // 提取 EKF 状态
    center_point_world_ = Eigen::Vector3d(state[0], state[1], state[2]);
    center_velocity_ = Eigen::Vector3d(state[3], state[4], 0);
    center_r_ = static_cast<float>(state[6]);

    auto t_end = std::chrono::steady_clock::now();
    time_cost_ = std::chrono::duration<float, std::milli>(t_end - t_start).count();
}

void Tracker::updateMeasurement(const TrackerArmor & armor, double current_time)
{
    measured_armor_ = armor;
    last_armor_pose_yaw_world_ = armor.ypr_world_.x();
    last_armor_number_ = armor.id;
    last_detection_time_ = current_time;
    is_lost_ = false;
}

void Tracker::updateFilteredValue(const TrackerArmor & armor)
{
    filter_armor_ = armor;
}

TrackerDebug Tracker::CreatedebugMsg(const builtin_interfaces::msg::Time & stamp) const
{
    TrackerDebug msg;
    msg.header.stamp = stamp;

    auto toVec3 = [](const Eigen::Vector3d & v) {
        geometry_msgs::msg::Vector3 vec;
        vec.x = v.x();
        vec.y = v.y();
        vec.z = v.z();
        return vec;
    };

    if (!is_lost_) {
        msg.target_point = toVec3(measured_armor_.xyz_camera_);
        msg.filtered_point = toVec3(filter_armor_.xyz_camera_);
        msg.target_point_world = toVec3(measured_armor_.xyz_world_);
        msg.filtered_point_world = toVec3(filter_armor_.xyz_world_);
    } else {
        geometry_msgs::msg::Vector3 center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 1.0;
        msg.target_point = center;
        msg.filtered_point = center;
        msg.target_point_world = center;
        msg.filtered_point_world = center;
    }

    msg.raw_yaw = last_armor_pose_yaw_world_;
    msg.filter_yaw = filter_armor_.ypr_world_.x();

    msg.center_x = static_cast<float>(center_point_world_.x());
    msg.center_y = static_cast<float>(center_point_world_.y());
    msg.center_r = center_r_;
    msg.center_v_x = static_cast<float>(center_velocity_.x());
    msg.center_v_y = static_cast<float>(center_velocity_.y());

    msg.time_cost = time_cost_;
    msg.method = "ekf";
    msg.solve_ok = solve_ok_;

    return msg;
}

const std::vector<Eigen::Vector<double, 4>> Tracker::getTrackerArmorList()
{
    // xyz, angle
    std::vector<Eigen::Vector<double, 4>> armor_list;
    armor_list.resize(4);
    Eigen::Vector<double, 9> state = ekf_.getStatePost();
    double r = state[6];
    double yaw = state[7];
    
    for (int i = 0; i < 4; ++i) {
        double angle = yaw + i * M_PI / 2;
        Eigen::Vector<double, 4> armor{
            state[0] - r * std::cos(angle),
            state[1] - r * std::sin(angle),
            state[2],
            normalizeRadAngle(angle)
        };
        armor_list[i] = armor;
    };
    return armor_list;
}
