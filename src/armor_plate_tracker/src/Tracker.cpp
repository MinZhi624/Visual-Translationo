#include "armor_plate_tracker/Tracker.hpp"

#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "rclcpp/logging.hpp"
#include <chrono>
#include <limits>
#include <rclcpp/logger.hpp>
#include <vector>

using armor_plate_interfaces::msg::TrackerDebug;

static double normalizeRadAngle(double rad)
{
    while (rad > M_PI) rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

// ========== Tracker ==========

Tracker::Tracker() = default;

void Tracker::Update(const std::vector<ArmorPlate> & armor_plates,
                     double current_time,
                     const GimbalData & gimbal)
{
    auto t_start = std::chrono::steady_clock::now();
    solve_ok_ = false;

    double dt = calculateDt(current_time);

    transformer_.update(gimbal);

    // 没有目标
    if (armor_plates.empty()) {
        is_lost_ = true;
        if (isLostTooLong(current_time))
            reset();
        else if (traget_.isInitialized())
            traget_.predict(dt);
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

    // 只要和目标相同数字的车
    // 未初始化时不过滤 id，使用全部装甲板来选择初始化目标
    std::vector<TrackerArmor> selected_armors;
    if (!traget_.isInitialized()) {
        selected_armors = armors;
    } else {
        for (const auto & armor : armors) {
            if (armor.id == last_armor_number_) {
                selected_armors.push_back(armor);
            }
        }
    }

    // 没有同 id 的装甲板，按丢失处理
    if (selected_armors.empty()) {
        is_lost_ = true;
        if (traget_.isInitialized())
            traget_.predict(dt);
        return;
    }

    // 初始化目标
    if (!traget_.isInitialized()) {
        TrackerArmor target = selected_armors[0];
        for (size_t i = 1; i < selected_armors.size(); ++i) {
            if (selected_armors[i].image_distance_to_center < target.image_distance_to_center) {
                target = selected_armors[i];
            }
        }

        traget_.init(target);
        updateMeasurement(target, current_time);
        return;
    }

    // EKF 滤波，更新所有装甲板
    traget_.predict(dt);
    traget_.update(selected_armors);

    // 提取 EKF 状态
    center_point_world_ = traget_.getCenterPointWorld();
    center_velocity_ = traget_.getCenterVelocity();
    center_r_ = static_cast<float>(traget_.getRadius());
    selected_armor_id_ = static_cast<int>(traget_.getSelectedArmorId());
    solve_ok_ = true;

    // TODO： 升级火控系统
    // 寻找目标 -- 以离图像中心最近的装甲板为基准
    TrackerArmor target = selected_armors[0];
    for (size_t i = 1; i < selected_armors.size(); ++i) {
        if (selected_armors[i].image_distance_to_center < target.image_distance_to_center) {
            target = selected_armors[i];
        }
    }

    // 滤波结果
    Eigen::Vector<double, 4> filtered_obs = traget_.getFilteredObservation();
    TrackerArmor filtered(filtered_obs);
    transformer_.updateTrackerArmor(filtered);
    updateMeasurement(target, current_time);
    updateFilteredValue(filtered);

    auto t_end = std::chrono::steady_clock::now();
    time_cost_ = std::chrono::duration<float, std::milli>(t_end - t_start).count();
}

void Tracker::reset()
{
    RCLCPP_WARN(rclcpp::get_logger("TRACKER"), "reset tracker");
    traget_.reset();

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
    traget_.init(armor);

    updateMeasurement(armor, current_time);
    updateFilteredValue(armor);
}

bool Tracker::checkYawMutation(float armor_pose_yaw)
{
    if (!traget_.isInitialized()) return false;
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
        dt = std::min(dt, 1.0);
    }
    last_update_time_ = current_time;
    return dt;
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

    msg.is_lost = is_lost_;
    msg.target_point_world = toVec3(measured_armor_.xyz_world_);

    if (!is_lost_) {
        msg.filtered_point_world = toVec3(filter_armor_.xyz_world_);
    } else if (traget_.isInitialized()) {
        auto armor_list = getTrackerArmorList();
        int idx = selected_armor_id_ >= 0 ? selected_armor_id_ : 0;
        msg.filtered_point_world = toVec3(Eigen::Vector3d(armor_list[idx][0], armor_list[idx][1], armor_list[idx][2]));
    } else {
        geometry_msgs::msg::Vector3 center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 1.0;
        msg.filtered_point_world = center;
    }

    msg.selected_armor_id = selected_armor_id_;
    msg.predicted_armor_points_world.clear();
    msg.predicted_armor_yaws_world.clear();
    if (traget_.isInitialized() && !is_lost_) {
        auto armor_list = getTrackerArmorList();
        msg.predicted_armor_points_world.reserve(armor_list.size());
        msg.predicted_armor_yaws_world.reserve(armor_list.size());
        for (const auto & armor : armor_list) {
            msg.predicted_armor_points_world.push_back(toVec3(Eigen::Vector3d(armor[0], armor[1], armor[2])));
            msg.predicted_armor_yaws_world.push_back(static_cast<float>(armor[3]));
        }
    }

    msg.raw_yaw = last_armor_pose_yaw_world_;
    msg.filter_yaw = filter_armor_.ypr_world_.x();

    msg.center_x = static_cast<float>(center_point_world_.x());
    msg.center_y = static_cast<float>(center_point_world_.y());
    msg.center_z = static_cast<float>(center_point_world_.z());
    msg.center_r = center_r_;
    msg.center_v_x = static_cast<float>(center_velocity_.x());
    msg.center_v_y = static_cast<float>(center_velocity_.y());

    msg.time_cost = time_cost_;
    msg.method = "ekf";
    msg.solve_ok = solve_ok_;

    return msg;
}

const std::vector<Eigen::Vector<double, 4>> Tracker::getTrackerArmorList() const
{
    auto arr = traget_.getTrackerArmorList();
    std::vector<Eigen::Vector<double, 4>> armor_list(arr.begin(), arr.end());
    return armor_list;
}
