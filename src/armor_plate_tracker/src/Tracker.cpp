#include "armor_plate_tracker/Tracker.hpp"

#include <algorithm>
#include <map>
#include <rclcpp/logger.hpp>
#include <vector>
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "rclcpp/logging.hpp"

using armor_plate_interfaces::msg::TrackerDebug;

// ========== Tracker ==========

Tracker::Tracker() = default;

void Tracker::Update(const std::vector<ArmorPlate> &armor_plates, double current_time, const GimbalData &gimbal)
{
    double dt = calculateDt(current_time);
    transformer_.update(gimbal);
    auto armors = ArmorPlateToTrackerArmor(armor_plates);
    auto grouped = groupByArmorName(armors);

    if (state_ != TrackerState::LOST) {
        target_.predict(dt);
    }

    bool is_found = false;

    if (state_ == TrackerState::LOST) {
        if (!grouped.empty()) {
            // TODO: 未来添加 ArmorName 级别的优先级策略
            // 当前：在所有装甲板中选最近图像中心的初始化
            TrackerArmor init_armor = selectRepresentative(armors);

            target_.init(init_armor);
            updateMeasurement(init_armor, current_time);
            is_found = true;
        }
    } else {
        // 非 LOST 状态：只认 last_armor_name_
        auto it = grouped.find(last_armor_name_);
        if (it != grouped.end()) {
            const auto &matched = it->second;
            target_.update(matched);  // 一群装甲板串行 correct
            TrackerArmor rep = selectRepresentative(matched);
            updateMeasurement(rep, current_time);
            is_found = true;
        }
    }

    if (state_ != TrackerState::LOST && !target_.checkEKFHealth()) {
        reset();
        is_found = false;
    }

    if (state_ != TrackerState::LOST) {
        extractFilteredResult();
    }

    updateState(is_found, current_time);  // 隐藏式状态更新，仅此一处修改 state_
}

void Tracker::updateState(bool is_found, double current_time)
{
    if (is_found) {
        last_detection_time_ = current_time;
    }

    switch (state_) {
        case TrackerState::LOST:
            if (is_found) {
                state_ = TrackerState::DETECTING;
                detect_count_ = 1;
            }
            break;

        case TrackerState::DETECTING:
            if (is_found) {
                if (++detect_count_ >= 5) {
                    state_ = TrackerState::TRACKING;
                    detect_count_ = 0;
                }
            } else {
                state_ = TrackerState::LOST;
                detect_count_ = 0;
            }
            break;

        case TrackerState::TRACKING:
            if (!is_found) {
                state_ = TrackerState::TEMP_LOST;
            }
            break;

        case TrackerState::TEMP_LOST:
            if (is_found) {
                state_ = TrackerState::TRACKING;
            } else if (isLostTooLong(current_time)) {
                state_ = TrackerState::LOST;
                detect_count_ = 0;
            }
            break;
    }
}

void Tracker::reset()
{
    RCLCPP_WARN(rclcpp::get_logger("TRACKER"), "reset tracker");
    target_.reset();

    state_ = TrackerState::LOST;
    detect_count_ = 0;
    last_detection_time_ = 0.0;
    last_armor_name_ = ArmorName::NONE;
    selected_armor_id_ = -1;
    center_r_ = 0.0f;
    measured_armor_ = TrackerArmor();
    filter_armor_ = TrackerArmor();
}

void Tracker::init(const TrackerArmor &armor, double current_time)
{
    target_.init(armor);
    updateMeasurement(armor, current_time);
    updateFilteredValue(armor);
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

void Tracker::updateMeasurement(const TrackerArmor &armor, double current_time)
{
    measured_armor_ = armor;
    last_armor_name_ = armor.armor_name;
    last_detection_time_ = current_time;
}

void Tracker::updateFilteredValue(const TrackerArmor &armor)
{
    filter_armor_ = armor;
}

void Tracker::extractFilteredResult()
{
    center_point_world_ = target_.getCenterPointWorld();
    center_velocity_ = target_.getCenterVelocity();
    center_r_ = static_cast<float>(target_.getRadius());

    if (!target_.checkEKFHealth()) {
        RCLCPP_WARN(rclcpp::get_logger("TRACKER"),
                    "EKF 异常: Converged = %d; Divergent = %d",
                    target_.isConverged(),
                    target_.isDivergent());
    }

    // 用 EKF 预测状态重建选中的装甲板滤波结果
    selected_armor_id_ = static_cast<int>(target_.getSelectedArmorId());
    Eigen::Vector<double, 4> filtered_obs =
        target_.getArmorObservation(static_cast<size_t>(selected_armor_id_));
    TrackerArmor filtered(filtered_obs);
    transformer_.updateTrackerArmor(filtered);
    updateFilteredValue(filtered);
}

TrackerDebug Tracker::CreatedebugMsg(const builtin_interfaces::msg::Time &stamp) const
{
    TrackerDebug msg;
    msg.header.stamp = stamp;

    auto toVec3 = [](const Eigen::Vector3d &v) {
        geometry_msgs::msg::Vector3 vec;
        vec.x = v.x();
        vec.y = v.y();
        vec.z = v.z();
        return vec;
    };

    msg.is_lost = isLost();
    msg.target_point_world = toVec3(measured_armor_.xyz_world_);

    if (!isLost()) {
        msg.filtered_point_world = toVec3(filter_armor_.xyz_world_);
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
    if (!isLost()) {
        auto armor_list = target_.getTargetArmorList();
        msg.predicted_armor_points_world.reserve(armor_list.size());
        msg.predicted_armor_yaws_world.reserve(armor_list.size());
        for (const auto &armor : armor_list) {
            msg.predicted_armor_points_world.push_back(toVec3(Eigen::Vector3d(armor[0], armor[1], armor[2])));
            msg.predicted_armor_yaws_world.push_back(static_cast<float>(armor[3]));
        }
    }

    msg.raw_yaw = measured_armor_.ypr_world_.x();
    msg.filter_yaw = filter_armor_.ypr_world_.x();

    msg.center_x = static_cast<float>(center_point_world_.x());
    msg.center_y = static_cast<float>(center_point_world_.y());
    msg.center_z = static_cast<float>(center_point_world_.z());
    msg.center_r = center_r_;
    msg.center_v_x = static_cast<float>(center_velocity_.x());
    msg.center_v_y = static_cast<float>(center_velocity_.y());
    msg.center_v_z = static_cast<float>(center_velocity_.z());
    msg.center_l = static_cast<float>(target_.getL());
    msg.center_h = static_cast<float>(target_.getH());

    return msg;
}

std::map<ArmorName, std::vector<TrackerArmor>> Tracker::groupByArmorName(
    const std::vector<TrackerArmor> &armors)
{
    std::map<ArmorName, std::vector<TrackerArmor>> grouped;
    for (const auto &armor : armors) {
        grouped[armor.armor_name].push_back(armor);
    }
    return grouped;
}

TrackerArmor Tracker::selectRepresentative(const std::vector<TrackerArmor> &armors)
{
    /*
        TODO: 未来添加 ArmorName 级别的优先级策略
        当前：在所有装甲板中选最近图像中心的初始化
    */
    if (armors.empty()) {
        return TrackerArmor();
    }
    auto it = std::min_element(armors.begin(), armors.end(),
                               [](const TrackerArmor &a, const TrackerArmor &b) {
                                   return a.image_distance_to_center < b.image_distance_to_center;
                               });
    return *it;
}

TrackerArmor Tracker::ArmorPlateToTrackerArmor(const ArmorPlate &armor_plate)
{
    TrackerArmor armor(
        Eigen::Vector3d(armor_plate.pose.position.x, armor_plate.pose.position.y, armor_plate.pose.position.z),
        Eigen::Quaterniond(armor_plate.pose.orientation.w,
                           armor_plate.pose.orientation.x,
                           armor_plate.pose.orientation.y,
                           armor_plate.pose.orientation.z));
    armor.armor_name = intToArmorName(armor_plate.number);
    armor.image_distance_to_center = armor_plate.image_distance_to_center;
    transformer_.updateTrackerArmor(armor);
    return armor;
}

std::vector<TrackerArmor> Tracker::ArmorPlateToTrackerArmor(const std::vector<ArmorPlate> &armor_plates)
{
    std::vector<TrackerArmor> armors;
    armors.reserve(armor_plates.size());
    for (const auto &plate : armor_plates) {
        TrackerArmor armor = ArmorPlateToTrackerArmor(plate);
        armors.push_back(armor);
    }
    return armors;
}
