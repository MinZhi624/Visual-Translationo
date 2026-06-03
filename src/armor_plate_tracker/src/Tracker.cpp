#include "armor_plate_tracker/Tracker.hpp"

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

    // 没有目标
    if (armor_plates.empty()) {
        updateState(false, current_time);
        if (state_ == TrackerState::LOST)
            reset();
        else if (traget_.isInitialized())
            traget_.predict(dt);
        return;
    }

    // ArmorPlate → TrackerArmor（一进来就转换，后续不再用 ArmorPlate）
    auto armors = ArmorPlateToTrackerArmor(armor_plates);

    // 只要和目标相同数字的车
    // 未初始化时不过滤 id，使用全部装甲板来选择初始化目标
    std::vector<TrackerArmor> selected_armors;
    if (!traget_.isInitialized()) {
        selected_armors = armors;
    }
    else {
        for (const auto &armor : armors) {
            if (armor.id == last_armor_number_) {
                selected_armors.push_back(armor);
            }
        }
    }

    // 没有同 id 的装甲板，按丢失处理
    if (selected_armors.empty()) {
        updateState(false, current_time);
        if (state_ == TrackerState::LOST)
            reset();
        else if (traget_.isInitialized())
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
        updateState(true, current_time);
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

    // TODO： 升级火控系统
    // 目前：寻找目标 -- 以离图像中心最近的装甲板为基准
    TrackerArmor target_armor = selected_armors[0];
    for (size_t i = 1; i < selected_armors.size(); ++i) {
        if (selected_armors[i].image_distance_to_center < target_armor.image_distance_to_center) {
            target_armor = selected_armors[i];
        }
    }
    if (traget_.isConverged() || traget_.isDivergent())
        RCLCPP_WARN(rclcpp::get_logger("TRACKER"),
                    "Converged = %d; Divergent = %d",
                    traget_.isConverged(),
                    traget_.isDivergent());

    // 滤波结果
    Eigen::Vector<double, 4> filtered_obs = traget_.getFilteredObservation();
    TrackerArmor filtered(filtered_obs);
    transformer_.updateTrackerArmor(filtered);

    updateMeasurement(target_armor, current_time);
    updateFilteredValue(filtered);
    updateState(true, current_time);
}

void Tracker::updateState(const bool &is_found, double current_time) {
    // 时间维护 + 更新状态
    if (traget_.isConverged() || traget_.isDivergent()) {
        RCLCPP_WARN(rclcpp::get_logger("TRACKER"), "EKF状态异常,is_converged = %d, is_divergent = %d",
                   traget_.isConverged(),traget_.isDivergent());
        state_ = TrackerState::LOST;
        detect_count_ = 0; 
        return;
    } 
    if (!is_found) {
        switch (state_) {
            case TrackerState::DETECTING:
                state_ = TrackerState::LOST;
                detect_count_ = 0; 
                break;
            case TrackerState::TRACKING:
                state_ = TrackerState::TEMP_LOST;
                break;
            case TrackerState::TEMP_LOST:
                if (isLostTooLong(current_time)) {
                    state_ = TrackerState::LOST;
                    detect_count_ = 0; 
                }
                break;
            default:
                break;
        }
    }
    else {
        last_detection_time_ = current_time;
        switch (state_) {
            case TrackerState::LOST:
                state_ = TrackerState::DETECTING;
                detect_count_++;
                break;
            case TrackerState::TEMP_LOST:
                state_ = TrackerState::TRACKING;
                break;
            case TrackerState::DETECTING:
                detect_count_++;
                if (detect_count_ >= 5) {
                    state_ = TrackerState::TRACKING;
                    detect_count_ = 0;
                }
                break;
            default:
                break;
        }
    }
}

void Tracker::reset() {
    RCLCPP_WARN(rclcpp::get_logger("TRACKER"), "reset tracker");
    traget_.reset();

    state_ = TrackerState::LOST;
    detect_count_ = 0;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_armor_pose_yaw_world_ = 0.0f;
    last_armor_number_ = 0;
    selected_armor_id_ = -1;
    center_r_ = 0.0f;
    measured_armor_ = TrackerArmor();
    filter_armor_ = TrackerArmor();
}

void Tracker::init(const TrackerArmor &armor, double current_time) {
    traget_.init(armor);

    updateMeasurement(armor, current_time);
    updateFilteredValue(armor);
}

bool Tracker::isLostTooLong(double current_time) const {
    return (current_time - last_detection_time_) > max_lost_time_;
}

double Tracker::calculateDt(double current_time) {
    double dt = 0.01;
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        dt = std::min(dt, 1.0);
    }
    last_update_time_ = current_time;
    return dt;
}

void Tracker::updateMeasurement(const TrackerArmor &armor, double current_time) {
    measured_armor_ = armor;
    last_armor_pose_yaw_world_ = armor.ypr_world_.x();
    last_armor_number_ = armor.id;
    last_detection_time_ = current_time;
}

void Tracker::updateFilteredValue(const TrackerArmor &armor) {
    filter_armor_ = armor;
}

TrackerDebug Tracker::CreatedebugMsg(const builtin_interfaces::msg::Time &stamp) const {
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
    }
    else if (traget_.isInitialized()) {
        auto armor_list = getTrackerArmorList();
        int idx = selected_armor_id_ >= 0 ? selected_armor_id_ : 0;
        msg.filtered_point_world = toVec3(Eigen::Vector3d(armor_list[idx][0], armor_list[idx][1], armor_list[idx][2]));
    }
    else {
        geometry_msgs::msg::Vector3 center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 1.0;
        msg.filtered_point_world = center;
    }

    msg.selected_armor_id = selected_armor_id_;
    msg.predicted_armor_points_world.clear();
    msg.predicted_armor_yaws_world.clear();
    if (traget_.isInitialized() && !isLost()) {
        auto armor_list = getTrackerArmorList();
        msg.predicted_armor_points_world.reserve(armor_list.size());
        msg.predicted_armor_yaws_world.reserve(armor_list.size());
        for (const auto &armor : armor_list) {
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

    return msg;
}

const std::vector<Eigen::Vector<double, 4>> Tracker::getTrackerArmorList() const {
    auto arr = traget_.getTrackerArmorList();
    std::vector<Eigen::Vector<double, 4>> armor_list(arr.begin(), arr.end());
    return armor_list;
}

TrackerArmor Tracker::ArmorPlateToTrackerArmor(const ArmorPlate &armor_plate) {
    TrackerArmor armor(
        Eigen::Vector3d(armor_plate.pose.position.x, armor_plate.pose.position.y, armor_plate.pose.position.z),
        Eigen::Quaterniond(armor_plate.pose.orientation.w,
                           armor_plate.pose.orientation.x,
                           armor_plate.pose.orientation.y,
                           armor_plate.pose.orientation.z));
    armor.id = armor_plate.number;
    armor.image_distance_to_center = armor_plate.image_distance_to_center;
    transformer_.updateTrackerArmor(armor);
    return armor;
}
std::vector<TrackerArmor> Tracker::ArmorPlateToTrackerArmor(const std::vector<ArmorPlate> &armor_plates) {
    std::vector<TrackerArmor> armors;
    armors.reserve(armor_plates.size());
    for (const auto &plate : armor_plates) {
        TrackerArmor armor = ArmorPlateToTrackerArmor(plate);
        armors.push_back(armor);
    }
    return armors;
}