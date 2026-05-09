#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using armor_plate_interfaces::msg::TrackerDebug;

// opencv坐标系 → 云台坐标系 (x向前, y向左, z向上)
const Eigen::Matrix3d R_w_cv = (Eigen::Matrix3d() <<
    0.0 ,  0.0,  1.0,
    -1.0,  0.0,  0.0,
    0.0 , -1.0,  0.0).finished();

// 默认构造函数
Tracker::Tracker()
    : yaw_(0.0f), pitch_(0.0f)
    , last_update_time_(0.0)
    , last_detection_time_(0.0)
    , initialized_(false)
    , max_lost_time_(0.1)
    , is_lost_(true)
    , yaw_mutation_threshold_(0.5f)
    , last_armor_number_("")
{
}
void Tracker::updateMeasurement(const ArmorPlate& armor_plate, double current_time)
{
    Eigen::Vector3d pc(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    Eigen::Vector3d pw = R_w_c_ * pc;

    measured_position_camera_ = pc;
    measured_position_world_  = pw;
    measured_yaw_   = calculateYaw(pc);
    measured_pitch_ = calculatePitch(pc);

    Eigen::Quaterniond qc(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    Eigen::Quaterniond qw = q_w_c_ * qc;
    float armor_yaw_world = calculatePoseYaw(qw);
    measured_orientation_world_ = getQuaternionFromYaw(armor_yaw_world);

    last_armor_pose_yaw_world_ = armor_yaw_world;
    last_armor_number_ = armor_plate.number;
    last_detection_time_ = current_time;
    is_lost_ = false;
}

void Tracker::updateFilteredValue(const Eigen::Vector3d& pc_f, const Eigen::Vector3d& pw_f)
{
    filter_position_camera_ = pc_f;
    filter_position_world_  = pw_f;
    yaw_   = calculateYaw(pc_f);
    pitch_ = calculatePitch(pc_f);
}
void Tracker::reset()
{
    // 重置 Yaw 前置滤波器
    Eigen::Vector2d yaw_zero = Eigen::Vector2d::Zero();
    Eigen::Matrix2d yaw_P = Eigen::Matrix2d::Identity();
    yaw_kf_.initialize(yaw_zero, yaw_P);

    // 重置 Center 中心滤波器
    Eigen::Vector4d center_zero = Eigen::Vector4d::Zero();
    Eigen::Matrix4d center_P = Eigen::Matrix4d::Identity();
    center_kf_.initialize(center_zero, center_P);

    // 清空历史数据
    center_filter_.reset();

    initialized_ = false;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_armor_pose_yaw_world_ = 0.0f;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
    measured_yaw_ = 0.0f;
    measured_pitch_ = 0.0f;
    last_armor_number_ = "";
    center_r_ = 0.0f;
}
void Tracker::init(const ArmorPlate& armor_plate, double current_time)
{
    Eigen::Vector3d target_position(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    Eigen::Quaterniond target_plate_q(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    Eigen::Quaterniond qw_m = q_w_c_ * target_plate_q;
    float armor_pose_yaw_world = calculatePoseYaw(qw_m);

    Eigen::Vector3d pw = R_w_c_ * target_position;
    const double r_init = 0.26;
    double x_c0 = pw.x() - r_init * std::sin(armor_pose_yaw_world);
    double y_c0 = pw.y() - r_init * std::cos(armor_pose_yaw_world);

    // 初始化 Yaw 前置滤波器
    Eigen::Vector2d yaw_init_state(armor_pose_yaw_world, 0.0);
    Eigen::Matrix2d yaw_init_P = Eigen::Matrix2d::Identity();
    yaw_init_P.diagonal() << 0.1, 1.0;
    yaw_kf_.initialize(yaw_init_state, yaw_init_P);

    // 初始化 Center 中心滤波器
    Eigen::Vector4d center_init_state(x_c0, y_c0, 0.0, 0.0);
    Eigen::Matrix4d center_init_P = Eigen::Matrix4d::Identity();
    center_init_P.diagonal() << 0.1, 0.1, 1.0, 1.0;
    center_kf_.initialize(center_init_state, center_init_P);

    // 首次初始化：测量值 = 滤波值 = 初始观测
    updateMeasurement(armor_plate, current_time);
    updateFilteredValue(target_position, pw);

    initialized_ = true;
}

// 选择最佳匹配目标
void Tracker::selectBestMatch(const std::vector<ArmorPlate>& armor_plates, ArmorPlate& target_armor) 
{
    size_t num_plates = armor_plates.size();
    // 如果只有一个目标，直接选择
    if (num_plates == 1) {
        target_armor = armor_plates[0];
        return;
    }
    // 多个目标
    if (!initialized_) {
        // 未初始化时，选择 image_distance_to_center 最小的目标
        size_t min_idx = 0;
        float min_dist = armor_plates[0].image_distance_to_center;
        for (size_t i = 1; i < num_plates; i++) {
            float image_distance  = armor_plates[i].image_distance_to_center;
            if (image_distance < min_dist) {
                min_dist = image_distance;
                min_idx = i;
            }
        }
        // 保存结果
        target_armor = armor_plates[min_idx];
        return;
    } 
    // 选择与当前预测最接近的

    // 使用旋转中心 + yaw 预测装甲板位置
    Eigen::Vector4d center_state = center_kf_.getStatePost();
    double yaw_pred = yaw_kf_.getStatePost()[0];
    double r = center_filter_.getR();
    Eigen::Vector3d pred_pos(
        center_state[0] + r * std::sin(yaw_pred),
        center_state[1] - r * std::cos(yaw_pred),
        measured_position_world_.z()
    );
    float min_dist = std::numeric_limits<float>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < num_plates; ++i) {
        Eigen::Vector3d p_c;
        p_c[0] = armor_plates[i].pose.position.x;
        p_c[1] = armor_plates[i].pose.position.y;
        p_c[2] = armor_plates[i].pose.position.z;
        Eigen::Vector3d p_w = R_w_c_ * p_c;
        Eigen::Vector3d diff = p_w - pred_pos;
        float dist_sq = diff.norm();
        if (dist_sq < min_dist) {
            min_dist = dist_sq;
            best_idx = i;
        }
    }
    // 保存结果
    target_armor = armor_plates[best_idx];
}

// 检查Yaw是否突变
bool Tracker::isYawMutation(const float& armor_pose_yaw)
{
    if (!initialized_) {
        return false;
    }
    float dy = std::abs(armor_pose_yaw - last_armor_pose_yaw_world_);
    dy = normalizeRadAngle(dy);
    return (dy > yaw_mutation_threshold_);
}

// 检查是否丢失太久
bool Tracker::isLostTooLong(double current_time) const
{
    double lost_duration = current_time - last_detection_time_;
    return lost_duration > max_lost_time_;
}

// 主更新函数
void Tracker::Update(const std::vector<ArmorPlate>& armor_plates,
                     double current_time,
                     float yaw_abs, float pitch_abs)
{
    auto t_start = std::chrono::steady_clock::now();
    solve_ok_ = false;
    // 计算dt（与上次更新的时间差）
    double dt = 0.01;  // 默认10ms
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        if (dt <= 0.0) dt = 0.001;  // 最小1ms
        if (dt > 1.0) dt = 1.0;     // 最大1s
    }
    last_update_time_ = current_time;
    
    // 坐标系转换计算
    double psi = yaw_abs;
    double theta = pitch_abs;
    // Rw<-c = Rx(pitch_abs)Ry(-yaw_abs) 解耦矩阵
    Eigen::Matrix3d R_cv_c;
    R_cv_c << cos(psi),            0,            -sin(psi),
            -sin(theta)*sin(psi), cos(theta),   -sin(theta)*cos(psi),
            cos(theta)*sin(psi),  sin(theta),   cos(theta)*cos(psi);
    R_w_c_ = R_w_cv * R_cv_c;
    R_c_w_ = R_w_c_.transpose();
    q_w_c_ = Eigen::Quaterniond(R_w_c_);
    // 更新状态转移矩阵中的dt
    
    // 没有目标情况
    if (armor_plates.size() == 0) {
        // 没有检测到目标，进入丢失状态
        is_lost_ = true;
        if (isLostTooLong(current_time)) reset(); // 丢失时间过长，重置
        return;
    }

    // 未初始化：选 image_distance_to_center 最小的板，设 number
    if (!initialized_) {
        ArmorPlate best_plate = armor_plates[0];
        for (size_t i = 1; i < armor_plates.size(); i++) {
            if (armor_plates[i].image_distance_to_center < best_plate.image_distance_to_center) {
                best_plate = armor_plates[i];
            }
        }
        last_armor_number_ = best_plate.number;
        init(best_plate, current_time);
        return;
    }

    // 按 number 筛选同车板
    std::vector<ArmorPlate> same_car;
    for (const auto& plate : armor_plates) {
        if (plate.number == last_armor_number_) {
            same_car.push_back(plate);
        }
    }

    // 没有同 number 的板 → number 变了 → 突变，重新选车
    if (same_car.empty()) {
        center_filter_.reset();
        reset();
        // 选 image_distance_to_center 最小的板作为新车
        ArmorPlate best_plate = armor_plates[0];
        for (size_t i = 1; i < armor_plates.size(); i++) {
            if (armor_plates[i].image_distance_to_center < best_plate.image_distance_to_center) {
                best_plate = armor_plates[i];
            }
        }
        last_armor_number_ = best_plate.number;
        init(best_plate, current_time);
        return;
    }

    // 从同车板中选最佳目标（用于 YawKF + updateMeasurement）
    ArmorPlate target_plate;
    selectBestMatch(same_car, target_plate);

    // 提取选中的目标位置（相机系）
    Eigen::Vector3d target_position(
        target_plate.pose.position.x,
        target_plate.pose.position.y,
        target_plate.pose.position.z
    );

    // 提取装甲板姿态四元数（相机系）
    Eigen::Quaterniond target_plate_q(
        target_plate.pose.orientation.w,
        target_plate.pose.orientation.x,
        target_plate.pose.orientation.y,
        target_plate.pose.orientation.z
    );
    // 提前计算世界坐标系下的装甲板姿态四元数
    Eigen::Quaterniond qw_m = q_w_c_ * target_plate_q;

    // 计算装甲板姿态 yaw（世界坐标系）
    float armor_pose_yaw_world = calculatePoseYaw(qw_m);
    // Yaw 前置滤波
    yaw_kf_.updateStateTransitionMatrix(dt);
    yaw_kf_.predict();
    double yaw_filtered = yaw_kf_.correct(static_cast<double>(armor_pose_yaw_world));

    // 判断突变，清空历史队列（旧方法：靠 yaw 跳变判断是否同一辆车，已被 number 筛选替代）
    // if (isYawMutation(yaw_filtered)) {
    //     center_filter_.reset();
    //     reset();
    // }

    // 保存当前帧选中的原始测量值
    updateMeasurement(target_plate, current_time);

    // 同车所有板都转世界坐标，存入 CenterFilter
    std::vector<ArmorPlateStamped> frame_plates;
    for (const auto& plate : same_car) {
        Eigen::Vector3d p_c(plate.pose.position.x, plate.pose.position.y, plate.pose.position.z);
        Eigen::Vector3d p_w = R_w_c_ * p_c;
        Eigen::Quaterniond q_c(plate.pose.orientation.w, plate.pose.orientation.x,
                               plate.pose.orientation.y, plate.pose.orientation.z);
        Eigen::Quaterniond q_w = q_w_c_ * q_c;
        double yaw = static_cast<double>(calculatePoseYaw(q_w));

        ArmorPlateStamped record;
        record.timestamp = current_time;
        record.position_world = p_w;
        record.yaw_world = yaw;
        frame_plates.push_back(record);
    }
    center_filter_.addFrame(frame_plates, current_time);

    // 最小二乘法求解旋转中心
    bool ls_ok = center_filter_.solve();

    // CenterKF 预测
    center_kf_.updateStateTransitionMatrix(dt);
    center_kf_.predict();

    // 世界坐标系下的目标位置（用于 backCalculateCenter fallback）
    Eigen::Vector3d pw_m = R_w_c_ * target_position;

    double r_used = 0.26; // 默认 r
    Eigen::Vector2d center_measurement;
    if (ls_ok) {
        center_measurement = center_filter_.getCenter();
        r_used = center_filter_.getR();
        // 用 LS 估计的速度
        Eigen::Vector2d vel = center_filter_.getVelocity();
        center_velocity_ = Eigen::Vector3d(vel[0], vel[1], 0.0);
    } else {
        // 数据不足，用上一帧的 r 反算旋转中心
        center_measurement = CenterFilter::backCalculateCenter(pw_m, yaw_filtered, center_filter_.getR());
    }

    // CenterKF 校正
    Eigen::Vector2d center_filtered = center_kf_.correct(center_measurement);
    Eigen::Vector4d center_state = center_kf_.getStatePost();
    center_point_world_ = Eigen::Vector3d(center_filtered[0], center_filtered[1], pw_m.z());
    if (!ls_ok) {
        // LS 失败时用 CenterKF 的速度
        center_velocity_ = Eigen::Vector3d(center_state[2], center_state[3], 0.0);
    }
    center_r_ = static_cast<float>(r_used);
    solve_ok_ = true;

    filter_orientation_world_ = getQuaternionFromYaw(yaw_filtered);

    // 用旋转中心 + yaw + r 计算滤波后的装甲板位置
    Eigen::Vector3d filter_pos_world(
        center_point_world_.x() + r_used * std::sin(yaw_filtered),
        center_point_world_.y() - r_used * std::cos(yaw_filtered),
        center_point_world_.z()
    );
    Eigen::Vector3d filter_pos_camera = R_c_w_ * filter_pos_world;
    updateFilteredValue(filter_pos_camera, filter_pos_world);

    auto t_end = std::chrono::steady_clock::now();
    time_cost_ = std::chrono::duration<float, std::milli>(t_end - t_start).count();
}
////////// 工具类 /////////
float calculatePoseYaw(const Eigen::Quaterniond &q)
{
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
} 
float calculateYaw(const Eigen::Vector3d& tvec)
{
    return -static_cast<float>(std::atan2(tvec.x(), tvec.z()));
}
float calculatePitch(const Eigen::Vector3d& tvec)
{
    double horizontal_dist = std::sqrt(tvec.x() * tvec.x() + tvec.z() * tvec.z());
    return static_cast<float>(std::atan2(-tvec.y(), horizontal_dist));
}
float calculateDistance(const Eigen::Vector3d& tvec)
{
    return static_cast<float>(tvec.norm());
}
float normalizeRadAngle(float rad)
{
    while(rad > M_PI) rad -= 2.0 * M_PI;
    while(rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}
Eigen::Quaterniond getQuaternionFromYaw(float yaw)
{
    // 有关世界Z轴的旋转
    Eigen::AngleAxisd yawAngle(yaw + M_PI / 2, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(yawAngle);
}

TrackerDebug Tracker::CreatedebugMsg(const builtin_interfaces::msg::Time& stamp) const
{
    TrackerDebug msg;
    msg.header.stamp = stamp;

    auto toVec3 = [](const Eigen::Vector3d& v) {
        geometry_msgs::msg::Vector3 vec;
        vec.x = v.x(); vec.y = v.y(); vec.z = v.z();
        return vec;
    };

    if (!is_lost_) {
        msg.target_point = toVec3(measured_position_camera_);
        msg.filtered_point = toVec3(filter_position_camera_);
        msg.target_point_world = toVec3(measured_position_world_);
        msg.filtered_point_world = toVec3(filter_position_world_);
    } else {
        geometry_msgs::msg::Vector3 center;
        center.x = 0.0; center.y = 0.0; center.z = 1.0;
        msg.target_point = center;
        msg.filtered_point = center;
        msg.target_point_world = center;
        msg.filtered_point_world = center;
    }

    msg.raw_yaw = last_armor_pose_yaw_world_;
    msg.filter_yaw = calculatePoseYaw(filter_orientation_world_);

    msg.center_x = static_cast<float>(center_point_world_.x());
    msg.center_y = static_cast<float>(center_point_world_.y());
    msg.center_r = center_r_;
    msg.center_v_x = static_cast<float>(center_velocity_.x());
    msg.center_v_y = static_cast<float>(center_velocity_.y());

    msg.time_cost = time_cost_;
    msg.method = "ls";
    msg.solve_ok = solve_ok_;

    return msg;
}
