#include "armor_plate_tracker/Tracker.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using armor_plate_interfaces::msg::TrackerDebug;

// 相机坐标系 -> 云台坐标系
static const Eigen::Matrix3d R_gimbal_camera = (Eigen::Matrix3d() <<
    0,  0,  1,
   -1,  0,  0,
    0, -1,  0).finished();
// 云台坐标系 -> 相机坐标系
static const Eigen::Matrix3d R_camera_gimbal = R_gimbal_camera.transpose();

Eigen::Matrix3d getR_world_gimbal(float yaw, float pitch)
{
    Eigen::Matrix3d R_yaw;
    R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d R_pitch;
    R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    // 云台坐标系到世界坐标系旋转矩阵
    Eigen::Matrix3d R_gimbal2world = R_yaw * R_pitch;
    return R_gimbal2world;
}

// 默认构造函数
Tracker::Tracker()
    : yaw_(0.0f), pitch_(0.0f)
    , last_update_time_(0.0)
    , last_detection_time_(0.0)
    , initialized_(false)
    , max_lost_time_(0.1)
    , is_lost_(true)
    , yaw_mutation_threshold_(0.05f)
    , last_armor_number_("")
{
}
void Tracker::reset()
{
    // EKF 构造函数已设置 Q/R，只需重置状态和协方差
    Eigen::Vector<double, 9> zero_state = Eigen::Vector<double, 9>::Zero();
    Eigen::Matrix<double, 9, 9> identity_P = Eigen::Matrix<double, 9, 9>::Identity();
    ekf_.initialize(zero_state, identity_P);
    
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
    // 跟新滤波器
    Eigen::Vector3d xyz_camera(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    Eigen::Quaterniond q_camera(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    // camera -> gimbal -> world
    Eigen::Quaterniond q_gimbal_measurement = q_g_c_ * q_camera;
    Eigen::Quaterniond q_world_measurement = q_w_g_ * q_gimbal_measurement;
    float armor_pose_yaw_world = calculatePoseYaw(q_world_measurement);

    Eigen::Vector3d xyz_gimbal = R_gimbal_camera * xyz_camera;
    Eigen::Vector3d xyz_world = R_w_g_ * xyz_gimbal;
    const double r_init = 0.26;
    double x_c0 = xyz_world.x() + r_init * std::cos(armor_pose_yaw_world);
    double y_c0 = xyz_world.y() + r_init * std::sin(armor_pose_yaw_world);
    
    Eigen::Vector<double, 9> init_state;
    init_state << x_c0, y_c0, xyz_world.z(), 0.0, 0.0, 0.0, r_init, armor_pose_yaw_world, 0.0;
    
    Eigen::Matrix<double, 9, 9> init_P = Eigen::Matrix<double, 9, 9>::Identity();
    init_P.diagonal() << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 0.01, 0.1, 1.0;
    
    ekf_.initialize(init_state, init_P);

    // 首次初始化：测量值 = 滤波值 = 初始观测
    updateMeasurement(armor_plate, current_time);
    updateFilteredValue(xyz_camera, xyz_world);

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

    // 使用 EKF 当前状态计算预测的装甲板中心位置
    Eigen::Vector<double, 9> state = ekf_.getStatePost();
    double r = state[6];
    double yaw = state[7];
    Eigen::Vector3d xyz_pred_world(
        state[0] - r * std::cos(yaw),
        state[1] - r * std::sin(yaw),
        state[2]
    );
    float min_dist = std::numeric_limits<float>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < num_plates; ++i) {
        Eigen::Vector3d xyz_camera;
        xyz_camera[0] = armor_plates[i].pose.position.x;
        xyz_camera[1] = armor_plates[i].pose.position.y;
        xyz_camera[2] = armor_plates[i].pose.position.z;
        // camera -> gimbal -> world
        Eigen::Vector3d xyz_gimbal = R_gimbal_camera * xyz_camera;
        Eigen::Vector3d xyz_world = R_w_g_ * xyz_gimbal;
        Eigen::Vector3d diff_world = xyz_world - xyz_pred_world;
        float dist_sq = diff_world.norm();
        if (dist_sq < min_dist) {
            min_dist = dist_sq;
            best_idx = i;
        }
    }
    // 保存结果
    target_armor = armor_plates[best_idx];
}

// 检查Yaw是否突变
bool Tracker::checkYawMutation(const float& armor_pose_yaw)
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

double Tracker::calculateDt(double current_time)
{
    double dt = 0.01;  // 默认10ms
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        if (dt <= 0.0) dt = 0.001;  // 最小1ms
        if (dt > 1.0) dt = 1.0;     // 最大1s
    }
    last_update_time_ = current_time;
    return dt;
}

// 主更新函数
void Tracker::Update(const std::vector<ArmorPlate>& armor_plates,
                     double current_time,
                     float yaw_abs, float pitch_abs)
{
    auto t_start = std::chrono::steady_clock::now();
    solve_ok_ = false;

    // 计算dt（与上次更新的时间差）
    double dt = calculateDt(current_time);
    
    // 坐标系转换计算
    R_w_g_ = getR_world_gimbal(yaw_abs, pitch_abs);
    R_g_w_ = R_w_g_.transpose();
    q_w_g_ = Eigen::Quaterniond(R_w_g_);
    q_g_c_ = Eigen::Quaterniond(R_gimbal_camera);
    // 更新状态转移矩阵中的dt
    ekf_.updateStateTransitionMatrix(dt);
    // 没有目标情况
    if (armor_plates.size() == 0) {
        // 没有检测到目标，进入丢失状态
        is_lost_ = true;
        if (isLostTooLong(current_time)) reset(); // 丢失时间过长，重置
        else if (initialized_) ekf_.predict(); // 丢失但未过长，继续预测
        return;
    }
    // 有检测结果，选择最佳匹配
    ArmorPlate target_plate;
    selectBestMatch(armor_plates, target_plate);
    
    // 提取选中的目标位置（相机系）
    Eigen::Vector3d xyz_camera(
        target_plate.pose.position.x,
        target_plate.pose.position.y,
        target_plate.pose.position.z
    );
    
    // 提取装甲板姿态四元数（相机系）
    Eigen::Quaterniond q_camera(
        target_plate.pose.orientation.w,
        target_plate.pose.orientation.x,
        target_plate.pose.orientation.y,
        target_plate.pose.orientation.z
    );
    // camera -> gimbal -> world
    Eigen::Quaterniond q_gimbal_measurement = q_g_c_ * q_camera;
    Eigen::Quaterniond q_world_measurement = q_w_g_ * q_gimbal_measurement;
    
    // 计算装甲板姿态 yaw（世界坐标系）
    float armor_pose_yaw_world = calculatePoseYaw(q_world_measurement);
    // 判断突变
    if (checkYawMutation(armor_pose_yaw_world)) reset();
    // 如果是第一次初始化，设置初始状态（世界坐标系）
    if (!initialized_) {
        init(target_plate, current_time);
        return;
    }

    // 保存当前帧选中的原始测量值
    updateMeasurement(target_plate, current_time);

    // 世界坐标系下 EKF: Predict
    ekf_.predict();

    // 世界坐标系下的原始测量值 (camera -> gimbal -> world)
    Eigen::Vector3d xyz_gimbal_measurement = R_gimbal_camera * xyz_camera;
    Eigen::Vector3d xyz_world_measurement = R_w_g_ * xyz_gimbal_measurement;
    Eigen::Vector<double, 4> measurement;
    measurement << xyz_world_measurement.x(), xyz_world_measurement.y(), xyz_world_measurement.z(), armor_pose_yaw_world;

    Eigen::Vector<double, 9> state;
    ekf_.correct(measurement);
    state = ekf_.getStatePost();
    solve_ok_ = true;

    Eigen::Vector3d xyz_world_filtered = ekf_.getFilteredObservation().head<3>();
    // world -> gimbal -> camera
    Eigen::Vector3d xyz_gimbal_filtered = R_g_w_ * xyz_world_filtered;
    Eigen::Vector3d xyz_camera_filtered = R_camera_gimbal * xyz_gimbal_filtered;
    updateFilteredValue(xyz_camera_filtered, xyz_world_filtered);

    // EKF 状态向量提取
    center_point_world_ = Eigen::Vector3d(state[0], state[1], state[2]);
    center_velocity_    = Eigen::Vector3d(state[3], state[4], 0);
    center_r_           = static_cast<float>(state[6]);
    filter_orientation_world_ = getQuaternionFromYaw(state[7]);
    auto t_end = std::chrono::steady_clock::now();
    time_cost_ = std::chrono::duration<float, std::milli>(t_end - t_start).count();
}
float calculatePoseYaw(const Eigen::Quaterniond &q)
{
    // Z 轴提取
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
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(yawAngle);
}

void Tracker::updateMeasurement(const ArmorPlate& armor_plate, double current_time)
{
    Eigen::Vector3d xyz_camera(
        armor_plate.pose.position.x,
        armor_plate.pose.position.y,
        armor_plate.pose.position.z
    );
    // camera -> gimbal -> world
    Eigen::Vector3d xyz_gimbal = R_gimbal_camera * xyz_camera;
    Eigen::Vector3d xyz_world = R_w_g_ * xyz_gimbal;

    measured_position_camera_ = xyz_camera;
    measured_position_world_  = xyz_world;
    measured_yaw_   = calculateYaw(xyz_camera);
    measured_pitch_ = calculatePitch(xyz_camera);

    Eigen::Quaterniond q_camera(
        armor_plate.pose.orientation.w,
        armor_plate.pose.orientation.x,
        armor_plate.pose.orientation.y,
        armor_plate.pose.orientation.z
    );
    // camera -> gimbal -> world
    Eigen::Quaterniond q_gimbal = q_g_c_ * q_camera;
    Eigen::Quaterniond q_world = q_w_g_ * q_gimbal;
    float armor_yaw_world = calculatePoseYaw(q_world);
    measured_orientation_world_ = getQuaternionFromYaw(armor_yaw_world);

    last_armor_pose_yaw_world_ = armor_yaw_world;
    last_armor_number_ = armor_plate.number;
    last_detection_time_ = current_time;
    is_lost_ = false;
}

void Tracker::updateFilteredValue(const Eigen::Vector3d& xyz_camera_filtered, const Eigen::Vector3d& xyz_world_filtered)
{
    filter_position_camera_ = xyz_camera_filtered;
    filter_position_world_  = xyz_world_filtered;
    yaw_   = calculateYaw(xyz_camera_filtered);
    pitch_ = calculatePitch(xyz_camera_filtered);
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
    msg.method = "ekf";
    msg.solve_ok = solve_ok_;

    return msg;
}