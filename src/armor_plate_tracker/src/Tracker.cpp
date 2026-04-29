#include "armor_plate_tracker/Tracker.hpp"
#include "Eigen/src/Geometry/Quaternion.h"
#include <algorithm>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
// opencv坐标系 → 云台坐标系 (x向前, y向左, z向上)
const Eigen::Matrix3d R_w_cv = (Eigen::Matrix3d() <<
    0.0 ,  0.0,  1.0,
    -1.0,  0.0,  0.0,
    0.0 , -1.0,  0.0).finished();
const Eigen::Quaterniond q_w_cv(R_w_cv);

// 默认构造函数
Tracker::Tracker()
    : x_kf_origin_(3, 1), y_kf_origin_(3, 1), z_kf_origin_(3, 1)
    , x_kf_(3, 1), y_kf_(3, 1), z_kf_(3, 1)
    , yaw_(0.0f), pitch_(0.0f)
    , last_update_time_(0.0)
    , last_detection_time_(0.0)
    , initialized_(false)
    , max_lost_time_(0.1)
    , is_lost_(false)
    , yaw_mutation_threshold_(0.05f) 
{
}
// 初始化滤波器参数
void Tracker::Init()
{
    // 初始化x/y/z滤波器
    initFilter(x_kf_origin_);
    initFilter(x_kf_);
    initFilter(y_kf_origin_);
    initFilter(y_kf_);
    initFilter(z_kf_origin_);
    initFilter(z_kf_);
    
    initialized_ = false;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_armor_pose_yaw_ = 0.0f;
    is_lost_ = false;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
}

// 设置最大丢失时间（秒）
void Tracker::setMaxLostTime(double seconds)
{
    max_lost_time_ = seconds;
}

// 设置突变阈值
void Tracker::setMutationThreshold(float yaw_thresh)
{
    yaw_mutation_threshold_ = yaw_thresh;
}

// 初始化单个滤波器（三阶模型：位置-速度-加速度）
void Tracker::initFilter(MyKalmanFilter& kf)
{
    // 测量矩阵 H (1x3) - 只观测位置
    Eigen::MatrixXf H(1, 3);
    H << 1.0f, 0.0f, 0.0f;
    kf.setMeasurementMatrix(H);
    
    // 过程噪声协方差 Q (3x3)
    Eigen::MatrixXf Q(3, 3);
    Q << 0.001f, 0.0f, 0.0f,
         0.0f, 0.001f, 0.0f,
         0.0f, 0.0f, 0.001f;
    kf.setProcessNoiseCov(Q);
    
    // 测量噪声协方差 R (1x1)
    Eigen::MatrixXf R(1, 1);
    R << 0.001f;
    kf.setMeasurementNoiseCov(R);
    
    // 初始化状态（设为0）
    Eigen::MatrixXf state(3, 1);
    state << 0.0f, 0.0f, 0.0f;
    kf.setStatePost(state);
    kf.setStatePre(state);
    
    // 初始化误差协方差 P (单位矩阵）
    Eigen::MatrixXf P(3, 3);
    P << 1.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 1.0f;
    kf.setErrorCovPost(P);
}

// 根据dt更新状态转移矩阵
void Tracker::updateTransitionMatrix(MyKalmanFilter& kf, double dt)
{
    Eigen::MatrixXf A(3, 3);
    float dt_f = static_cast<float>(dt);
    float dt2_half = 0.5f * dt_f * dt_f;
    A << 1.0f, dt_f, dt2_half,
         0.0f, 1.0f, dt_f,
         0.0f, 0.0f, 1.0f;
    kf.setTransitionMatrix(A);
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

    // 使用当前 x,y,z 状态作为预测位置
    Eigen::Vector3d pred_pos(
        x_kf_.getStatePost()(0, 0),
        y_kf_.getStatePost()(0, 0),
        z_kf_.getStatePost()(0, 0)
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

// 检查是否突变
bool Tracker::isMutation(const float& armor_pose_yaw)
{
    if (!initialized_) {
        return false;
    }
    float dy = std::abs(armor_pose_yaw - last_armor_pose_yaw_);
    dy = normalizeRadAngle(dy);
    return (dy > yaw_mutation_threshold_);
}

// 重置滤波器
void Tracker::resetFilter()
{
    // 复制原始模板滤波器
    x_kf_ = x_kf_origin_;
    y_kf_ = y_kf_origin_;
    z_kf_ = z_kf_origin_;
    initialized_ = false;
    is_lost_ = false;
    last_armor_pose_yaw_ = 0.0f;
    measured_pitch_ = 0.0f;
    measured_yaw_ = 0.0f;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
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
    updateTransitionMatrix(x_kf_, dt);
    updateTransitionMatrix(y_kf_, dt);
    updateTransitionMatrix(z_kf_, dt);
    // 没有目标情况
    if (armor_plates.size() == 0) {
        // 没有检测到目标，进入丢失状态
        is_lost_ = true;
        if (isLostTooLong(current_time)) {
            // 丢失太久，重置滤波器
            resetFilter();
        } else {
            // 短暂丢失，继续预测
            if (initialized_) {
                x_kf_.predict();
                y_kf_.predict();
                z_kf_.predict();
            }
        }
        return;
    }
    // 有检测结果，选择最佳匹配
    ArmorPlate target_plate;
    selectBestMatch(armor_plates, target_plate);
    
    // 提取选中的目标位置（相机系）
    Eigen::Vector3d target_position(
        target_plate.pose.position.x,
        target_plate.pose.position.y,
        target_plate.pose.position.z
    );
    
    // 保存当前帧选中的原始测量值
    measured_position_camera_ = target_position;
    measured_yaw_ = calculateYaw(target_position);
    measured_pitch_ = calculatePitch(target_position);
    
    // 检查是否突变（基于装甲板姿态四元数的yaw）
    Eigen::Quaterniond target_plate_q(
        target_plate.pose.orientation.w,
        target_plate.pose.orientation.x,
        target_plate.pose.orientation.y,
        target_plate.pose.orientation.z
    );
    float armor_pose_yaw = calculatePoseYaw(target_plate_q);
    if (isMutation(armor_pose_yaw)) resetFilter();

    // 如果是第一次初始化，设置初始状态（世界坐标系）
    if (!initialized_) {
        Eigen::Vector3d pw = R_w_c_ * target_position;
        Eigen::MatrixXf init_state(3, 1);
        init_state << static_cast<float>(pw.x()), 0.0f, 0.0f;
        x_kf_.setStatePost(init_state);
        
        init_state << static_cast<float>(pw.y()), 0.0f, 0.0f;
        y_kf_.setStatePost(init_state);
        
        init_state << static_cast<float>(pw.z()), 0.0f, 0.0f;
        z_kf_.setStatePost(init_state);
        
        yaw_ = calculateYaw(target_position);
        pitch_ = calculatePitch(target_position);
        initialized_ = true;
        is_lost_ = false;
        last_detection_time_ = current_time;
        return;
    }
    
    // 世界坐标系下 KF
    // Predict
    x_kf_.predict();
    y_kf_.predict();
    z_kf_.predict();
    // Correct
    Eigen::Vector3d pw_m = R_w_c_ * target_position;
    Eigen::MatrixXf measurement(1, 1);
    measurement << static_cast<float>(pw_m.x());
    x_kf_.correct(measurement);
     
    measurement << static_cast<float>(pw_m.y());
    y_kf_.correct(measurement);

    measurement << static_cast<float>(pw_m.z());
    z_kf_.correct(measurement);
    // 提取世界系滤波结果，转回相机系，再算角度
    float wx_f = x_kf_.getStatePost()(0, 0);
    float wy_f = y_kf_.getStatePost()(0, 0);
    float wz_f = z_kf_.getStatePost()(0, 0);
    Eigen::Vector3d pw_f = Eigen::Vector3d(wx_f, wy_f, wz_f);
    Eigen::Vector3d pc_f = R_c_w_ * pw_f;
    yaw_ = calculateYaw(pc_f);
    pitch_ = calculatePitch(pc_f);
    // 保存世界系滤波结果
    Eigen::Quaterniond qw_m = q_w_c_ * target_plate_q; 
    measured_position_world = poseFromEigen(pw_m, qw_m);
    filter_position_world = poseFromEigen(pw_f, qw_m);
    measured_position_camera = target_position;
    filter_position_camera = pc_f;
    // 更新检测到目标的时间和状态
    last_detection_time_ = current_time;
    last_armor_pose_yaw_ = armor_pose_yaw;
    is_lost_ = false;
}
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
PoseStamped poseFromEigen(const Eigen::Vector3d& tvec, const Eigen::Quaterniond& q)
{
    PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = tvec.x();
    pose_stamped.pose.position.y = tvec.y();
    pose_stamped.pose.position.z = tvec.z();
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    return pose_stamped;
}