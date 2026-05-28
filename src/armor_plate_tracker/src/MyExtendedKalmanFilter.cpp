#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"
#include <Eigen/Dense>
#include "Eigen/src/Core/Matrix.h"

static double normalizeRadAngle(double & rad)
{
    while (rad > M_PI) rad -= 2.0f * M_PI;
    while (rad < -M_PI) rad += 2.0f * M_PI;
    return rad;
}

MyExtendedKalmanFilter::MyExtendedKalmanFilter()
{
    state_pre_ = Eigen::Vector<double, 9>::Zero();
    state_post_ = Eigen::Vector<double, 9>::Zero();

    error_cov_pre_ = Eigen::Matrix<double, 9, 9>::Identity();
    error_cov_post_ = Eigen::Matrix<double, 9, 9>::Identity();

    state_transition_matrix_ = Eigen::Matrix<double, 9, 9>::Identity();
    observation_jacobian_ = Eigen::Matrix<double, 4, 9>::Zero();

    process_noise_cov_ = Eigen::Matrix<double, 9, 9>::Zero();
    process_noise_cov_.diagonal() <<
        0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.0005, 0.001, 0.01;
        
    observation_noise_cov_ = Eigen::Matrix<double, 4, 4>::Zero();
    observation_noise_cov_.diagonal() <<
        4e-3, 4e-3, 1,  9e-2;

    kalman_gain_ = Eigen::Matrix<double, 9, 4>::Zero();
    origin_observation_ = Eigen::Vector<double, 4>::Zero();
    filtered_observation_ = Eigen::Vector<double, 4>::Zero();
}

void MyExtendedKalmanFilter::initialize(
    const Eigen::Vector<double, 9>& state_pre,
    const Eigen::Matrix<double, 9, 9>& error_cov_pre)
{
    state_pre_ = state_pre;
    state_post_ = state_pre;
    error_cov_pre_ = error_cov_pre;
    error_cov_post_ = error_cov_pre;
}

void MyExtendedKalmanFilter::predict()
{
    state_pre_ = state_transition_matrix_ * state_post_;
    error_cov_pre_ = state_transition_matrix_ * error_cov_post_ * state_transition_matrix_.transpose()
                   + process_noise_cov_;
    
    // 先验 yaw 归一化到 [-π, π]，防止 predict 后角度越界
    while (state_pre_[7] > M_PI) state_pre_[7] -= 2.0 * M_PI;
    while (state_pre_[7] < -M_PI) state_pre_[7] += 2.0 * M_PI;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::correct(const Eigen::Vector<double, 4>& measurement, int armor_id)
{
    armor_id_ = armor_id;
    // 保存原始观测值
    origin_observation_ = measurement;
    observation_jacobian_= calculateObservationJacobian();
    // 计算观测预测值
    auto predicted_obs = measurementFunction(state_pre_);

    // 计算卡尔曼增益 K = P_pre * H^T * (H * P_pre * H^T + R)^(-1)
    kalman_gain_ = error_cov_pre_ * observation_jacobian_.transpose()
                 * (observation_jacobian_ * error_cov_pre_ * observation_jacobian_.transpose()
                    + observation_noise_cov_).inverse();

    // 计算残差并更新后验状态
    Eigen::Vector<double, 4> residual = measurement - predicted_obs;
    
    // 角度归一化 yaw angle
    residual[0] = normalizeRadAngle(residual[0]);
    residual[3] = normalizeRadAngle(residual[3]);
    
    state_post_ = state_pre_ + kalman_gain_ * residual;
    // 约束 
    checkValue();

    // 更新后验误差协方差 P_post = (I - K * H) * P_pre
    Eigen::Matrix<double, 9, 9> identity = Eigen::Matrix<double, 9, 9>::Identity();
    error_cov_post_ = (identity - kalman_gain_ * observation_jacobian_) * error_cov_pre_;

    // 计算滤波后的观测值
    filtered_observation_ = measurementFunction(state_post_);
    return filtered_observation_;
}

void MyExtendedKalmanFilter::updateStateTransitionMatrix(const double& dt)
{
    /*
        方程如下:  都是线性的
        x^k = x^{k-1} + v^{k-1}_x * dt 
        y^k = y^{k-1} + v^{k-1}_y * dt
        Z^k = Z^{k-1} + v^{k-1}_z * dt
        v^k_x = v^{k-1}_x
        v^k_y = v^{k-1}_y
        v^k_z = v^{k-1}_z 
        r^k = r^{k-1}
        yaw^k = yaw^{k-1} + v^{k-1}_yaw * dt
        v^k_yaw = v^{k-1}_yaw
    */
    state_transition_matrix_ << 
        1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
}

Eigen::Matrix<double, 4, 9> MyExtendedKalmanFilter::calculateObservationJacobian()
{
    // 链式法则求导 
    auto xyza_state_jacobian = calculateStateToXYZAJacobian(state_pre_, armor_id_);
    auto xyza_armor_ = measurementFunctionStateToXYZA(state_pre_, armor_id_);
    auto ypda_xyza_jacobian = calculateXYZAToYPDAJacobian(xyza_armor_);
    return ypda_xyza_jacobian * xyza_state_jacobian; // 4x9
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunction(const Eigen::Vector<double, 9>& state)
{
    auto xyza_state = measurementFunctionStateToXYZA(state, armor_id_);
    auto ypda_xyza = measurementFunctionXYZAToYPDA(xyza_state);
    return ypda_xyza;
}

void MyExtendedKalmanFilter::checkValue()
{
    // r 硬限制在 [0.12, 0.4]
    state_post_[6] = std::max(0.12, state_post_[6]);
    state_post_[6] = std::min(0.4, state_post_[6]);

    // yaw 归一化到 [-π, π]
    while (state_post_[7] > M_PI) state_post_[7] -= 2.0 * M_PI;
    while (state_post_[7] < -M_PI) state_post_[7] += 2.0 * M_PI;
}

Eigen::Matrix<double, 4, 9> MyExtendedKalmanFilter::calculateStateToXYZAJacobian(const Eigen::Vector<double, 9>& state, int armor_id)
{
    /*  
        方程如下:
        car_yaw = yaw + armor_id * PI / 2
        x = x_c - r * cos(car_yaw)
        y = y_c - r * sin(car_yaw)
        z = z
        yaw = car_yaw

        H = dh/dx =
        [1, 0, 0, 0, 0, 0, -cos(car_yaw),  r*sin(car_yaw), 0]
        [0, 1, 0, 0, 0, 0, -sin(car_yaw), -r*cos(car_yaw), 0]
        [0, 0, 1, 0, 0, 0, 0,          0,           0]
        [0, 0, 0, 0, 0, 0, 0,          1,           0]
    */
    Eigen::Matrix<double, 4, 9> xyza_state_jacobian;
    double car_yaw = state[7] + armor_id * M_PI / 2;
    xyza_state_jacobian <<
        1, 0, 0, 0, 0, 0, -std::cos(car_yaw),  state[6] * std::sin(car_yaw),  0,
        0, 1, 0, 0, 0, 0, -std::sin(car_yaw), -state[6] * std::cos(car_yaw), 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0;
    return xyza_state_jacobian;
}
Eigen::Matrix4d MyExtendedKalmanFilter::calculateXYZAToYPDAJacobian(const Eigen::Vector<double, 4> & xyza)
{
    double x = xyza[0];
    double y = xyza[1];
    double z = xyza[2];

    double dyaw_dx = -y / (x * x + y * y);
    double dyaw_dy = x / (x * x + y * y);
    double dyaw_dz = 0.0;

    double dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    double dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
    double dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

    double ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
    double ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
    double ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);
    Eigen::Matrix4d jacobian;
    jacobian << 
        dyaw_dx,        dyaw_dy,        dyaw_dz,        0.0,
        dpitch_dx,      dpitch_dy,      dpitch_dz,      0.0,
        ddistance_dx,   ddistance_dy,   ddistance_dz,   0.0,
        0.0,            0.0,            0.0,            1.0;
    return jacobian;
};
Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunctionStateToXYZA(const Eigen::Vector<double, 9>& state, int armor_id)
{   
    /*  
        方程如下:
        car_yaw = yaw + armor_id * PI / 2
        x = x_c - r * cos(car_yaw)
        y = y_c - r * sin(car_yaw)
        z = z
        yaw = car_yaw
    */
    double x_c = state[0];
    double y_c = state[1];
    double z_c = state[2];
    double r = state[6];
    double yaw = state[7];
    double car_yaw = yaw + armor_id * M_PI / 2;

    Eigen::Vector<double, 4> observation;
    observation <<
        x_c - r * std::cos(car_yaw),
        y_c - r * std::sin(car_yaw),
        z_c,
        car_yaw;
    return observation;  // 4x1
}
Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunctionXYZAToYPDA(const Eigen::Vector<double, 4>& xyza)
{   
    /*
        方程如下:
        yaw = atan2(y, x)
        pitch = atan2(z, sqrt(x^2 + y^2))
        distance = sqrt(x^2 + y^2 + z^2)
    */
    double x = xyza[0];
    double y = xyza[1];
    double z = xyza[2];

    double yaw = std::atan2(y, x);
    double pitch = std::atan2(z, std::sqrt(x * x + y * y));
    double distance = Eigen::Vector3d{x, y, z}.norm();
    return {
        yaw,
        pitch,
        distance, 
        xyza[3]  
    };
}