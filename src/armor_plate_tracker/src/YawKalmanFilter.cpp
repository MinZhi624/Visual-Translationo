#include "armor_plate_tracker/YawKalmanFilter.hpp"
#include <cmath>

YawKalmanFilter::YawKalmanFilter()
{
    state_pre_.setZero();
    state_post_.setZero();

    error_cov_pre_.setIdentity();
    error_cov_post_.setIdentity();

    state_transition_matrix_.setIdentity();
    observation_matrix_ << 1.0, 0.0;

    // Q: 过程噪声
    process_noise_cov_.setZero();
    process_noise_cov_.diagonal() << 0.001, 0.001;

    // R: 观测噪声
    observation_noise_cov_ = 0.0001;

    kalman_gain_.setZero();
    origin_observation_ = 0.0;
    filtered_observation_ = 0.0;
}

void YawKalmanFilter::initialize(
    const Eigen::Vector2d& state_pre,
    const Eigen::Matrix2d& error_cov_pre)
{
    state_pre_ = state_pre;
    state_post_ = state_pre;
    error_cov_pre_ = error_cov_pre;
    error_cov_post_ = error_cov_pre;
}

void YawKalmanFilter::predict()
{
    state_pre_ = state_transition_matrix_ * state_post_;
    error_cov_pre_ = state_transition_matrix_ * error_cov_post_ * state_transition_matrix_.transpose()
                   + process_noise_cov_;

    // yaw 归一化到 [-π, π]
    while (state_pre_[0] > M_PI) state_pre_[0] -= 2.0 * M_PI;
    while (state_pre_[0] < -M_PI) state_pre_[0] += 2.0 * M_PI;
}

double YawKalmanFilter::correct(double measurement)
{
    origin_observation_ = measurement;

    // 观测预测 h(x_pre) = H * x_pre
    double predicted_obs = observation_matrix_ * state_pre_;

    // 协方差投影到观测空间 S = H * P_pre * H^T + R
    double S = (observation_matrix_ * error_cov_pre_ * observation_matrix_.transpose())(0, 0)
             + observation_noise_cov_;

    // 卡尔曼增益 K = P_pre * H^T * S^(-1)
    kalman_gain_ = error_cov_pre_ * observation_matrix_.transpose() / S;

    // 残差（带角度归一化）
    double residual = measurement - predicted_obs;
    while (residual > M_PI) residual -= 2.0 * M_PI;
    while (residual < -M_PI) residual += 2.0 * M_PI;

    // 后验状态更新
    state_post_ = state_pre_ + kalman_gain_ * residual;

    // yaw 归一化
    while (state_post_[0] > M_PI) state_post_[0] -= 2.0 * M_PI;
    while (state_post_[0] < -M_PI) state_post_[0] += 2.0 * M_PI;

    // 后验协方差更新 P_post = (I - K * H) * P_pre
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    error_cov_post_ = (I - kalman_gain_ * observation_matrix_) * error_cov_pre_;

    // 滤波后的观测值
    filtered_observation_ = state_post_[0];

    return filtered_observation_;
}

void YawKalmanFilter::updateStateTransitionMatrix(double dt)
{
    /*
        方程:
        yaw^k  = yaw^{k-1} + omega^{k-1} * dt
        omega^k = omega^{k-1}
    */
    state_transition_matrix_ <<
        1.0, dt,
        0.0, 1.0;
}
