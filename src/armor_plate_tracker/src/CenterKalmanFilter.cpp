#include "armor_plate_tracker/CenterKalmanFilter.hpp"
#include <Eigen/Dense>

CenterKalmanFilter::CenterKalmanFilter()
{
    state_pre_.setZero();
    state_post_.setZero();

    error_cov_pre_.setIdentity();
    error_cov_post_.setIdentity();

    state_transition_matrix_.setIdentity();
    // H = [[1, 0, 0, 0],
    //      [0, 1, 0, 0]]
    observation_matrix_.setZero();
    observation_matrix_(0, 0) = 1.0;
    observation_matrix_(1, 1) = 1.0;

    // Q: 过程噪声
    process_noise_cov_.setZero();
    process_noise_cov_.diagonal() << 0.01, 0.01, 0.1, 0.1;

    // R: 观测噪声
    observation_noise_cov_.setZero();
    observation_noise_cov_.diagonal() << 0.004, 0.004;

    kalman_gain_.setZero();
    origin_observation_.setZero();
    filtered_observation_.setZero();
}

void CenterKalmanFilter::initialize(
    const Eigen::Vector4d& state_pre,
    const Eigen::Matrix4d& error_cov_pre)
{
    state_pre_ = state_pre;
    state_post_ = state_pre;
    error_cov_pre_ = error_cov_pre;
    error_cov_post_ = error_cov_pre;
}

void CenterKalmanFilter::predict()
{
    state_pre_ = state_transition_matrix_ * state_post_;
    error_cov_pre_ = state_transition_matrix_ * error_cov_post_ * state_transition_matrix_.transpose()
                   + process_noise_cov_;
}

Eigen::Vector2d CenterKalmanFilter::correct(const Eigen::Vector2d& measurement)
{
    origin_observation_ = measurement;

    // 观测预测 h(x_pre) = H * x_pre
    Eigen::Vector2d predicted_obs = observation_matrix_ * state_pre_;

    // 协方差投影到观测空间 S = H * P_pre * H^T + R
    Eigen::Matrix2d S = observation_matrix_ * error_cov_pre_ * observation_matrix_.transpose()
                      + observation_noise_cov_;

    // 卡尔曼增益 K = P_pre * H^T * S^(-1)
    kalman_gain_ = error_cov_pre_ * observation_matrix_.transpose() * S.inverse();

    // 残差
    Eigen::Vector2d residual = measurement - predicted_obs;

    // 后验状态更新
    state_post_ = state_pre_ + kalman_gain_ * residual;

    // 后验协方差更新 P_post = (I - K * H) * P_pre
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    error_cov_post_ = (I - kalman_gain_ * observation_matrix_) * error_cov_pre_;

    // 滤波后的观测值
    filtered_observation_ = state_post_.head<2>();

    return filtered_observation_;
}

void CenterKalmanFilter::updateStateTransitionMatrix(double dt)
{
    /*
        方程:
        x_c^k = x_c^{k-1} + v_x^{k-1} * dt
        y_c^k = y_c^{k-1} + v_y^{k-1} * dt
        v_x^k = v_x^{k-1}
        v_y^k = v_y^{k-1}
    */
    state_transition_matrix_ <<
        1.0, 0.0, dt,  0.0,
        0.0, 1.0, 0.0, dt,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
}
