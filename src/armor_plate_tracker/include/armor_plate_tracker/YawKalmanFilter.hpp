#pragma once
#include <Eigen/Core>

class YawKalmanFilter
{
private:
    // x 状态向量 [yaw, omega]
    Eigen::Vector2d state_pre_;
    Eigen::Vector2d state_post_;
    // P 状态协方差矩阵
    Eigen::Matrix2d error_cov_pre_;
    Eigen::Matrix2d error_cov_post_;
    // F 状态转移矩阵
    Eigen::Matrix2d state_transition_matrix_;
    // H 观测矩阵 (线性，直接观测 yaw)
    Eigen::RowVector2d observation_matrix_;
    // Q 过程噪声协方差矩阵
    Eigen::Matrix2d process_noise_cov_;
    // R 观测噪声协方差矩阵
    double observation_noise_cov_;
    // K 卡尔曼增益
    Eigen::Vector2d kalman_gain_;
    // Z 观测值、滤波值
    double origin_observation_;
    double filtered_observation_;

public:
    YawKalmanFilter();
    void initialize(const Eigen::Vector2d& state_pre, const Eigen::Matrix2d& error_cov_pre);
    // 核心
    void predict();
    double correct(double measurement);
    // 设置
    void updateStateTransitionMatrix(double dt);

    Eigen::Vector2d getStatePre() const { return state_pre_; }
    Eigen::Vector2d getStatePost() const { return state_post_; }
    double getFilteredObservation() const { return filtered_observation_; }
};
