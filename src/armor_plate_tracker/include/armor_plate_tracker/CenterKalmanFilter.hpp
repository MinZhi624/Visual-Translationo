#pragma once
#include <Eigen/Core>

class CenterKalmanFilter
{
private:
    // x 状态向量 [x_c, y_c, v_x, v_y]
    Eigen::Vector4d state_pre_;
    Eigen::Vector4d state_post_;
    // P 状态协方差矩阵
    Eigen::Matrix4d error_cov_pre_;
    Eigen::Matrix4d error_cov_post_;
    // F 状态转移矩阵
    Eigen::Matrix4d state_transition_matrix_;
    // H 观测矩阵 (线性，直接观测 x_c, y_c)
    Eigen::Matrix<double, 2, 4> observation_matrix_;
    // Q 过程噪声协方差矩阵
    Eigen::Matrix4d process_noise_cov_;
    // R 观测噪声协方差矩阵
    Eigen::Matrix2d observation_noise_cov_;
    // K 卡尔曼增益
    Eigen::Matrix<double, 4, 2> kalman_gain_;
    // Z 观测值、滤波值
    Eigen::Vector2d origin_observation_;
    Eigen::Vector2d filtered_observation_;

public:
    CenterKalmanFilter();
    void initialize(const Eigen::Vector4d& state_pre, const Eigen::Matrix4d& error_cov_pre);
    // 核心
    void predict();
    Eigen::Vector2d correct(const Eigen::Vector2d& measurement);
    // 设置
    void updateStateTransitionMatrix(double dt);

    Eigen::Vector4d getStatePre() const { return state_pre_; }
    Eigen::Vector4d getStatePost() const { return state_post_; }
    Eigen::Vector2d getFilteredObservation() const { return filtered_observation_; }
};
