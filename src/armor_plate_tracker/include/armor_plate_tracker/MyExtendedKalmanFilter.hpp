#pragma once
#include <Eigen/Core>

class MyExtendedKalmanFilter
{
private:
    // x 状态向量
    // 包括： x, y, z, v_x, v_y, v_z, r, yaw, v_yaw, 
    // 其中x，y代表的是旋转轴的位置， z则是装甲板高度
    Eigen::Vector<double, 9> state_pre_;
    Eigen::Vector<double, 9> state_post_;
    // P 状态协方差矩阵
    Eigen::Matrix<double, 9, 9> error_cov_pre_;
    Eigen::Matrix<double, 9, 9> error_cov_post_;
    // F 状态转移矩阵 --> 这里状态转移是线性化
    Eigen::Matrix<double, 9, 9> state_transition_matrix_;
    // H 观测雅可比矩阵 
    Eigen::Matrix<double, 4, 9> observation_jacobian_;
    // Q 过程噪声协方差矩阵
    Eigen::Matrix<double, 9, 9> process_noise_cov_;
    // R 观测噪声协方差矩阵
    Eigen::Matrix<double, 4, 4> observation_noise_cov_;
    // K 卡尔曼增益
    Eigen::Matrix<double, 9, 4> kalman_gain_;
    // Z 观测值、滤波值
    Eigen::Vector<double, 4> origin_observation_;    
    Eigen::Vector<double, 4> filtered_observation_;  
    void calculateObservationJacobian();
    void measurementFunction(const Eigen::Vector<double, 9>& state, Eigen::Vector<double, 4>& observation);
    void checkValue();
public:
    MyExtendedKalmanFilter();
    void initialize(const Eigen::Vector<double, 9>& state_pre, const Eigen::Matrix<double, 9, 9>& error_cov_pre);
    // 核心
    void predict();
    Eigen::Vector<double, 4> correct(const Eigen::Vector<double, 4>& measurement);
    // 设置
    void updateStateTransitionMatrix(const double& dt);
    void setStatePre(Eigen::Vector<double, 9> state_pre) { state_pre_ = state_pre; }
    void setStatePost(Eigen::Vector<double, 9> state_post) { state_post_ = state_post; }
    void setErrorCovPre(Eigen::Matrix<double, 9, 9> error_cov_pre) { error_cov_pre_ = error_cov_pre; }
    void setErrorCovPost(Eigen::Matrix<double, 9, 9> error_cov_post) { error_cov_post_ = error_cov_post; }

    Eigen::Vector<double, 9> getStatePre() const { return state_pre_; }
    Eigen::Vector<double, 9> getStatePost() const { return state_post_; }
    Eigen::Vector<double, 4> getFilteredObservation() const { return filtered_observation_; }
};