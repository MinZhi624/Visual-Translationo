#pragma once
#include <Eigen/Core>

class MyExtendedKalmanFilter
{
private:
    // x 状态向量
    //       0   1    2   3    4   5    6     7     8
    // 包括： x, v_x,  y, v_y,  z, v_z, yaw, omega,  r, 
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
    int armor_id_ = 0;
    
    Eigen::Matrix<double, 4, 9> calculateObservationJacobian();
    Eigen::Matrix4d calculateXYZAToYPDAJacobian(const Eigen::Vector<double, 4> & xyza);
    Eigen::Matrix<double, 4, 9> calculateStateToXYZAJacobian(const Eigen::Vector<double, 9> & state, int armor_id);

    Eigen::Vector<double, 4> measurementFunction(const Eigen::Vector<double, 9>& state);
    Eigen::Vector<double, 4> measurementFunctionStateToXYZA(const Eigen::Vector<double, 9>& state, int armor_id);
    Eigen::Vector<double, 4> measurementFunctionXYZAToYPDA(const Eigen::Vector<double, 4>& xyza);
    
    void checkValue();
public:
    MyExtendedKalmanFilter();
    void initialize(const Eigen::Vector<double, 9>& state_pre, const Eigen::Matrix<double, 9, 9>& error_cov_pre);
    // 核心
    void predict();
    Eigen::Vector<double, 4> correct(const Eigen::Vector<double, 4>& measurement, int armor_id);
    // 设置
    void updateProcessNoiseCov(const double & dt);
    void updateStateTransitionMatrix(const double & dt);
    void setStatePre(Eigen::Vector<double, 9> state_pre) { state_pre_ = state_pre; }
    void setStatePost(Eigen::Vector<double, 9> state_post) { state_post_ = state_post; }
    void setErrorCovPre(Eigen::Matrix<double, 9, 9> error_cov_pre) { error_cov_pre_ = error_cov_pre; }
    void setErrorCovPost(Eigen::Matrix<double, 9, 9> error_cov_post) { error_cov_post_ = error_cov_post; }

    Eigen::Vector<double, 9> getStatePre() const { return state_pre_; }
    Eigen::Vector<double, 9> getStatePost() const { return state_post_; }
    Eigen::Vector<double, 4> getFilteredObservation() const { return filtered_observation_; }
};
