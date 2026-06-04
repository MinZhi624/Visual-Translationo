#pragma once
#include <Eigen/Core>
#include <deque>

class MyExtendedKalmanFilter
{
private:
    // x 状态向量
    //       0   1    2   3    4   5    6     7     8  9  10
    // 包括： x, v_x,  y, v_y,  z, v_z, yaw, omega,  r, l, h
    // 其中 x、y、z 代表旋转轴位置，r 是基础半径，l/h 是另一组装甲板的半径/高度差
    Eigen::Vector<double, 11> state_;
    // P 状态协方差矩阵
    Eigen::Matrix<double, 11, 11> error_cov_;
    // F 状态转移矩阵 --> 这里状态转移是线性化
    Eigen::Matrix<double, 11, 11> state_transition_matrix_;
    // H 观测雅可比矩阵 
    Eigen::Matrix<double, 4, 11> observation_jacobian_;
    // Q 过程噪声协方差矩阵
    Eigen::Matrix<double, 11, 11> process_noise_cov_;
    // R 观测噪声协方差矩阵
    Eigen::Matrix<double, 4, 4> observation_noise_cov_;
    // K 卡尔曼增益
    Eigen::Matrix<double, 11, 4> kalman_gain_;
    // Z 观测值、滤波值
    Eigen::Vector<double, 4> origin_observation_;    
    Eigen::Vector<double, 4> filtered_observation_;  
    // 状态评估
    std::deque<int> nis_failures_; // 这里采用int是为了方便统计
    static constexpr double NIS_THRESHOLD = 9.49; // 4维观测95%置信度下的标准NIS阈值
    int armor_id_ = 0;
    
    Eigen::Matrix<double, 4, 11> calculateObservationJacobian();
    static Eigen::Matrix4d calculateXYZAToYPDAJacobian(const Eigen::Vector<double, 4> & xyza);
    static Eigen::Matrix<double, 4, 11> calculateStateToXYZAJacobian(const Eigen::Vector<double, 11> & state, int armor_id);

    Eigen::Vector<double, 4> measurementFunction();
    static Eigen::Vector<double, 4> measurementFunctionStateToXYZA(const Eigen::Vector<double, 11>& state, int armor_id);
    static Eigen::Vector<double, 4> measurementFunctionXYZAToYPDA(const Eigen::Vector<double, 4>& xyza);
public:
    static constexpr size_t NIS_WINDOW_SIZE = 100;

    MyExtendedKalmanFilter();
    void initialize(const Eigen::Vector<double, 11>& state, const Eigen::Matrix<double, 11, 11>& error_cov);
    // 核心
    void predict();
    Eigen::Vector<double, 4> correct(const Eigen::Vector<double, 4>& measurement, int armor_id);
    // 设置
    void updateProcessNoiseCov(const double & dt);
    void updateStateTransitionMatrix(const double & dt);
    std::deque<int> getNISFailures() const { return nis_failures_; }
    Eigen::Vector<double, 11> getState() const { return state_; }
    Eigen::Vector<double, 4> getFilteredObservation() const { return filtered_observation_; }
};
