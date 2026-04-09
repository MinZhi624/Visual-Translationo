#pragma once

#include <Eigen/Core>
/// <summary>
/// 通过Eigen库实现的卡尔曼滤波器。这里简化了控制维度为0，即没有控制输入
/// </summary>
class MyKalmanFilter
{
protected:
	int state_dim_; // 状态维度
	int measure_dim_; // 测量维度

	Eigen::MatrixXf transition_matrix_; //A: 状态转移矩阵 (状态维×状态维)
	Eigen::MatrixXf measurement_matrix_; //H: 测量矩阵 (测量维×状态维)

	Eigen::MatrixXf process_noise_cov_; //Q: 过程噪声协方差 (状态维×状态维)
	Eigen::MatrixXf measurement_noise_cov_; // R: 测量噪声协方差 (测量维×测量维)
	// x: 状态估计向量 (状态维×1)
	Eigen::MatrixXf state_pre_; //先验状态  \hat{x_{k^{-}} }
	Eigen::MatrixXf state_post_; //后验状态 \hat{x_{k}}
	// P: 误差协方差矩阵 (状态维×状态维)
	Eigen::MatrixXf error_cov_pre_; //先验误差协方差 \hat{P_{k^{-}} }
	Eigen::MatrixXf error_cov_post_; //后验误差协方差 \hat{P_{k}}

	Eigen::MatrixXf kalman_gain_; //K: 卡尔曼增益 (状态维×测量维)


public:
	/// <summary>
	/// 初始化卡尔曼滤波器
	/// </summary>
	/// <param name="state_dim">状态向量维度</param>
	/// <param name="measure_dim">测量状态维度</param>
	/// <param name="control_dim">控制维度</param>
	MyKalmanFilter(int state_dim, int measure_dim);

	MyKalmanFilter();
	/// <summary>
	/// 预测，更新先验状态和先验误差协方差
	/// </summary>
	void predict();

	/// <summary>
	/// 校准，计算卡尔曼增益(K_{k}),更新后验状态和后验误差协方差
	/// </summary>
	/// <param name="measurement">观测值</param>
	/// <returns>后验数据</returns>
	Eigen::MatrixXf correct(Eigen::MatrixXf measurement);

	//一系列设置函数
	void setTransitionMatrix(Eigen::MatrixXf transition_matrix);
	void setMeasurementMatrix(Eigen::MatrixXf measurement_matrix);
	void setProcessNoiseCov(Eigen::MatrixXf process_noise_cov);
	void setMeasurementNoiseCov(Eigen::MatrixXf measurement_noise_cov);
	void setStatePre(Eigen::MatrixXf state_pre);
	void setStatePost(Eigen::MatrixXf state_post);
	void setErrorCovPre(Eigen::MatrixXf error_cov_pre);
	void setErrorCovPost(Eigen::MatrixXf error_cov_post);
	/// <summary>
	/// 检查两个矩阵的维度是否一致
	/// </summary>
	/// <param name="matrix1">矩阵1</param>
	/// <param name="matrix2">矩阵2</param>
	/// <returns>true说明一致,flase说明不一致</returns>
	bool checkDimensions(Eigen::MatrixXf matrix1,Eigen::MatrixXf matrix2);

	// Getter for state_post_
	Eigen::MatrixXf getStatePost() const { return state_post_; }
};
