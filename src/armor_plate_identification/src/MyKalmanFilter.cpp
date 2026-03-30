#include "armor_plate_identification/MyKalmanFilter.hpp"
#include <Eigen/Dense>
#include <iostream>

MyKalmanFilter::MyKalmanFilter(int state_dim, int measure_dim) : state_dim_(state_dim), measure_dim_(measure_dim)
{
	transition_matrix_ = Eigen::MatrixXf::Identity(state_dim,state_dim); //A: 状态转移矩阵 (状态维×状态维)
	measurement_matrix_ = Eigen::MatrixXf::Zero(measure_dim, state_dim); //H: 测量矩阵 (测量维×状态维))
	process_noise_cov_ = Eigen::MatrixXf::Identity(state_dim, state_dim); //Q: 过程噪声协方差 (状态维×状态维)
	measurement_noise_cov_ = Eigen::MatrixXf::Identity(measure_dim, measure_dim); // R: 测量噪声协方差 (测量维×测量维)
	state_pre_ = Eigen::MatrixXf::Zero(state_dim, 1); //先验状态  \hat{x_{k^{-}} }
	state_post_ = Eigen::MatrixXf::Zero(state_dim, 1); //后验状态 \hat{x_{k}}
	error_cov_pre_ = Eigen::MatrixXf::Identity(state_dim, state_dim); //先验误差协方差 \hat{P_{k^{-}} }
	error_cov_post_ = Eigen::MatrixXf::Identity(state_dim, state_dim); //后验误差协方差 \hat{P_{k}}
	kalman_gain_ = Eigen::MatrixXf::Zero(state_dim, measure_dim); //K: 卡尔曼增益 (状态维×测量维)
}
MyKalmanFilter::MyKalmanFilter()
{
	*this = MyKalmanFilter(0, 0);
}
void MyKalmanFilter::predict() 
{
	// 先验状态
	state_pre_ = transition_matrix_ * state_post_;
	// 先验误差协方差
	error_cov_pre_ = transition_matrix_ * error_cov_post_ * transition_matrix_.transpose() + process_noise_cov_;
}

Eigen::MatrixXf MyKalmanFilter::correct(Eigen::MatrixXf measurement)
{
	// 计算 Kalman 增益
	kalman_gain_ = (error_cov_pre_ * measurement_matrix_.transpose()) * (measurement_matrix_ * error_cov_pre_ * measurement_matrix_.transpose() + measurement_noise_cov_).inverse();
	//后验统计
	state_post_ = state_pre_ + kalman_gain_ * (measurement - measurement_matrix_ * state_pre_);
	//后验误差协方差
	error_cov_post_ = (Eigen::MatrixXf::Identity(state_dim_,state_dim_) - kalman_gain_ * measurement_matrix_) * error_cov_pre_;
	return state_post_;
}

void MyKalmanFilter::setTransitionMatrix(Eigen::MatrixXf transition_matrix)
{
	if (checkDimensions(transition_matrix_, transition_matrix)) {
		transition_matrix_ = transition_matrix;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setMeasurementMatrix(Eigen::MatrixXf measurement_matrix)
{
	if (checkDimensions(measurement_matrix_, measurement_matrix)) {
		measurement_matrix_ = measurement_matrix;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setProcessNoiseCov(Eigen::MatrixXf process_noise_cov)
{
	if (checkDimensions(process_noise_cov_, process_noise_cov)) {
		process_noise_cov_ = process_noise_cov;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setMeasurementNoiseCov(Eigen::MatrixXf measurement_noise_cov)
{
	if (checkDimensions(measurement_noise_cov_, measurement_noise_cov)) {
		measurement_noise_cov_ = measurement_noise_cov;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setStatePre(Eigen::MatrixXf state_pre)
{
	if (checkDimensions(state_pre_, state_pre)) {
		state_pre_ = state_pre;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setStatePost(Eigen::MatrixXf state_post)
{
	if (checkDimensions(state_post_, state_post)) {
		state_post_ = state_post;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setErrorCovPre(Eigen::MatrixXf error_cov_pre)
{
	if (checkDimensions(error_cov_pre_, error_cov_pre)) {
		error_cov_pre_ = error_cov_pre;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}
void MyKalmanFilter::setErrorCovPost(Eigen::MatrixXf error_cov_post)
{
	if (checkDimensions(error_cov_post_, error_cov_post)) {
		error_cov_post_ = error_cov_post;
	}
	else {
		std::cout << "维度不匹配" << std::endl;
		exit(0);
	}
}

bool MyKalmanFilter::checkDimensions(Eigen::MatrixXf matrix1, Eigen::MatrixXf matrix2)
{
	if (matrix1.rows() != matrix2.rows() || matrix1.cols() != matrix2.cols()) return false;
	else return true;
}