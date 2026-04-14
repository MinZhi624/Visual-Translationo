#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

class PoseSolver
{
private:
	//===== PNP解算 =====//
	std::vector<cv::Point3f> world_points_;	// 装甲板坐标系点
	cv::Mat camera_matrix_; 				// 初始化相机内参
	cv::Mat distortion_coefficients_;		// 相机畸变系数
	cv::Mat rectification_matrix_;			// 校正矩阵
	cv::Mat projection_matrix_;				// 投影矩阵
	std::vector<cv::Point2f> image_points_;	// 图像上的装甲板四个角点
	// 输出矩阵
	cv::Mat rvec_;					
	cv::Mat tvec_;
	cv::Mat r_matrix_;
	Eigen::Quaterniond q_;
	float image_distance_to_center_;
public:
	
	PoseSolver();
	PoseSolver(std::vector<cv::Point3f> world_points,
		cv::Mat camera_matrix, 
		cv::Mat distortion_coefficients
	);
	PoseSolver(std::vector<cv::Point3f> world_points,
		cv::Mat camera_matrix,
		cv::Mat distortion_coefficients,
		cv::Mat projection_matrix
	);

	/// @brief 解算位姿，计算瞄准角误差
	/// @param camera_points 图像上的装甲板四个角点
	void solve(const std::vector<cv::Point2f>& camera_points);

	/// @brief 将相机的三维坐标系点转换到图像坐标系
	/// @param point3D 相机坐标系三维点
	/// @return 图像坐标系二维点
	cv::Point2f reprojection(cv::Point3f point3D);

	/// @brief 初始化卡尔曼滤波器，恢复到初始状态
	void initKF();
	
	/// @brief 计算图像中心到目标中心的距离（像素）
	/// @param target_center_point 图像中目标中心点坐标
	/// @return 距离（像素）
	float calculateImageDistanceToCenter(cv::Point2f target_center_point);

	cv::Mat getTvec() const { return tvec_; };
	cv::Mat getRvec() const { return rvec_; };
	cv::Mat getRMatrix() const { return r_matrix_; };
	Eigen::Quaterniond getQuaternion() const { return q_; };
	float getImageDistanceToCenter() const { return image_distance_to_center_; }
};

/// @brief 将CV::MAT旋转矩阵转换为EIGEN中四元数
/// @param R cv类型的旋转矩阵
/// @return eigen类型的四元数
Eigen::Quaterniond calculateQuaternion(const cv::Mat& R);