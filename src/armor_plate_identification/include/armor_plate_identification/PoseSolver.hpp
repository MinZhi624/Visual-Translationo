#pragma once
#include "armor_plate_identification/Armor.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

class PoseSolver
{
private:
	//===== PNP解算 =====//
	cv::Mat camera_matrix_; 				// 初始化相机内参
	cv::Mat distortion_coefficients_;		// 相机畸变系数
	cv::Mat rectification_matrix_;			// 校正矩阵
	cv::Mat projection_matrix_;				// 投影矩阵
public:

	PoseSolver();
	PoseSolver(
		cv::Mat camera_matrix,
		cv::Mat distortion_coefficients
	);
	PoseSolver(
		cv::Mat camera_matrix,
		cv::Mat distortion_coefficients,
		cv::Mat projection_matrix
	);

	void solve(Armor & armor);

	cv::Point2f xyzCameraToPixel(cv::Point3f point3D) const;

	float calculateImageDistanceToCenter(const cv::Point2f & target_center_point);
};
