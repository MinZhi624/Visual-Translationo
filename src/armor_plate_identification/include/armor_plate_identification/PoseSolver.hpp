#pragma once
#include <opencv2/core.hpp>

class PoseSolver
{
private:
	std::vector<cv::Point3f> world_points_;	// 装甲板坐标系点
	cv::Mat camera_matrix_; 				// 初始化相机内参
	cv::Mat distortion_coefficients_;		// 相机畸变系数
	cv::Mat rectification_matrix_;			// 校正矩阵
	cv::Mat projection_matrix_;				// 投影矩阵
	// 输出矩阵
	cv::Mat rvec_;					
	cv::Mat tvec_;
	float yaw_;
	float pitch_;
	float distance_;
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

	/// @brief 解算位姿的主函数，输入相机坐标系的点，得到相机坐标系的位姿同时还有距离和姿态角度
	/// @param camera_points 相机坐标系的点
	void solve(const std::vector<cv::Point2f>& camera_points);

	/// @brief 将相机的三维坐标系点转换到图像坐标系
	/// @param point3D 相机坐标系三维点
	/// @return 图像坐标系二维点
	cv::Point2f reprojection(cv::Point3f point3D);

	cv::Mat getTvec() const { return tvec_; };
	cv::Mat getRvec() const { return rvec_; };
	float getDistance() const { return distance_; };
	float getYaw() const {return yaw_; }
	float getPitch() const { return pitch_; }
};

float calculateYaw(cv::Mat r_matrix);
float calculatePitch(cv::Mat r_matrix);
float calculateDistance(cv::Mat t_vector);