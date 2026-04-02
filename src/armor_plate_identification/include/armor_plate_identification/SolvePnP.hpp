#pragma once
#include <opencv2/core.hpp>
#include <deque>

class SolvePnP
{
private:
	//装甲板坐标系点
	std::vector<cv::Point3f> world_points_;
	//初始化相机内参
	cv::Mat camera_matrix_;
	//相机畸变系数
	cv::Mat distortion_coefficients_;
	//输出矩阵
	cv::Mat rvec_;
	cv::Mat tvec_;
	//校正矩阵
	cv::Mat rectification_matrix_;
	//投影矩阵
	cv::Mat projection_matrix_;
public:
	
	SolvePnP(std::vector<cv::Point2f> image_points);

	/// @brief 将相机的三维坐标系点转换到图像坐标系
	/// @param point3D 相机坐标系三维点
	/// @return 图像坐标系二维点
	cv::Point2f reprojection(cv::Point3f point3D);

	cv::Mat getTvec() const { return tvec_; };
	cv::Mat getRvec() const { return rvec_; };
	float getDistance();
	float getYaw();
	float getPitch();
};