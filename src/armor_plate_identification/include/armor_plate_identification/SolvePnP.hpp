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

	/// <summary>
	/// 将三维坐标（相机坐标系）转换为二维坐标（图像坐标系）
	/// </summary>
	/// <param name="point3D">三维坐标点（相机坐标系）</param>
	/// <returns></returns>
	cv::Point2f reprojection(cv::Point3f point3D);

	cv::Mat getTvec() const { return tvec_; };
	cv::Mat getRvec() const { return rvec_; };
	float getDistance();
	float getYaw();
	float getPitch();
};

/// <summary>
/// 最小二乘法拟合球
/// </summary>
/// <param name="points"></param>
/// <returns></returns>
cv::Point3f fitSphereCenter(const std::deque<cv::Point3f>& points);
