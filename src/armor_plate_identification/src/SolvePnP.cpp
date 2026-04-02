#include "armor_plate_identification/SolvePnP.hpp"
#include <opencv2/calib3d.hpp>

SolvePnP::SolvePnP(std::vector<cv::Point2f> image_points)
{
	//初始化
	//装甲板坐标点
	world_points_.push_back(cv::Point3f(-67.5f, -27.5f, 0)); // 0
	world_points_.push_back(cv::Point3f(67.5f, -27.5f, 0)); // 1
	world_points_.push_back(cv::Point3f(-67.5f, 27.5f, 0)); // 2
	world_points_.push_back(cv::Point3f(67.5f, 27.5f, 0)); // 3
	//初始化相机内参
	camera_matrix_ = (cv::Mat_<double>(3, 3) <<
		2374.54248, 0., 698.85288,
		0., 2377.53648, 520.8649,
		0., 0., 1.);
	//相机畸变系数
	distortion_coefficients_ = (cv::Mat_<double>(1, 5) <<
		-0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
	//解算
	cv::solvePnP(world_points_, image_points, camera_matrix_, distortion_coefficients_, rvec_, tvec_);
}

cv::Point2f SolvePnP::reprojection(cv::Point3f point3D)
{
	// 创建3D点矩阵
	cv::Mat point_mat = (cv::Mat_<double>(3, 1) <<
		point3D.x, point3D.y, point3D.z);
	// 投影（齐次坐标）
	cv::Mat pixel_homogeneous = camera_matrix_ * point_mat;
	// 归一化（齐次坐标除以z）
	double z = pixel_homogeneous.at<double>(2);
	double u = pixel_homogeneous.at<double>(0) / z;
	double v = pixel_homogeneous.at<double>(1) / z;
	return cv::Point2f(u, v);
}

float SolvePnP::getDistance()
{
	return cv::norm(tvec_);
}
float SolvePnP::getYaw()
{
	cv::Mat R;
	//解算R矩阵
	cv::Rodrigues(rvec_, R);
	//从旋转矩阵提取欧拉角
	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6;
	// 是否处于万向锁状态
	double z;
	if (!singular) {
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else {
		z = 0;
	}
	return z;
}
float SolvePnP::getPitch()
{
	cv::Mat R;
	//解算R矩阵
	cv::Rodrigues(rvec_, R);
	//从旋转矩阵提取欧拉角
	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	double y;
	y = atan2(-R.at<double>(2, 0), sy);
	return y;
}
