#include "armor_plate_identification/PoseSolver.hpp"
#include <opencv2/calib3d.hpp>

#ifdef DEBUG_POSE
	#include <opencv2/highgui.hpp>
	#include <opencv2/imgproc.hpp>
#endif

PoseSolver::PoseSolver()
    : world_points_(),
      camera_matrix_(),
      distortion_coefficients_(),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_(),
      rvec_(),
      tvec_()
{
}

PoseSolver::PoseSolver(std::vector<cv::Point3f> world_points,
    cv::Mat camera_matrix, 
    cv::Mat distortion_coefficients
)
    : world_points_(std::move(world_points)),
      camera_matrix_(std::move(camera_matrix)),
      distortion_coefficients_(std::move(distortion_coefficients)),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_(camera_matrix_ * rectification_matrix_),
      rvec_(),
      tvec_()
{
}

PoseSolver::PoseSolver(std::vector<cv::Point3f> world_points,
    cv::Mat camera_matrix,
    cv::Mat distortion_coefficients,
    cv::Mat projection_matrix
)
    : world_points_(std::move(world_points)),
      camera_matrix_(std::move(camera_matrix)),
      distortion_coefficients_(std::move(distortion_coefficients)),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_(std::move(projection_matrix)),
      rvec_(),
      tvec_()
{
}

void PoseSolver::solve(const std::vector<cv::Point2f>& camera_points)
{
	cv::solvePnP(world_points_, camera_points, camera_matrix_, distortion_coefficients_, rvec_, tvec_, false, cv::SOLVEPNP_SQPNP);
	cv::Mat r_matrix;
	cv::Rodrigues(rvec_, r_matrix);
	image_points_ = camera_points;
	r_matrix_= r_matrix;
	yaw_ = calculateYaw(tvec_);
	pitch_ = calculatePitch(tvec_);
	distance_ = calculateDistance(tvec_);
}

cv::Point2f PoseSolver::reprojection(cv::Point3f point3D)
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
float calculateYaw(cv::Mat tvec)
{
	// 双向量法：水平瞄准角误差
	// tvec = [tx, ty, tz] 是装甲板中心在相机坐标系中的位置
	// yaw = atan2(tx, tz)：水平面内的方位角（左右偏差）
	double tx = tvec.at<double>(0);
	double tz = tvec.at<double>(2);
	return static_cast<float>(std::atan2(tx, tz) * 180.0 / CV_PI);
}
float calculatePitch(cv::Mat tvec)
{
	// 双向量法：垂直瞄准角误差
	// pitch = atan2(-ty, sqrt(tx^2 + tz^2))：与水平面的夹角（上下偏差）
	double tx = tvec.at<double>(0);
	double ty = tvec.at<double>(1);
	double tz = tvec.at<double>(2);
	double horizontal_dist = std::sqrt(tx * tx + tz * tz);
	return static_cast<float>(std::atan2(-ty, horizontal_dist) * 180.0 / CV_PI);
}
float calculateDistance(cv::Mat tvec)
{
	// 到目标的直线距离
	return static_cast<float>(cv::norm(tvec));
}

#ifdef DEBUG_POSE
	void PoseSolver::drawPose(cv::Mat& image)
	{
		// 写出yaw pitch来。
		cv::putText(image, "yaw: " + std::to_string(yaw_), (image_points_[0] + image_points_[1]) / 2, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
		cv::putText(image, "pitch: " + std::to_string(pitch_), (image_points_[2] + image_points_[3]) / 2, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255), 2);
	}
#endif