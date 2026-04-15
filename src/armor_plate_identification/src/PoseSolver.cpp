#include "armor_plate_identification/PoseSolver.hpp"
#include <opencv2/calib3d.hpp>

#include <opencv2/imgproc.hpp>

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
	cv::solvePnP(world_points_, camera_points, camera_matrix_, distortion_coefficients_, rvec_, tvec_, false, cv::SOLVEPNP_IPPE);
	cv::Mat r_matrix;
	cv::Rodrigues(rvec_, r_matrix);
	image_points_ = camera_points;
	r_matrix_= r_matrix;
	q_ = calculateQuaternion(r_matrix);
	cv::Point2f target_center_point = (camera_points[0] + camera_points[1] + camera_points[2] + camera_points[3])  / 4;
	image_distance_to_center_ = calculateImageDistanceToCenter(target_center_point);
}
float PoseSolver::calculateImageDistanceToCenter(cv::Point2f target_center_point)
{
	// 计算图像中心到目标中心的距离，通过相机内参来求
	double cx = camera_matrix_.at<double>(0, 2);
	double cy = camera_matrix_.at<double>(1, 2);
	cv::Point2f image_center_point(cx, cy);
	return cv::norm(image_center_point - target_center_point);
}
cv::Mat PoseSolver::undistortImage(const cv::Mat& src) const
{
    cv::Mat dst;
    if (camera_matrix_.empty() || distortion_coefficients_.empty()) {
        return src.clone();
    }
    cv::undistort(src, dst, camera_matrix_, distortion_coefficients_);
    return dst;
}

Eigen::Quaterniond calculateQuaternion(const cv::Mat& R)
{
	Eigen::Matrix3d eigen_R;
	eigen_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
	           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
	           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
	return Eigen::Quaterniond(eigen_R);
}