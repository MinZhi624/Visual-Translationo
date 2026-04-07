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
	r_matrix_= r_matrix;
	// 计算Yaw、Pitch、距离
	yaw_ = calculateYaw(r_matrix);
	pitch_ = calculatePitch(r_matrix);
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


float calculateYaw(cv::Mat r_matrix)
{
	// 使用相机Z轴在装甲板坐标系中的投影计算Yaw角
	// 避免万向锁：直接从旋转矩阵的第三列（相机Z轴在世界坐标系的方向）计算
	// R = [x_axis, y_axis, z_axis] (相机坐标系在世界坐标系中的表示)
	// 相机看向的Z轴方向为 R * [0,0,1]^T = r_matrix.col(2)
	// 但更直观的是：计算从相机看向目标的水平偏转角
	
	// 方案：使用旋转矩阵的逆（世界到相机的旋转）
	// 或者直接取旋转矩阵的转置（对于正交矩阵，逆=转置）
	cv::Mat R_cam_to_world = r_matrix.t();
	
	// 相机Z轴在世界坐标系中的方向 (第三列)
	double zx = R_cam_to_world.at<double>(0, 2);
	double zz = R_cam_to_world.at<double>(2, 2);
	
	// Yaw: 水平面内投影的方位角
	return static_cast<float>(std::atan2(zx, zz) * 180.0 / CV_PI);
}
float calculatePitch(cv::Mat r_matrix)
{
	// Pitch: 俯仰角，使用相机Z轴在垂直方向的倾斜
	// 同样避免万向锁，直接从旋转矩阵计算
	cv::Mat R_cam_to_world = r_matrix.t();
	
	// 相机Z轴在世界坐标系中的方向 (第三列)
	double zx = R_cam_to_world.at<double>(0, 2);
	double zy = R_cam_to_world.at<double>(1, 2);
	double zz = R_cam_to_world.at<double>(2, 2);
	
	// Pitch: 与水平面的夹角
	double horizontal_dist = std::sqrt(zx * zx + zz * zz);
	return static_cast<float>(std::atan2(-zy, horizontal_dist) * 180.0 / CV_PI);
}
float calculateDistance(cv::Mat t_vector)
{
	// Distance: 平移向量的欧几里得范数（到目标的直线距离）
	return static_cast<float>(cv::norm(t_vector));
}

#ifdef DEBUG_POSE
	void drawPose(cv::Mat& image, float yaw, float pitch, const std::vector<cv::Point2f>& points)
	{
		cv::putText(image, "Yaw: " + std::to_string(yaw), (points[0] + points[1]) / 2, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 255), 1);
        cv::putText(image, "Pitch: " + std::to_string(pitch), (points[2] + points[3]) / 2, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 255), 1);
	}
#endif