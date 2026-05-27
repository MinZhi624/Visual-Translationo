#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>

class PoseSolver
{
private:
	struct LastArmorYawRecord {
		double yaw = 0.0;
		cv::Point2f center;
	};

	struct PnPCandidate {
		cv::Mat rvec;
		cv::Mat tvec;
		double yaw = 0.0;
		double world_pitch = 0.0;
		double reprojection_error = 0.0;
	};

	//===== PNP解算 =====//
	cv::Mat camera_matrix_; 				// 初始化相机内参
	cv::Mat distortion_coefficients_;		// 相机畸变系数
	cv::Mat rectification_matrix_;			// 校正矩阵
	cv::Mat projection_matrix_;				// 投影矩阵

	std::unordered_map<int, std::vector<LastArmorYawRecord>> record_;

	static double normalizeRadAngle(double rad);
	static double calcYawFromRvec(const cv::Mat & rvec);
	static double calcPitchFromRotation(const Eigen::Matrix3d & R);
	static Eigen::Matrix3d calcRWorldGimbal(double yaw_abs, double pitch_abs);
	static double calcWorldPitchFromRvec(const cv::Mat & rvec, double yaw_abs, double pitch_abs);
	static double calcReprojectionError(
		const std::vector<cv::Point3f> & object_points,
		const std::vector<cv::Point2f> & image_points,
		const cv::Mat & camera_matrix,
		const cv::Mat & distortion_coefficients,
		const cv::Mat & rvec,
		const cv::Mat & tvec
	);
	std::vector<PnPCandidate> createPnPCandidates(
		const std::vector<cv::Point3f> & object_points,
		const std::vector<cv::Point2f> & image_points,
		double yaw_abs,
		double pitch_abs
	) const;
	static size_t selectByGeometry(const std::vector<PnPCandidate> & candidates);
	static size_t selectByYawContinuity(const std::vector<PnPCandidate> & candidates, double nearest_yaw);
	size_t selectBestCandidate(
		const std::vector<PnPCandidate> & candidates,
		int armor_name_key,
		const cv::Point2f & target_center
	) const;
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

	void solve(std::vector<DetectorArmor> & armors, float yaw_abs = 0.0f, float pitch_abs = 0.0f);

	cv::Point2f xyzCameraToPixel(cv::Point3f point3D) const;

	float calculateImageDistanceToCenter(const cv::Point2f & target_center_point);
};
