#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include <armor_plate_identification/yaw/IYawSearchObserver.hpp>
#include <armor_plate_interfaces/ArmorPose.hpp>
#include <armor_plate_interfaces/GimbalData.hpp>
#include <armor_plate_interfaces/armor_geometry.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <unordered_map>

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
	//===== 坐标系装换 =====//
	Eigen::Matrix3d R_world_gimbal_;
	Eigen::Matrix3d R_gimbal_world_;

	std::unordered_map<int, std::vector<LastArmorYawRecord>> record_;

	static double calculateWorldPitchFromRvec(const cv::Mat & rvec, const GimbalData & gimbal);
	double calculateReprojectionError(
		const std::vector<cv::Point3f> & object_points,
		const std::vector<cv::Point2f> & image_points,
		const cv::Mat & rvec = cv::Mat(),
		const cv::Mat & tvec = cv::Mat()
	) const;

	double calculateReprojectionError(
		const DetectorArmor & armor,
		const double & yaw
	);
	std::vector<PnPCandidate> createPnPCandidates(
		const std::vector<cv::Point3f> & object_points,
		const std::vector<cv::Point2f> & image_points,
		const GimbalData & gimbal
	) const;
	// ===== pitch单自由度锁定 ===== //
	void optimizeYaw(DetectorArmor & armor, std::size_t armor_index);
	IYawSearchObserver* yaw_observer_ = nullptr;
	// ===== PNP双重解算 ===== //
	static size_t selectByGeometry(const std::vector<PnPCandidate> & candidates);
	static size_t selectByYawContinuity(const std::vector<PnPCandidate> & candidates, double nearest_yaw);
	size_t selectBestCandidate(
		const std::vector<PnPCandidate> & candidates,
		int armor_name_key,
		const cv::Point2f & targetd_center
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

	void solve(std::vector<DetectorArmor> & armors, const GimbalData & gimbal = GimbalData{});

	void setYawSearchObserver(IYawSearchObserver* observer) { yaw_observer_ = observer; }

	cv::Point2f xyzCameraToPixel(cv::Point3f point3D) const;
	cv::Point2f xyzWorldToPixel(Eigen::Vector3d & point3D, const GimbalData & gimbal) const;

	std::vector<cv::Point2f> reprojectArmor(const ArmorPose & armor_pose) const;
	std::vector<cv::Point2f> reprojectArmor(const ArmorPose & armor_pose, const GimbalData & gimbal) const;

	float calculateImageDistanceToCenter(const cv::Point2f & target_center_point);

private:
	std::vector<cv::Point2f> reprojectArmorImpl(
		const ArmorPose & armor_pose,
		const Eigen::Matrix3d & R_gimbal_world) const;
};
