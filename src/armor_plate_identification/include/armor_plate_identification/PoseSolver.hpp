#pragma once
#include "armor_plate_identification/DetectorArmor.hpp"
#include <armor_plate_interfaces/GimbalData.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>

// 装甲板单位是mm
static constexpr float SMALL_ARMOR_WIDTH = 135;
static constexpr float SMALL_ARMOR_HEIGHT = 55;
static constexpr float LARGE_ARMOR_WIDTH = 225;
static constexpr float LARGE_ARMOR_HEIGHT = 55;

// PNP解算的单位是m
static constexpr double SMALL_HALF_WIDTH = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
static constexpr double SMALL_HALF_HEIGHT = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
static constexpr double LARGE_HALF_WIDTH = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
static constexpr double LARGE_HALF_HEIGHT = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;


// 顺时针左上角是0，以X轴为法向量。x向前，y向左，z向上.
static const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
    cv::Point3f(0, SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT),    // 左上
    cv::Point3f(0, -SMALL_HALF_WIDTH, SMALL_HALF_HEIGHT),   // 右上
    cv::Point3f(0, -SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT),  // 右下
    cv::Point3f(0, SMALL_HALF_WIDTH, -SMALL_HALF_HEIGHT)    // 左下
};

static const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
    cv::Point3f(0, LARGE_HALF_WIDTH, LARGE_HALF_HEIGHT),    // 左上
    cv::Point3f(0, -LARGE_HALF_WIDTH, LARGE_HALF_HEIGHT),   // 右上
    cv::Point3f(0, -LARGE_HALF_WIDTH, -LARGE_HALF_HEIGHT),  // 右下
    cv::Point3f(0, LARGE_HALF_WIDTH, -LARGE_HALF_HEIGHT)    // 左下
};

static const double ARMOR_PITCH_DEGREE = 15.0f;

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

	constexpr static double SEARCH_RANGE = 140.0;

	//===== PNP解算 =====//
	cv::Mat camera_matrix_; 				// 初始化相机内参
	cv::Mat distortion_coefficients_;		// 相机畸变系数
	cv::Mat rectification_matrix_;			// 校正矩阵
	cv::Mat projection_matrix_;				// 投影矩阵
	//===== 坐标系装换 =====//
	Eigen::Matrix3d R_world_gimbal_;
	Eigen::Matrix3d R_gimbal_world_;

	std::unordered_map<int, std::vector<LastArmorYawRecord>> record_;

	static double calculateYawFromRvec(const cv::Mat & rvec);
	static double calculatePitchFromRotation(const Eigen::Matrix3d & R);
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
	void optimizeYaw(DetectorArmor & armor);
	// ===== PNP双重解算 ===== //
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

	void solve(std::vector<DetectorArmor> & armors, const GimbalData & gimbal = GimbalData{});

	cv::Point2f xyzCameraToPixel(cv::Point3f point3D) const;
	cv::Point2f xyzWorldToPixel(Eigen::Vector3d & point3D, const GimbalData & gimbal) const;
	
	std::vector<cv::Point2f> reprojectArmor(
		const Eigen::Vector3d & xyz_world, 
		const double & angle,
		const ArmorType & armor_type);

	float calculateImageDistanceToCenter(const cv::Point2f & target_center_point);
};
