#include "armor_plate_identification/PoseSolver.hpp"
#include "armor_plate_common/angle.hpp"
#include "armor_plate_common/transform.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

static constexpr double SAME_ARMOR_CENTER_THRESH = 30.0;
static constexpr double YAW_MUTATION_THRESH = M_PI_2;
static constexpr double REPROJECTION_ERROR_MARGIN = 3.0;
static constexpr double MIN_VALID_ARMOR_PITCH_WORLD = -0.05;

PoseSolver::PoseSolver()
    : camera_matrix_(),
      distortion_coefficients_(),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_()
{
}

PoseSolver::PoseSolver(
    cv::Mat camera_matrix,
    cv::Mat distortion_coefficients
)
    : camera_matrix_(std::move(camera_matrix)),
      distortion_coefficients_(std::move(distortion_coefficients)),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_(camera_matrix_ * rectification_matrix_)
{
}

PoseSolver::PoseSolver(
    cv::Mat camera_matrix,
    cv::Mat distortion_coefficients,
    cv::Mat projection_matrix
)
    : camera_matrix_(std::move(camera_matrix)),
      distortion_coefficients_(std::move(distortion_coefficients)),
      rectification_matrix_(cv::Mat::eye(3, 3, CV_64F)),
      projection_matrix_(std::move(projection_matrix))
{
}

void PoseSolver::solve(std::vector<DetectorArmor> & armors, const GimbalData & gimbal)
{
    std::unordered_map<int, std::vector<LastArmorYawRecord>> new_record;
    for (auto & armor : armors) {
        const auto& world_points = (armor.type_ == ArmorType::LARGE)
            ? LARGE_ARMOR_POINTS
            : SMALL_ARMOR_POINTS;
        cv::Point2f target_center = (armor.points_[0] + armor.points_[1] + armor.points_[2] + armor.points_[3]) / 4;

        const int armor_name_key = static_cast<int>(armor.name_);
        std::vector<PnPCandidate> candidates = createPnPCandidates(world_points, armor.points_, gimbal);
        size_t best_id = selectBestCandidate(candidates, armor_name_key, target_center);

        cv::Mat rmat;
        cv::Rodrigues(candidates[best_id].rvec, rmat);
        
        Eigen::Matrix3d R_camrea_armor;
        Eigen::Vector3d t_camera; 
        cv::cv2eigen(rmat, R_camrea_armor);
        cv::cv2eigen(candidates[best_id].tvec, t_camera);

        armor.xyz_camera_ = t_camera;
        armor.q_camera_ = Eigen::Quaterniond(R_camrea_armor);

        armor.image_distance_to_center_ = calculateImageDistanceToCenter(target_center);

        new_record[armor_name_key].push_back({candidates[best_id].yaw, target_center});
    }
    record_ = std::move(new_record);
}

cv::Point2f PoseSolver::xyzCameraToPixel(cv::Point3f point3D) const
{
    if (point3D.z <= 1e-6f) {
        return cv::Point2f(-1.0f, -1.0f);
    }
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);
    double inv_z = 1.0 / point3D.z;
    double u = fx * point3D.x * inv_z + cx;
    double v = fy * point3D.y * inv_z + cy;
    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}
cv::Point2f PoseSolver::xyzWorldToPixel(Eigen::Vector3d & point3D, const GimbalData & gimbal) const
{
    const Eigen::Matrix3d R_world_gimbal = armor_plate_common::calculateRWorldGimbal(gimbal.yaw_abs, gimbal.pitch_abs);
    const Eigen::Matrix3d R_world_camera = R_world_gimbal * armor_plate_common::R_GIMBAL_CAMERA;
    const Eigen::Matrix3d R_camera_world = R_world_camera.transpose();
    Eigen::Vector3d point_camrea = R_camera_world * point3D;
    cv::Point3d point_camrea_cv = {point_camrea.x(), point_camrea.y(), point_camrea.z()};
    return xyzCameraToPixel(point_camrea_cv);
}

float PoseSolver::calculateImageDistanceToCenter(const cv::Point2f & target_center_point)
{
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);
    cv::Point2f image_center_point(cx, cy);
    return cv::norm(image_center_point - target_center_point);
}

std::vector<PoseSolver::PnPCandidate> PoseSolver::createPnPCandidates(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    const GimbalData & gimbal) const
{
    std::vector<cv::Mat> ippe_rvecs;
    std::vector<cv::Mat> ippe_tvecs;
    cv::solvePnPGeneric(object_points, image_points, camera_matrix_, distortion_coefficients_,
                        ippe_rvecs, ippe_tvecs, false, cv::SOLVEPNP_IPPE);

    std::vector<PnPCandidate> candidates;
    candidates.reserve(ippe_rvecs.size());
    for (size_t i = 0; i < ippe_rvecs.size(); ++i) {
        PnPCandidate candidate;
        candidate.rvec = ippe_rvecs[i];
        candidate.tvec = ippe_tvecs[i];
        candidate.yaw = calculateYawFromRvec(candidate.rvec);
        candidate.world_pitch = calculateWorldPitchFromRvec(candidate.rvec, gimbal);
        candidate.reprojection_error = calculateReprojectionError(
            object_points, image_points, camera_matrix_, distortion_coefficients_,
            candidate.rvec, candidate.tvec);
        candidates.push_back(candidate);
    }
    // 如果IPPE没有解，就用SOLVEPNP_ITERATIVE
    if (candidates.empty()) {
        PnPCandidate candidate;
        cv::solvePnP(object_points, image_points, camera_matrix_, distortion_coefficients_,
                     candidate.rvec, candidate.tvec, false, cv::SOLVEPNP_ITERATIVE);
        candidate.yaw = calculateYawFromRvec(candidate.rvec);
        candidate.world_pitch = calculateWorldPitchFromRvec(candidate.rvec, gimbal);
        candidate.reprojection_error = calculateReprojectionError(
            object_points, image_points, camera_matrix_, distortion_coefficients_,
            candidate.rvec, candidate.tvec);
        candidates.push_back(candidate);
    }

    return candidates;
}

size_t PoseSolver::selectByGeometry(const std::vector<PnPCandidate> & candidates)
{
    size_t best_id = 0;
    for (size_t i = 1; i < candidates.size(); ++i) {
        const bool candidate_pitch_valid = candidates[i].world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;
        const bool best_pitch_valid = candidates[best_id].world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;

        if (candidate_pitch_valid && !best_pitch_valid) {
            best_id = i;
        } else if (!candidate_pitch_valid && best_pitch_valid) {
            // 保留当前 best_id，不做任何操作
        } else {
            if (candidates[i].reprojection_error < candidates[best_id].reprojection_error) {
                best_id = i;
            }
        }
    }
    return best_id;
}

size_t PoseSolver::selectByYawContinuity(const std::vector<PnPCandidate> & candidates, double nearest_yaw)
{
    const bool has_valid_pitch = std::any_of(
        candidates.begin(), candidates.end(), [](const PnPCandidate & c) {
            return c.world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;
        });

    size_t continuous_id = 0;
    double min_yaw_delta = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (has_valid_pitch && candidates[i].world_pitch < MIN_VALID_ARMOR_PITCH_WORLD) {
            continue;
        }
        const double yaw_delta = std::abs(armor_plate_common::normalizeRadAngle(candidates[i].yaw - nearest_yaw));
        if (yaw_delta < min_yaw_delta) {
            min_yaw_delta = yaw_delta;
            continuous_id = i;
        }
    }
    return continuous_id;
}

size_t PoseSolver::selectBestCandidate(
    const std::vector<PnPCandidate> & candidates,
    int armor_name_key,
    const cv::Point2f & target_center) const
{
    // 纯几何最优（当前帧 pitch + 重投影误差）
    size_t best_id = selectByGeometry(candidates);

    // 查找同编号的历史记录
    auto group_it = record_.find(armor_name_key);
    if (group_it == record_.end() || candidates.size() < 2) {
        return best_id;
    }

    size_t nearest_idx = group_it->second.size();
    double min_center_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < group_it->second.size(); ++i) {
        double dist = cv::norm(target_center - group_it->second[i].center);
        if (dist < SAME_ARMOR_CENTER_THRESH && dist < min_center_dist) {
            min_center_dist = dist;
            nearest_idx = i;
        }
    }
    if (nearest_idx >= group_it->second.size()) {
        return best_id;
    }
    const auto& nearest_record = group_it->second[nearest_idx];

    //  历史连续性最优
    size_t continuous_id = selectByYawContinuity(candidates, nearest_record.yaw);

    // 4. 仲裁：几何最优发生了 yaw 突变，且连续性解误差没差太多时，修正为连续性解
    const double best_yaw_delta =
        std::abs(armor_plate_common::normalizeRadAngle(candidates[best_id].yaw - nearest_record.yaw));
    const double error_margin =
        candidates[continuous_id].reprojection_error - candidates[best_id].reprojection_error;

    if (best_yaw_delta > YAW_MUTATION_THRESH && error_margin < REPROJECTION_ERROR_MARGIN) {
        best_id = continuous_id;
    }

    return best_id;
}

// ========== 工具类 ==========

double PoseSolver::calculateYawFromRvec(const cv::Mat & rvec)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    Eigen::Matrix3d R;
    cv::cv2eigen(rmat, R);
    Eigen::Quaterniond q(R);

    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

double PoseSolver::calculatePitchFromRotation(const Eigen::Matrix3d & R)
{
    Eigen::Quaterniond q(R);
    const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    return (std::abs(sinp) >= 1.0) ? std::copysign(M_PI_2, sinp) : std::asin(sinp);
}

double PoseSolver::calculateWorldPitchFromRvec(const cv::Mat & rvec, const GimbalData & gimbal)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    Eigen::Matrix3d R_camera_armor;
    cv::cv2eigen(rmat, R_camera_armor);

    Eigen::Matrix3d R_world_armor =
        armor_plate_common::calculateRWorldGimbal(gimbal.yaw_abs, gimbal.pitch_abs)
        * armor_plate_common::R_GIMBAL_CAMERA * R_camera_armor;
    return calculatePitchFromRotation(R_world_armor);
}

double PoseSolver::calculateReprojectionError(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const cv::Mat & rvec,
    const cv::Mat & tvec)
{
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coefficients, projected_points);

    double error = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        error += cv::norm(image_points[i] - projected_points[i]);
    }
    return error;
}
