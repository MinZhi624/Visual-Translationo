#include "armor_plate_identification/PoseSolver.hpp"
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/logging.hpp"


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

void PoseSolver::solve(Armor & armor)
{
    const auto& world_points = (armor.type_ == ArmorType::LARGE)
        ? LARGE_ARMOR_POINTS
        : SMALL_ARMOR_POINTS;
    cv::Mat rvec, tvec;
    cv::solvePnP(world_points, armor.points_, camera_matrix_, distortion_coefficients_,rvec, tvec, false, cv::SOLVEPNP_IPPE);

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    
    Eigen::Matrix3d R_camrea_armor;
    Eigen::Vector3d t_camera; 
    cv::cv2eigen(rmat, R_camrea_armor);
    cv::cv2eigen(tvec, t_camera);

    armor.xyz_camera_ = t_camera;
    armor.q_camera_ = Eigen::Quaterniond(R_camrea_armor);

    cv::Point2f target_center = (armor.points_[0] + armor.points_[1] + armor.points_[2] + armor.points_[3]) / 4;
    armor.image_distance_to_center_ = calculateImageDistanceToCenter(target_center);
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
float PoseSolver::calculateImageDistanceToCenter(const cv::Point2f & target_center_point)
{
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);
    cv::Point2f image_center_point(cx, cy);
    return cv::norm(image_center_point - target_center_point);
}


