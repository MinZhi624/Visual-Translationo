#include "armor_plate_identification/camera/Camera.hpp"
#include "armor_plate_identification/camera/GalaxyCamera.hpp"
#include "armor_plate_identification/camera/MindVisionCamera.hpp"

#include <opencv2/core/persistence.hpp>
#include <stdexcept>
#include <fstream>

static cv::Mat readMatFromYamlNode(const cv::FileNode& node)
{
    int rows = static_cast<int>(node["rows"]);
    int cols = static_cast<int>(node["cols"]);
    cv::FileNode data_node = node["data"];

    std::vector<double> data;
    data.reserve(static_cast<size_t>(rows * cols));
    for (auto it = data_node.begin(); it != data_node.end(); ++it) {
        data.push_back(static_cast<double>(*it));
    }

    return cv::Mat(rows, cols, CV_64F, data.data()).clone();
}

CameraIntrinsics CameraBase::loadIntrinsicsFromYaml(const std::string& path)
{
    {
        std::ifstream file_check(path);
        if (!file_check.is_open()) {
            throw std::runtime_error("相机信息文件不存在或无法读取: " + path);
        }
    }

    cv::FileStorage fs;
    try {
        fs.open(path, cv::FileStorage::READ);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("相机信息文件格式错误: " + path + "，OpenCV: " + e.what());
    }
    if (!fs.isOpened()) {
        throw std::runtime_error("没有成功打开相机信息文件: " + path);
    }

    CameraIntrinsics intrinsics;
    intrinsics.camera_matrix = readMatFromYamlNode(fs["camera_matrix"]);
    intrinsics.distortion_coefficients = readMatFromYamlNode(fs["distortion_coefficients"]);
    intrinsics.projection_matrix = readMatFromYamlNode(fs["projection_matrix"]);

    return intrinsics;
}

std::unique_ptr<CameraBase> CameraBase::create(const CameraConfig& config)
{
    switch (config.type) {
        case CameraConfig::GALAXY:
            return std::make_unique<GalaxyCamera>(config);
        case CameraConfig::MINDVISION:
            return std::make_unique<MindVisionCamera>(config);
        default:
            return nullptr;
    }
}

Camera::Camera(const CameraConfig& config)
    : impl_(CameraBase::create(config))
{
}

Camera::~Camera()
{
    close();
}

bool Camera::initialize()
{
    if (!impl_) return false;
    return impl_->initialize();
}

Frame Camera::read()
{
    if (!impl_) return Frame{};
    return impl_->read();
}

void Camera::close()
{
    if (impl_) {
        impl_->close();
    }
}

CameraIntrinsics Camera::getIntrinsics() const
{
    if (!impl_) return CameraIntrinsics{};
    return impl_->getIntrinsics();
}
