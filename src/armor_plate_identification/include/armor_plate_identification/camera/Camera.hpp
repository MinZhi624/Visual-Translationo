#pragma once

#include <opencv2/core.hpp>
#include <chrono>
#include <memory>
#include <string>

struct Frame
{
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
};

struct CameraIntrinsics
{
    cv::Mat camera_matrix;            // 3x3, CV_64F
    cv::Mat distortion_coefficients;  // 1x5, CV_64F
    cv::Mat projection_matrix;        // 3x4, CV_64F
};

struct CameraConfig
{
    enum Type { GALAXY, MINDVISION } type = Type::GALAXY;
    double exposure = 3500.0;     
    double gain = 1.0;            
    std::string camera_info_url;  
};

/** @brief 相机基类*/
class CameraBase
{
public:
    virtual ~CameraBase() = default;

    virtual bool initialize() = 0;
    virtual Frame read() = 0;
    virtual void close() = 0;
    virtual CameraIntrinsics getIntrinsics() const = 0;

    static std::unique_ptr<CameraBase> create(const CameraConfig& config);
    static CameraIntrinsics loadIntrinsicsFromYaml(const std::string& path);
};

/** @brief 相机管理类：值语义包装，内部自动创建对应 CameraBase 实现 */
class Camera {
public:
    Camera() = default;
    explicit Camera(const CameraConfig& config);
    ~Camera();

    // 存在unique_ptr 的对象，不能拷贝构造和赋值，只能移动构造和赋值 
    Camera(Camera&&) = default;
    Camera& operator=(Camera&&) = default;

    bool initialize();
    Frame read();
    void close();
    CameraIntrinsics getIntrinsics() const;

private:
    std::unique_ptr<CameraBase> impl_;
};
