#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <memory>
#include <string>

// Galaxy SDK
#include "GxIAPI.h"
#include "DxImageProc.h"

// MindVision SDK
#include "CameraApi.h"

class CameraDriver {
public:
    CameraDriver() = default;
    explicit CameraDriver(rclcpp::Node* node);
    ~CameraDriver();

    bool init(const std::string& camera_type, double exposure, double gain);
    cv::Mat Read();
    void close();
    sensor_msgs::msg::CameraInfo getCameraInfo() const;

private:
    rclcpp::Node* node_;
    std::string camera_type_;
    
    // Camera info
    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    
    // Galaxy
    GX_DEV_HANDLE galaxy_handle_ = nullptr;
    int64_t galaxy_payload_size_ = 0;
    std::vector<char> galaxy_bayer_buffer_;
    std::vector<unsigned char> galaxy_rgb_buffer_;
    
    // MindVision
    CameraHandle mv_handle_ = 0;
    tSdkCameraCapbility mv_capability_{};
    std::vector<unsigned char> mv_rgb_buffer_;
    
    bool initialized_ = false;

    bool initGalaxy(double exposure, double gain);
    bool initMindVision(double exposure, double gain);
    cv::Mat readGalaxy();
    cv::Mat readMindVision();
};
