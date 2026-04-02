#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "CameraApi.h"

class ArmorCameraCapture
{
public:
    ~ArmorCameraCapture();

    bool open();
    bool read(cv::Mat &frame);
    void release();

    /// @brief 调整相机曝光时间和增益
    /// @param exposure_us 曝光时间，单位微秒
    /// @param gain 增益
    /// @return 是否成功
    bool setLowExposureForLightBar(int exposure_us = 300, int gain = 20);

    double getExposureTime() const;
    int getGain() const;

private:
    CameraHandle camera_handle_{};
    tSdkCameraCapbility capability_{};
    std::vector<unsigned char> rgb_buffer_;
    bool initialized_ = false;
};