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

    /// <summary>
    /// 设置低曝光模式，专为灯条识别优化
    /// 短曝光抑制环境光，突出发光灯条
    /// </summary>
    /// <param name="exposure_us">曝光时间（微秒），默认3000us（3ms）</param>
    /// <param name="gain">模拟增益，默认80</param>
    /// <returns>设置是否成功</returns>
    bool setLowExposureForLightBar(int exposure_us = 3000, int gain = 80);

    /// <summary>
    /// 获取当前曝光时间（微秒）
    /// </summary>
    double getExposureTime() const;

    /// <summary>
    /// 获取当前模拟增益
    /// </summary>
    int getGain() const;

private:
    CameraHandle camera_handle_{};
    tSdkCameraCapbility capability_{};
    std::vector<unsigned char> rgb_buffer_;
    bool initialized_ = false;
};