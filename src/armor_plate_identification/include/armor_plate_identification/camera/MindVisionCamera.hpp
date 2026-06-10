#pragma once

#include "armor_plate_identification/camera/Camera.hpp"

// MindVision SDK
#include "CameraApi.h"

#include <vector>

class MindVisionCamera : public CameraBase
{
public:
    explicit MindVisionCamera(const CameraConfig& config);
    ~MindVisionCamera() override;

    bool initialize() override;
    Frame read() override;
    void close() override;
    CameraIntrinsics getIntrinsics() const override;

private:
    CameraConfig config_;
    CameraIntrinsics intrinsics_;

    CameraHandle handle_ = 0;
    tSdkCameraCapbility capability_{};
    std::vector<unsigned char> rgb_buffer_;

    bool initialized_ = false;
};
