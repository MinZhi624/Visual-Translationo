#pragma once

#include "armor_plate_identification/camera/Camera.hpp"

// Galaxy SDK
#include "GxIAPI.h"
#include "DxImageProc.h"

#include <vector>

class GalaxyCamera : public CameraBase
{
public:
    explicit GalaxyCamera(const CameraConfig& config);
    ~GalaxyCamera() override;

    bool initialize() override;
    Frame read() override;
    void close() override;
    CameraIntrinsics getIntrinsics() const override;

private:
    CameraConfig config_;
    CameraIntrinsics intrinsics_;

    GX_DEV_HANDLE handle_ = nullptr;
    int64_t payload_size_ = 0;
    std::vector<char> bayer_buffer_;
    std::vector<unsigned char> rgb_buffer_;

    bool initialized_ = false;
};
