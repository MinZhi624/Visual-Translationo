#include "armor_plate_identification/camera/GalaxyCamera.hpp"

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <thread>

#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

GalaxyCamera::GalaxyCamera(const CameraConfig& config) : config_(config)
{
}

GalaxyCamera::~GalaxyCamera()
{
    close();
}

bool GalaxyCamera::initialize()
{
    GX_STATUS status;
    std::cerr << "[GalaxyCamera] Starting init!" << std::endl;

    do {
        status = GXInitLib();
        if (!GX_SUCCESS(status)) {
            std::cerr << "[GalaxyCamera] Init GxIAPI failed, code = " << status << "!" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } while (!GX_SUCCESS(status));

    while (true) {
        uint32_t device_count = 0;
        status = GXUpdateDeviceList(&device_count, 100);
        if (device_count < 1) {
            std::cerr << "[GalaxyCamera] No camera found. device_count = " << device_count << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        status = GXOpenDeviceByIndex(1, &handle_);
        if (!GX_SUCCESS(status)) {
            std::cerr << "[GalaxyCamera] Can not open camera, status = " << status << std::endl;
        } else {
            break;
        }
    }

    int64_t width = 0, height = 0, width_max = 0, height_max = 0;
    GXGetInt(handle_, GX_INT_WIDTH, &width);
    GXGetInt(handle_, GX_INT_WIDTH_MAX, &width_max);
    GXGetInt(handle_, GX_INT_HEIGHT, &height);
    GXGetInt(handle_, GX_INT_HEIGHT_MAX, &height_max);

    int64_t payload = 0;
    GXGetInt(handle_, GX_INT_PAYLOAD_SIZE, &payload);
    payload_size_ = payload;
    bayer_buffer_.resize(static_cast<size_t>(payload_size_));
    rgb_buffer_.reserve(static_cast<size_t>(height_max * width_max * 3));

    // Set exposure and gain
    GXSetFloat(handle_, GX_FLOAT_EXPOSURE_TIME, config_.exposure);
    GXSetFloat(handle_, GX_FLOAT_GAIN, config_.gain);

    GXSendCommand(handle_, GX_COMMAND_ACQUISITION_START);

    // Load camera intrinsics
    if (!config_.camera_info_url.empty()) {
        try {
            intrinsics_ = CameraBase::loadIntrinsicsFromYaml(config_.camera_info_url);
        } catch (const std::exception& e) {
            std::cerr << "[GalaxyCamera] Failed to load camera info: " << e.what() << std::endl;
            return false;
        }
    }

    initialized_ = true;
    std::cerr << "[GalaxyCamera] Initialized: " << width << " x " << height << std::endl;
    return true;
}

Frame GalaxyCamera::read()
{
    if (!initialized_) {
        return Frame{};
    }

    GX_FRAME_DATA bayer_frame{};
    bayer_frame.pImgBuf = bayer_buffer_.data();

    auto status = GXGetImage(handle_, &bayer_frame, 500);
    if (!GX_SUCCESS(status)) {
        std::cerr << "[GalaxyCamera] Get buffer failed, status = " << status << std::endl;
        GXSendCommand(handle_, GX_COMMAND_ACQUISITION_STOP);
        GXSendCommand(handle_, GX_COMMAND_ACQUISITION_START);
        return Frame{};
    }

    // 读取到原始图像就打时间戳
    auto timestamp = std::chrono::steady_clock::now();

    DX_PIXEL_COLOR_FILTER bayer_type;
    switch (bayer_frame.nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR8: bayer_type = BAYERGR; break;
        case GX_PIXEL_FORMAT_BAYER_RG8: bayer_type = BAYERRG; break;
        case GX_PIXEL_FORMAT_BAYER_GB8: bayer_type = BAYERGB; break;
        case GX_PIXEL_FORMAT_BAYER_BG8: bayer_type = BAYERBG; break;
        default:
            std::cerr << "[GalaxyCamera] Unsupported Bayer layout: " << bayer_frame.nPixelFormat << "!" << std::endl;
            return Frame{};
    }

    rgb_buffer_.resize(static_cast<size_t>(bayer_frame.nWidth) * bayer_frame.nHeight * 3);
    status = DxRaw8toRGB24(
        bayer_frame.pImgBuf, rgb_buffer_.data(),
        bayer_frame.nWidth, bayer_frame.nHeight,
        RAW2RGB_NEIGHBOUR, bayer_type, false);
    if (!GX_SUCCESS(status)) {
        std::cerr << "[GalaxyCamera] Failed to convert Bayer to RGB, status = " << status << std::endl;
        return Frame{};
    }

    cv::Mat rgb_img(bayer_frame.nHeight, bayer_frame.nWidth, CV_8UC3, rgb_buffer_.data());
    cv::Mat bgr_img;
    cv::cvtColor(rgb_img, bgr_img, cv::COLOR_RGB2BGR);

    return Frame{bgr_img.clone(), timestamp};
}

void GalaxyCamera::close()
{
    if (!initialized_) {
        return;
    }

    if (handle_) {
        GXSendCommand(handle_, GX_COMMAND_ACQUISITION_STOP);
        GXCloseDevice(handle_);
        GXCloseLib();
        handle_ = nullptr;
    }

    initialized_ = false;
}

CameraIntrinsics GalaxyCamera::getIntrinsics() const
{
    return intrinsics_;
}
