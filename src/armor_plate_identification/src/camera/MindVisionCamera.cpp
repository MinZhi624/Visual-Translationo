#include "armor_plate_identification/camera/MindVisionCamera.hpp"

#include <opencv2/imgproc.hpp>
#include <iostream>

MindVisionCamera::MindVisionCamera(const CameraConfig& config) : config_(config)
{
}

MindVisionCamera::~MindVisionCamera()
{
    close();
}

bool MindVisionCamera::initialize()
{
    std::cerr << "[MindVisionCamera] Starting init!" << std::endl;

    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[MindVisionCamera] CameraSdkInit failed!" << std::endl;
        return false;
    }

    int camera_count = 1;
    tSdkCameraDevInfo camera_enum_list;
    if (CameraEnumerateDevice(&camera_enum_list, &camera_count) != CAMERA_STATUS_SUCCESS || camera_count == 0) {
        std::cerr << "[MindVisionCamera] No camera found!" << std::endl;
        return false;
    }

    if (CameraInit(&camera_enum_list, -1, -1, &handle_) != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[MindVisionCamera] CameraInit failed!" << std::endl;
        return false;
    }

    CameraGetCapability(handle_, &capability_);

    const bool is_mono = capability_.sIspCapacity.bMonoSensor;
    const UINT output_format = is_mono ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_BGR8;
    CameraSetIspOutFormat(handle_, output_format);

    const std::size_t max_w = static_cast<std::size_t>(capability_.sResolutionRange.iWidthMax);
    const std::size_t max_h = static_cast<std::size_t>(capability_.sResolutionRange.iHeightMax);
    const std::size_t channels = is_mono ? 1U : 3U;
    rgb_buffer_.resize(max_w * max_h * channels);

    // Set exposure and gain
    CameraSetAeState(handle_, false);
    CameraSetExposureTime(handle_, config_.exposure);
    CameraSetAnalogGain(handle_, static_cast<int>(config_.gain));

    CameraPlay(handle_);

    // Load camera intrinsics
    if (!config_.camera_info_url.empty()) {
        try {
            intrinsics_ = CameraBase::loadIntrinsicsFromYaml(config_.camera_info_url);
        } catch (const std::exception& e) {
            std::cerr << "[MindVisionCamera] Failed to load camera info: " << e.what() << std::endl;
            return false;
        }
    }

    initialized_ = true;
    std::cerr << "[MindVisionCamera] Initialized" << std::endl;
    return true;
}

Frame MindVisionCamera::read()
{
    if (!initialized_) {
        return Frame{};
    }

    tSdkFrameHead frame_info{};
    BYTE* raw_buffer = nullptr;

    auto status = CameraGetImageBuffer(handle_, &frame_info, &raw_buffer, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[MindVisionCamera] Failed to get image buffer, status = " << status << std::endl;
        return Frame{};
    }

    // 读取到原始图像就打时间戳
    auto timestamp = std::chrono::steady_clock::now();
    // 再处理图像格式问题
    CameraImageProcess(handle_, raw_buffer, rgb_buffer_.data(), &frame_info);
    CameraReleaseImageBuffer(handle_, raw_buffer);

    const int image_type = frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3;
    cv::Mat frame(frame_info.iHeight, frame_info.iWidth, image_type, rgb_buffer_.data());
    cv::Mat result = frame.clone();

    if (image_type == CV_8UC1) {
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
    }

    return Frame{result, timestamp};
}

void MindVisionCamera::close()
{
    if (!initialized_) {
        return;
    }

    if (handle_) {
        CameraUnInit(handle_);
        handle_ = 0;
    }

    initialized_ = false;
}

CameraIntrinsics MindVisionCamera::getIntrinsics() const
{
    return intrinsics_;
}
