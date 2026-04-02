#include "armor_plate_identification/CameraDriver.hpp"

ArmorCameraCapture::~ArmorCameraCapture()
{
    release();
}

bool ArmorCameraCapture::open()
{
    // ---------- 迈德威视工业相机初始化 ----------
    // 1. 初始化 SDK
    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    // 2. 枚举设备，获取相机列表
    std::vector<tSdkCameraDevInfo> camera_list(4);  // 最多支持4台相机
    INT camera_count = static_cast<INT>(camera_list.size());
    if (CameraEnumerateDevice(camera_list.data(), &camera_count) != CAMERA_STATUS_SUCCESS || camera_count <= 0)
    {
        // 没有找到相机或枚举失败
        return false;  
    }

    // 3. 初始化第一台相机（索引0），获取相机句柄
    if (CameraInit(&camera_list[0], -1, -1, &camera_handle_) != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }
    initialized_ = true;  // 标记已初始化

    // 4. 获取相机能力描述，用于后续判断传感器类型等
    if (CameraGetCapability(camera_handle_, &capability_) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }

    // 5. 判断是否为黑白传感器，并设置输出格式
    const bool is_mono_sensor = capability_.sIspCapacity.bMonoSensor;
    const UINT output_format = is_mono_sensor ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_BGR8;
    if (CameraSetIspOutFormat(camera_handle_, output_format) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }

    // 6. 分配图像缓冲区（最大分辨率 * 通道数）
    const std::size_t max_width = static_cast<std::size_t>(capability_.sResolutionRange.iWidthMax);
    const std::size_t max_height = static_cast<std::size_t>(capability_.sResolutionRange.iHeightMax);
    const std::size_t output_channels = is_mono_sensor ? 1U : 3U;
    rgb_buffer_.resize(max_width * max_height * output_channels);
    if (rgb_buffer_.empty())
    {
        release();
        return false;
    }

    // 7. 开始图像采集
    if (CameraPlay(camera_handle_) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }
    return true;
}

bool ArmorCameraCapture::read(cv::Mat &frame)
{
    // 迈德威视相机：获取原始图像并处理为 RGB/BGR
    tSdkFrameHead frame_info{};   // 帧头信息，包含宽度、高度、格式等
    BYTE *raw_buffer = nullptr;   // 原始图像缓冲区（由 SDK 管理）

    // 1. 获取一帧图像缓冲区（超时 1000ms）
    const CameraSdkStatus buffer_status = CameraGetImageBuffer(camera_handle_, &frame_info, &raw_buffer, 1000);
    if (buffer_status != CAMERA_STATUS_SUCCESS) {
        return false;  // 超时或失败
    }
    // 2. 将原始图像处理为 RGB 格式，存入 rgb_buffer_
    const CameraSdkStatus process_status = CameraImageProcess(camera_handle_, raw_buffer, rgb_buffer_.data(), &frame_info);
    CameraReleaseImageBuffer(camera_handle_, raw_buffer);  // 释放原始缓冲区（必须调用）
    if (process_status != CAMERA_STATUS_SUCCESS) {
        return false;
    }

    // 3. 将 rgb_buffer_ 转换为 OpenCV Mat（克隆一份，避免缓冲区被覆盖）
    const int image_type = frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3;
    frame = cv::Mat(frame_info.iHeight, frame_info.iWidth, image_type, rgb_buffer_.data()).clone();
    if (frame.empty()) {
        return false;
    }

    // 4. 如果是灰度图，转换为三通道 BGR（方便后续统一处理）
    if (image_type == CV_8UC1) {
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }
    return true;
}

void ArmorCameraCapture::release()
{
    if (initialized_)
    {
        CameraUnInit(camera_handle_);  // 反初始化相机
        initialized_ = false;
    }
    rgb_buffer_.clear();  // 释放缓冲区内存
}

bool ArmorCameraCapture::setLowExposureForLightBar(int exposure_us, int gain)
{
    if (!initialized_) {
        return false;
    }
    // 1. 关闭自动曝光，切换到手动模式
    if (CameraSetAeState(camera_handle_, FALSE) != CAMERA_STATUS_SUCCESS) {
        return false;
    }

    // 2. 设置短曝光时间（抑制环境光，突出发光灯条）
    // 灯条本身发光，即使短曝光也能被清晰捕捉
    if (CameraSetExposureTime(camera_handle_, static_cast<double>(exposure_us)) != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    // 3. 设置模拟增益（适当提高增益增强灯条亮度）
    if (CameraSetAnalogGain(camera_handle_, gain) != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    return true;
}

double ArmorCameraCapture::getExposureTime() const
{
    if (!initialized_)
    {
        return -1.0;
    }
    double exposure = 0.0;
    if (CameraGetExposureTime(camera_handle_, &exposure) == CAMERA_STATUS_SUCCESS)
    {
        return exposure;
    }
    return -1.0;
}

int ArmorCameraCapture::getGain() const
{
    if (!initialized_)
    {
        return -1;
    }
    int gain = 0;
    if (CameraGetAnalogGain(camera_handle_, &gain) == CAMERA_STATUS_SUCCESS)
    {
        return gain;
    }
    return -1;
}
