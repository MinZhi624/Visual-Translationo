#include "armor_plate_identification/CameraDriver.hpp"

#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

CameraDriver::CameraDriver(rclcpp::Node* node) : node_(node)
{
}

CameraDriver::~CameraDriver()
{
    close();
}

bool CameraDriver::init(const std::string& camera_type, double exposure, double gain)
{
    camera_type_ = camera_type;
    if (camera_type_ == "galaxy") {
        return initGalaxy(exposure, gain);
    } else if (camera_type_ == "mindvision") {
        return initMindVision(exposure, gain);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "未知相机类型: %s", camera_type_.c_str());
        return false;
    }
}

bool CameraDriver::initGalaxy(double exposure, double gain)
{
    GX_STATUS status;
    RCLCPP_INFO(node_->get_logger(), "Starting GalaxyCamera init!");

    do {
        status = GXInitLib();
        if (!GX_SUCCESS(status)) {
            RCLCPP_FATAL(node_->get_logger(), "Init GxIAPI failed, code = %x!", status);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } while (!GX_SUCCESS(status) && rclcpp::ok());

    while (rclcpp::ok()) {
        uint32_t device_count = 0;
        status = GXUpdateDeviceList(&device_count, 100);
        if (device_count < 1) {
            RCLCPP_WARN(node_->get_logger(), "No camera found. device_count = %d", device_count);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        status = GXOpenDeviceByIndex(1, &galaxy_handle_);
        if (!GX_SUCCESS(status)) {
            RCLCPP_ERROR(node_->get_logger(), "Can not open camera, status = %d", status);
        } else {
            break;
        }
    }

    int64_t width = 0, height = 0, width_max = 0, height_max = 0;
    GXGetInt(galaxy_handle_, GX_INT_WIDTH, &width);
    GXGetInt(galaxy_handle_, GX_INT_WIDTH_MAX, &width_max);
    GXGetInt(galaxy_handle_, GX_INT_HEIGHT, &height);
    GXGetInt(galaxy_handle_, GX_INT_HEIGHT_MAX, &height_max);

    int64_t payload_size = 0;
    GXGetInt(galaxy_handle_, GX_INT_PAYLOAD_SIZE, &payload_size);
    galaxy_payload_size_ = payload_size;
    galaxy_bayer_buffer_.resize(galaxy_payload_size_);
    galaxy_rgb_buffer_.reserve(height_max * width_max * 3);

    // Set exposure and gain
    GXSetFloat(galaxy_handle_, GX_FLOAT_EXPOSURE_TIME, exposure);
    GXSetFloat(galaxy_handle_, GX_FLOAT_GAIN, gain);

    GXSendCommand(galaxy_handle_, GX_COMMAND_ACQUISITION_START);

    // Load camera info
    camera_name_ = "narrow_stereo";
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(node_, camera_name_);
    std::string camera_info_url = "package://armor_plate_identification/config/galaxy_camera_info.yaml";
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Galaxy camera initialized: %ld x %ld", width, height);
    return true;
}

bool CameraDriver::initMindVision(double exposure, double gain)
{
    RCLCPP_INFO(node_->get_logger(), "Starting MindVisionCamera init!");

    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "CameraSdkInit failed!");
        return false;
    }

    int camera_count = 1;
    tSdkCameraDevInfo camera_enum_list;
    if (CameraEnumerateDevice(&camera_enum_list, &camera_count) != CAMERA_STATUS_SUCCESS || camera_count == 0) {
        RCLCPP_ERROR(node_->get_logger(), "No MindVision camera found!");
        return false;
    }

    if (CameraInit(&camera_enum_list, -1, -1, &mv_handle_) != CAMERA_STATUS_SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "CameraInit failed!");
        return false;
    }

    CameraGetCapability(mv_handle_, &mv_capability_);

    const bool is_mono = mv_capability_.sIspCapacity.bMonoSensor;
    const UINT output_format = is_mono ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_BGR8;
    CameraSetIspOutFormat(mv_handle_, output_format);

    const std::size_t max_w = static_cast<std::size_t>(mv_capability_.sResolutionRange.iWidthMax);
    const std::size_t max_h = static_cast<std::size_t>(mv_capability_.sResolutionRange.iHeightMax);
    const std::size_t channels = is_mono ? 1U : 3U;
    mv_rgb_buffer_.resize(max_w * max_h * channels);

    // Set exposure and gain
    CameraSetAeState(mv_handle_, false);
    CameraSetExposureTime(mv_handle_, exposure);
    CameraSetAnalogGain(mv_handle_, static_cast<int>(gain));

    CameraPlay(mv_handle_);

    // Load camera info
    camera_name_ = "narrow_stereo";
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(node_, camera_name_);
    std::string camera_info_url = "package://armor_plate_identification/config/mindvision_camera_info.yaml";
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "MindVision camera initialized");
    return true;
}

cv::Mat CameraDriver::Read()
{
    if (!initialized_) return cv::Mat();

    if (camera_type_ == "galaxy") {
        return readGalaxy();
    } else if (camera_type_ == "mindvision") {
        return readMindVision();
    }
    return cv::Mat();
}

cv::Mat CameraDriver::readGalaxy()
{
    GX_FRAME_DATA bayer_frame {};
    bayer_frame.pImgBuf = galaxy_bayer_buffer_.data();

    auto status = GXGetImage(galaxy_handle_, &bayer_frame, 500);
    if (!GX_SUCCESS(status)) {
        RCLCPP_WARN(node_->get_logger(), "Get buffer failed, status = %d", status);
        GXSendCommand(galaxy_handle_, GX_COMMAND_ACQUISITION_STOP);
        GXSendCommand(galaxy_handle_, GX_COMMAND_ACQUISITION_START);
        return cv::Mat();
    }

    DX_PIXEL_COLOR_FILTER bayer_type;
    switch (bayer_frame.nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR8: bayer_type = BAYERGR; break;
        case GX_PIXEL_FORMAT_BAYER_RG8: bayer_type = BAYERRG; break;
        case GX_PIXEL_FORMAT_BAYER_GB8: bayer_type = BAYERGB; break;
        case GX_PIXEL_FORMAT_BAYER_BG8: bayer_type = BAYERBG; break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unsupported Bayer layout: %d!", bayer_frame.nPixelFormat);
            return cv::Mat();
    }

    galaxy_rgb_buffer_.resize(static_cast<size_t>(bayer_frame.nWidth) * bayer_frame.nHeight * 3);
    status = DxRaw8toRGB24(bayer_frame.pImgBuf, galaxy_rgb_buffer_.data(),
                           bayer_frame.nWidth, bayer_frame.nHeight,
                           RAW2RGB_NEIGHBOUR, bayer_type, false);
    if (!GX_SUCCESS(status)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to convert Bayer to RGB, status = %d", status);
        return cv::Mat();
    }

    cv::Mat rgb_img(bayer_frame.nHeight, bayer_frame.nWidth, CV_8UC3, galaxy_rgb_buffer_.data());
    cv::Mat bgr_img;
    cv::cvtColor(rgb_img, bgr_img, cv::COLOR_RGB2BGR);
    return bgr_img.clone();
}

cv::Mat CameraDriver::readMindVision()
{
    tSdkFrameHead frame_info {};
    BYTE* raw_buffer = nullptr;

    auto status = CameraGetImageBuffer(mv_handle_, &frame_info, &raw_buffer, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "Failed to get image buffer, status = %d", status);
        return cv::Mat();
    }

    CameraImageProcess(mv_handle_, raw_buffer, mv_rgb_buffer_.data(), &frame_info);
    CameraReleaseImageBuffer(mv_handle_, raw_buffer);

    const int image_type = frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3;
    cv::Mat frame(frame_info.iHeight, frame_info.iWidth, image_type, mv_rgb_buffer_.data());
    cv::Mat result = frame.clone();

    if (image_type == CV_8UC1) {
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
    }

    return result;
}

void CameraDriver::close()
{
    if (!initialized_) return;

    if (camera_type_ == "galaxy" && galaxy_handle_) {
        GXSendCommand(galaxy_handle_, GX_COMMAND_ACQUISITION_STOP);
        GXCloseDevice(galaxy_handle_);
        GXCloseLib();
        galaxy_handle_ = nullptr;
    } else if (camera_type_ == "mindvision" && mv_handle_) {
        CameraUnInit(mv_handle_);
        mv_handle_ = 0;
    }

    initialized_ = false;
}

sensor_msgs::msg::CameraInfo CameraDriver::getCameraInfo() const
{
    return camera_info_msg_;
}
