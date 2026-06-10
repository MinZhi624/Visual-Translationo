#include "armor_plate_identification/ArmorPlateIdentification.hpp"

void ArmorPlateIdentification::run()
{
    if (!headless_) {
        gui_worker_.start();
    }

    while (rclcpp::ok()) {
        Frame frame;
        if (!frame_queue_.pop(frame, std::chrono::milliseconds(100))) continue;
        read_stamp_ = convertSteadyToRosTime(frame.timestamp);

        img_show_ = frame.img;
        debug_base_.onFrameStart();

        identification(frame.img);
        solvePose();
        publish();
        save();
        show();

        debug_base_.onFrameEnd();
        KeyEvent event = headless_ ? KeyEvent{} : gui_worker_.consumeKey();
        if (control(event)) break;
    }
    gui_worker_.stop();
}

bool ArmorPlateIdentification::control(const KeyEvent& event)
{
    debug_base_.control(event);

    if (event.action == KeyAction::Exit) {
        return true;
    }

    if (event.action == KeyAction::Pause && !headless_) {
        RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
        while (rclcpp::ok()) {
            auto pause_event = gui_worker_.consumeKey();
            if (pause_event.action != KeyAction::None) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (debug_base_.isDebugTimeControl()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(debug_base_.getDelayTimeMs()));
    }

    return false;
}

void ArmorPlateIdentification::trackerDebugCallBack(const TrackerDebug::SharedPtr msg)
{
    tracker_debug_queue_.push(msg);
}

void ArmorPlateIdentification::processTrackerDebug(const TrackerDebug::SharedPtr msg)
{
    Record rec;
    Record matched;
    bool found = false;
    while (img_queue_.pop(rec, std::chrono::milliseconds(0))) {
        if (rec.img_stamp == msg->header.stamp) {
            matched = rec;
            found = true;
            break;
        }
    }
    if (!found) return;

    cv::Mat debug_img = matched.img.clone();

    debug_tracker_.drawTragetPoints(debug_img, *msg, matched.gimbal);
    debug_tracker_.drawPredictedCar(debug_img, *msg, matched.gimbal);

    if (!headless_) {
        debug_tracker_.pushTrackerDebugFrame(debug_img);
    }
}

void ArmorPlateIdentification::trackerDebugWorker()
{
    while (rclcpp::ok() && tracker_debug_worker_running_) {
        TrackerDebug::SharedPtr msg;
        if (!tracker_debug_queue_.pop(msg, std::chrono::milliseconds(100))) continue;
        if (msg) processTrackerDebug(msg);
    }
}

void ArmorPlateIdentification::stopTrackerDebugWorker()
{
    tracker_debug_worker_running_ = false;
    if (tracker_debug_thread_.joinable()) tracker_debug_thread_.join();
}

void ArmorPlateIdentification::cameraCaptureWorker()
{
    int fail_count = 0;
    while (camera_capture_running_.load() && rclcpp::ok()) {
        auto frame = camera_.read();
        if (frame.img.empty()) {
            fail_count++;
            if (fail_count > 5) {
                RCLCPP_FATAL(this->get_logger(), "Camera read failed!");
                rclcpp::shutdown();
                break;
            }
            continue;
        }
        fail_count = 0;
        frame_queue_.push(frame);
    }
}

void ArmorPlateIdentification::stopCameraCaptureWorker()
{
    camera_capture_running_ = false;
    camera_.close();
    if (camera_capture_thread_.joinable()) {
        camera_capture_thread_.join();
    }
}

void ArmorPlateIdentification::init()
{
    target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");

    initDetector();

    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", rclcpp::SensorDataQoS());

    gimbal_angle_sub_ = this->create_subscription<GimbalAngle>(
        "gimbal_angle", rclcpp::SensorDataQoS(),
        [this](const GimbalAngle::SharedPtr msg) {
            GimbalRecord rec;
            rec.stamp = msg->stamp;
            rec.data.yaw_abs = msg->yaw_abs;
            rec.data.pitch_abs = msg->pitch_abs;
            gimbal_queue_.push(rec);
        }
    );

    std::string camera_type = this->declare_parameter<std::string>("camera_type", "galaxy");
    double exposure_time = this->declare_parameter<double>("exposure_time", 3500.0);
    double gain = this->declare_parameter<double>("gain", 1.0);

    std::string package_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    std::string camera_info_url;
    if (camera_type == "galaxy") {
        camera_info_url = package_dir + "/" + this->declare_parameter<std::string>("galaxy_camera_info_url", "config/galaxy_camera_info.yaml");
    } else {
        camera_info_url = package_dir + "/" + this->declare_parameter<std::string>("mindvision_camera_info_url", "config/mindvision_camera_info.yaml");
    }

    CameraConfig config;
    config.type = (camera_type == "galaxy") ? CameraConfig::GALAXY : CameraConfig::MINDVISION;
    config.exposure = exposure_time;
    config.gain = gain;
    config.camera_info_url = camera_info_url;

    camera_ = Camera(config);
    if (!camera_.initialize()) {
        RCLCPP_FATAL(this->get_logger(), "相机初始化失败，程序退出");
        rclcpp::shutdown();
        return;
    }

    camera_capture_running_ = true;
    camera_capture_thread_ = std::thread(&ArmorPlateIdentification::cameraCaptureWorker, this);

    initPoseSolver();
    initDebug();

    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&ArmorPlateIdentification::trackerDebugCallBack, this, std::placeholders::_1)
    );
    tracker_debug_worker_running_ = true;
    tracker_debug_thread_ = std::thread(&ArmorPlateIdentification::trackerDebugWorker, this);

    RCLCPP_INFO(this->get_logger(), "识别节点已启动，相机类型: %s", camera_type.c_str());
    RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停");
    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
}

void ArmorPlateIdentification::identification(cv::Mat& img_bgr)
{
    cv::Mat img_thre = lights_.preprocess(img_bgr);
    debug_base_.mark("preprocess");

    lights_.detectArmors(img_thre, img_bgr);
    GuiWorker::drawArmors(img_show_, lights_.getArmors());
    debug_base_.mark("detectArmors");

    armors_ = lights_.getArmors();

    if (debug_base_.isRecordingRois()) {
        debug_base_.feedRejected(lights_.getRejectedNumberRois());
    }

    debug_base_.debugLights(lights_.getLights());
    debug_base_.debugNumberClassification(lights_.getArmors());
    debug_base_.debugPreprocessing(img_bgr, lights_.getPreprocessDebug());
}

void ArmorPlateIdentification::solvePose()
{
    auto to_ns = [](const auto& s) { return (int64_t)s.sec * 1000000000LL + s.nanosec; };
    int64_t image_ns = to_ns(read_stamp_);

    GimbalRecord rec;
    while (gimbal_queue_.pop(rec, std::chrono::milliseconds(0))) {
        if (!gimbal_has_data_) {
            gimbal_behind_ = rec;
            gimbal_has_data_ = true;
            continue;
        }
        int64_t rec_ns = to_ns(rec.stamp);
        if (rec_ns <= image_ns) {
            gimbal_ahead_ = gimbal_behind_;
            gimbal_behind_ = rec;
        } else {
            gimbal_ahead_ = gimbal_behind_;
            gimbal_behind_ = rec;
            break;
        }
    }

    GimbalData gimbal;
    if (gimbal_has_data_) {
        int64_t t_a = to_ns(gimbal_ahead_.stamp);
        int64_t t_b = to_ns(gimbal_behind_.stamp);
        if (t_b > t_a && image_ns >= t_a && image_ns <= t_b) {
            float k = static_cast<float>(image_ns - t_a) / static_cast<float>(t_b - t_a);
            gimbal.yaw_abs = gimbal_ahead_.data.yaw_abs + k * (gimbal_behind_.data.yaw_abs - gimbal_ahead_.data.yaw_abs);
            gimbal.pitch_abs = gimbal_ahead_.data.pitch_abs + k * (gimbal_behind_.data.pitch_abs - gimbal_ahead_.data.pitch_abs);
        } else {
            gimbal.yaw_abs = gimbal_behind_.data.yaw_abs;
            gimbal.pitch_abs = gimbal_behind_.data.pitch_abs;
        }
    } else {
        // 回退到最新值
        gimbal.yaw_abs = matched_gimbal_.yaw_abs;
        gimbal.pitch_abs = matched_gimbal_.pitch_abs;
    }

    matched_gimbal_ = gimbal;
    pose_solver_.solve(armors_, matched_gimbal_);
}

void ArmorPlateIdentification::publish()
{
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = read_stamp_;
    armor_plates_msg.header.frame_id = "camera_link";
    armor_plates_msg.armor_plates.reserve(armors_.size());
    for (const auto& armor : armors_) {
        ArmorPlate armor_plate;
        armor_plate.x_world = armor.xyz_world_.x();
        armor_plate.y_world = armor.xyz_world_.y();
        armor_plate.z_world = armor.xyz_world_.z();
        armor_plate.yaw_world = armor.ypr_world_.x();
        armor_plate.number = static_cast<int>(armor.name_);
        armor_plate.image_distance_to_center = armor.image_distance_to_center_;
        armor_plates_msg.armor_plates.push_back(armor_plate);
    }
    armor_plates_msg.gimbal_yaw_abs = matched_gimbal_.yaw_abs;
    armor_plates_msg.gimbal_pitch_abs = matched_gimbal_.pitch_abs;
    armor_plates_pub_->publish(armor_plates_msg);
}

void ArmorPlateIdentification::save()
{
    debug_base_.save();

    img_queue_.push({read_stamp_, img_show_, matched_gimbal_});
}

void ArmorPlateIdentification::show()
{
    debug_base_.draw(img_show_);

    if (!headless_) {
        cv::Mat show_img;
        cv::resize(img_show_, show_img, cv::Size(), 0.5, 0.5);
        gui_worker_.pushFrame(DebugWindow::IDENTIFICATION, show_img);
    }

    debug_base_.show();
    auto frames = debug_base_.getDisplayFrames();
    for (const auto& [name, img] : frames) {
        gui_worker_.pushFrame(name, img);
    }
}

ArmorPlateIdentification::ArmorPlateIdentification() : Node("armor_plate_identification_node")
{
    init();
}

ArmorPlateIdentification::~ArmorPlateIdentification()
{
    stopTrackerDebugWorker();
    stopCameraCaptureWorker();
    gui_worker_.stop();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorPlateIdentification>();
    std::thread spin_thread([&]() { rclcpp::spin(node); });
    node->run();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

void ArmorPlateIdentification::initDebug()
{
    DebugBaseParams base_params;
    base_params.debug_timecontrol_ = this->declare_parameter<bool>("debug_timecontrol", false);
    base_params.debug_lights_ = this->declare_parameter<bool>("debug_lights", false);
    base_params.debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    base_params.debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    base_params.delay_time = this->declare_parameter<int>("delay_time", 0);
    base_params.stats_interval = this->declare_parameter<int>("stats_interval", 50);

    headless_ = this->declare_parameter<bool>("headless", false);
    debug_base_ = DebugIdentification(base_params);

    if (base_params.debug_lights_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
    if (base_params.debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
    if (base_params.debug_number_classification_) RCLCPP_INFO(this->get_logger(), "数字识别DEBUG模式开启");
    if (base_params.debug_timecontrol_) {
        RCLCPP_INFO(this->get_logger(), "DEBUG模式：+/-调速度  P-暂停  ESC-退出");
    }
}

void ArmorPlateIdentification::initDetector()
{
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    std::string model_relative_path = this->declare_parameter<std::string>("model_path", "");
    std::string model_path = package_share_dir + "/" + model_relative_path;
    float number_threshold = static_cast<float>(this->declare_parameter<double>("number_threshold", 0.15));
    LightParams light_params;
    light_params.min_contours_area_ = 30;
    light_params.min_contours_ratio_ = 0.06f;
    light_params.max_contours_ratio_ = 0.5f;
    ArmorParams armor_params;
    armor_params.max_angle_diff_ = static_cast<float>(this->declare_parameter<double>("max_angle_diff", 10.0));
    armor_params.min_length_ratio_ = static_cast<float>(this->declare_parameter<double>("min_length_ratio", 0.7));
    armor_params.min_x_diff_ratio_ = static_cast<float>(this->declare_parameter<double>("min_x_diff_ratio", 0.75));
    armor_params.max_y_diff_ratio_ = static_cast<float>(this->declare_parameter<double>("max_y_diff_ratio", 1.0));
    armor_params.max_distance_ratio_ = static_cast<float>(this->declare_parameter<double>("max_distance_ratio", 0.8));
    armor_params.min_distance_ratio_ = static_cast<float>(this->declare_parameter<double>("min_distance_ratio", 0.1));
    armor_params.target_color_ = target_color_;
    lights_ = Detector(model_path, number_threshold,
                       light_params, armor_params,
                       this->declare_parameter<int>("threshold_value", 160),
                       this->declare_parameter<int>("color_threshold", 100));
}

void ArmorPlateIdentification::initPoseSolver()
{
    auto intrinsics = camera_.getIntrinsics();
    pose_solver_ = PoseSolver(
        intrinsics.camera_matrix,
        intrinsics.distortion_coefficients,
        intrinsics.projection_matrix
    );
}

builtin_interfaces::msg::Time ArmorPlateIdentification::convertSteadyToRosTime(
    const std::chrono::steady_clock::time_point& steady_stamp)
{
    auto now_steady = std::chrono::steady_clock::now();
    auto now_ros = this->now();
    auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now_steady - steady_stamp).count();

    // 每 100 帧打印一次 demosaicing / 格式转换延迟
    static int frame_count = 0;
    if (++frame_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Camera processing latency: %.3f ms", elapsed_ns / 1e6);
    }

    return now_ros - rclcpp::Duration::from_nanoseconds(elapsed_ns);
}
