#include "armor_plate_identification/ArmorPlateIdentification.hpp"

void ArmorPlateIdentification::run()
{
    RCLCPP_INFO(this->get_logger(), "开始图像处理循环");
    int fail_count = 0;
    while (rclcpp::ok()) {
        cv::Mat frame = camera_driver_.Read();
        read_stamp_ = this->now();
        if (frame.empty()) {
            fail_count++;
            if (fail_count > 5) {
                RCLCPP_FATAL(this->get_logger(), "Camera read failed!");
                rclcpp::shutdown();
            }
            continue;
        }
        fail_count = 0;
        img_show_ = frame.clone();

        debug_base_.onFrameStart();

        identification(frame);
        solvePose();
        publish();
        save();
        show();

        auto action = debug_base_.handleKey(cv::waitKey(1));
        if (action == KeyAction::Exit) break;
        if (action == KeyAction::Pause) {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
        }
        debug_base_.onFrameEnd();
        
        if (debug_base_.isDebugTimeControl()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(debug_base_.getDelayTimeMs()));
        }
    }
}

void ArmorPlateIdentification::trackerDebugCallBack(const TrackerDebug::SharedPtr msg)
{
    std::deque<ImageSave> images_buffs;
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        if (img_buffs_.empty()) return;
        images_buffs = img_buffs_;
    }
    // 寻找时间头最接近的帧（容差 1ms）
    auto to_ns = [](const auto& s) { return (int64_t)s.sec * 1000000000LL + s.nanosec; };
    auto it = std::find_if(images_buffs.begin(), images_buffs.end(), [msg, &to_ns](const ImageSave& image_save) {
        return std::abs(to_ns(image_save.img_stamp) - to_ns(msg->header.stamp)) < 1000000;
    });
    if (it == images_buffs.end()) return;

    cv::Mat debug_img = it->img.clone();
    cv::Point3f target_cam(msg->target_point.x, msg->target_point.y, msg->target_point.z);
    cv::Point3f filtered_cam(msg->filtered_point.x, msg->filtered_point.y, msg->filtered_point.z);

    cv::Point2f target_px = pose_solver_.xyzCameraToPixel(target_cam);
    cv::Point2f filtered_px = pose_solver_.xyzCameraToPixel(filtered_cam);

    auto drawCross = [](cv::Mat& img, const cv::Point2f& center, const cv::Scalar& color, int radius = 12) {
        cv::circle(img, center, radius, color, 2, cv::LINE_AA);
        cv::line(img, center + cv::Point2f(-radius, 0), center + cv::Point2f(radius, 0), color, 2, cv::LINE_AA);
        cv::line(img, center + cv::Point2f(0, -radius), center + cv::Point2f(0, radius), color, 2, cv::LINE_AA);
    };

    if (target_px.x >= 0) {
        drawCross(debug_img, target_px, cv::Scalar(0, 0, 255), 12);
    }
    if (filtered_px.x >= 0) {
        drawCross(debug_img, filtered_px, cv::Scalar(0, 255, 0), 12);
    }
    cv::Mat show_img;
    cv::resize(debug_img, show_img, cv::Size(), 0.5, 0.5);
    cv::imshow("Tracker Debug", show_img);
    cv::waitKey(1);
}

void ArmorPlateIdentification::init()
{
    target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");

    initDetector();

    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
    gimbal_angle_sub_ = this->create_subscription<GimbalAngle>(
        "gimbal_angle", 10,
        [this](const GimbalAngle::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(gimbal_mutex_);
            gimbal_data_.yaw_abs = msg->yaw_abs;
            gimbal_data_.pitch_abs = msg->pitch_abs;
        }
    );

    camera_type_ = this->declare_parameter<std::string>("camera_type", "galaxy");
    double exposure_time = this->declare_parameter<double>("exposure_time", 3500.0);
    double gain = this->declare_parameter<double>("gain", 1.0);
    if (!camera_driver_.init(camera_type_, exposure_time, gain)) {
        RCLCPP_FATAL(this->get_logger(), "相机初始化失败，程序退出");
        rclcpp::shutdown();
        return;
    }

    initPoseSolver();
    initDebug();

    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&ArmorPlateIdentification::trackerDebugCallBack, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "识别节点已启动，相机类型: %s", camera_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停");
    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
}

void ArmorPlateIdentification::identification(cv::Mat& img_bgr)
{
    cv::Mat img_thre = lights_.preprocess(img_bgr);
    debug_base_.mark("preprocess");

    lights_.detectArmors(img_thre, img_bgr);
    drawArmors(img_show_, lights_.getArmors());
    debug_base_.mark("detectArmors");

    armors_ = lights_.getArmors();

    // 收集 rejected ROI（录制模式）
    if (debug_base_.isRecordingRois()) {
        debug_base_.feedRejected(lights_.getRejectedNumberRois());
    }

    debug_base_.debugLights(lights_.getLights());
    debug_base_.debugNumberClassification(lights_.getArmors());
    debug_base_.debugPreprocessing(img_bgr, lights_.getPreprocessDebug());
}

void ArmorPlateIdentification::solvePose()
{
    std::vector<ArmorPlate> armor_plates;
    for (auto& armor : armors_) {
        pose_solver_.solve(armor);
        ArmorPlate armor_plate;
        armor_plate.pose.position.x = armor.xyz_camera_.x();
        armor_plate.pose.position.y = armor.xyz_camera_.y();
        armor_plate.pose.position.z = armor.xyz_camera_.z();
        armor_plate.pose.orientation.x = armor.q_camera_.x();
        armor_plate.pose.orientation.y = armor.q_camera_.y();
        armor_plate.pose.orientation.z = armor.q_camera_.z();
        armor_plate.pose.orientation.w = armor.q_camera_.w();
        armor_plate.number = static_cast<int>(armor.name_);
        armor_plate.image_distance_to_center = armor.image_distance_to_center_;
        armor_plates.push_back(armor_plate);
    }
    armor_plates_ = armor_plates;
}

void ArmorPlateIdentification::publish()
{
    // 发布装甲板数据
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = read_stamp_;
    armor_plates_msg.header.frame_id = "camera_link";
    armor_plates_msg.armor_plates = armor_plates_;
    {
        std::lock_guard<std::mutex> lock(gimbal_mutex_);
        armor_plates_msg.gimbal_yaw_abs = gimbal_data_.yaw_abs;
        armor_plates_msg.gimbal_pitch_abs = gimbal_data_.pitch_abs;
    }
    armor_plates_pub_->publish(armor_plates_msg);
}

void ArmorPlateIdentification::save()
{
    debug_base_.save();

    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        img_buffs_.push_back({read_stamp_ ,img_show_.clone()});
        if (img_buffs_.size() > 10) img_buffs_.pop_front();
    }
}

void ArmorPlateIdentification::show()
{
    debug_base_.draw(img_show_);

    cv::Mat show_img;
    cv::resize(img_show_, show_img, cv::Size(), 0.5, 0.5);
    cv::imshow("Identification", show_img);

    debug_base_.show();
}

ArmorPlateIdentification::ArmorPlateIdentification() : Node("armor_plate_identification_node"), camera_driver_(this)
{
    init();
}

ArmorPlateIdentification::~ArmorPlateIdentification()
{
    camera_driver_.close();
    cv::destroyAllWindows();
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
    base_params.debug_timecontrol_ = this->declare_parameter<bool>("debug_base", false);
    base_params.debug_lights_ = this->declare_parameter<bool>("debug_identification", false);
    base_params.debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    base_params.debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    base_params.delay_time = this->declare_parameter<int>("delay_time", 0);
    base_params.stats_interval = this->declare_parameter<int>("stats_interval", 50);

    debug_base_ = DebugBase(base_params);

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
    camera_info_msg_ = camera_driver_.getCameraInfo();
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
    cv::Mat projection_matrix = cv::Mat::zeros(3, 4, CV_64F);
    for (int i = 0; i < 9; ++i) {
        camera_matrix.at<double>(i / 3, i % 3) = camera_info_msg_.k[i];
    }
    for (size_t i = 0; i < camera_info_msg_.d.size() && i < 5; ++i) {
        distortion_coefficients.at<double>(0, static_cast<int>(i)) = camera_info_msg_.d[i];
    }
    for (int i = 0; i < 12; ++i) {
        projection_matrix.at<double>(i / 4, i % 4) = camera_info_msg_.p[i];
    }
    pose_solver_ = PoseSolver(camera_matrix, distortion_coefficients, projection_matrix);
}