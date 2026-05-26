// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Test.hpp"
#include <rclcpp/logging.hpp>

void Test::run()
{
    cv::Mat frame;
    while (rclcpp::ok()) {
        c_ >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "视频播放结束");
            return;
        }
        img_show_ = frame.clone();
        debug_test_.onFrameStart();

        identification(frame);
        solvePose();
        publish();
        save();
        show();

        auto action = debug_test_.handleKey(cv::waitKey(1));
        if (action == KeyAction::Exit) break;
        if (action == KeyAction::Pause) {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
        }

        debug_test_.onFrameEnd();

        if (debug_test_.isDebugTimeControl()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(debug_test_.getDelayTimeMs()));
        }

        if (debug_test_.shouldExit()) {
            RCLCPP_INFO(this->get_logger(), "帧调试：已播放到第 %d 帧，结束", debug_test_.getDebugFrameCount());
            cv::destroyAllWindows();
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    cv::destroyAllWindows();
}

void Test::trackerDebugCallBack(const TrackerDebug::SharedPtr msg)
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
    cv::Point3f target_gambal(msg->target_point.x, msg->target_point.y, msg->target_point.z);
    cv::Point3f filtered_gambal(msg->filtered_point.x, msg->filtered_point.y, msg->filtered_point.z);

    cv::Point2f target_px = pose_solver_.xyzCameraToPixel(target_gambal);
    cv::Point2f filtered_px = pose_solver_.xyzCameraToPixel(filtered_gambal);

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
    if (debug_test_.shouldShow()) {
        cv::Mat show_img;
        cv::resize(debug_img, show_img, cv::Size(), 0.5, 0.5);
        cv::imshow("Tracker Debug", show_img);
        cv::waitKey(1);
    }

    // 输出 TrackerDebug 数据（仅在 debug_frame 模式下记录）
    if (debug_test_.isDebugFrameMode()) {
        std::string log_dir = "Debug/Tracker/" + test_name_ + "/ekf/temp";
        debug_test_.saveTrackerDebug(log_dir, *msg);
        if (++tracker_debug_count_ >= debug_test_.getDebugFrameCount()) {
            RCLCPP_INFO(this->get_logger(), "Tracker 已收到 %d 条消息，结束", tracker_debug_count_);
            rclcpp::shutdown();
        }
    }
}

void Test::init(const std::string& video_path)
{
    // ===== 相机初始化 ==== //
    c_.open(video_path);
    if (!c_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path.c_str());
        rclcpp::shutdown();
        return;
    }
    std::filesystem::path vp(video_path);
    test_name_ = vp.stem().string();
    RCLCPP_INFO(this->get_logger(), "测试名称: %s", test_name_.c_str());
    double fps = c_.get(cv::CAP_PROP_FPS);
    if (fps <= 0) {
        RCLCPP_WARN(this->get_logger(), "无法获取视频FPS，使用默认值: 50.0");
        fps_ = 50.0;
    } else {
        RCLCPP_INFO(this->get_logger(), "视频FPS: %.2f", fps);
        fps_ = fps;
    }

    initDetector();
    initPoseSolver();

    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);

    initDebug();

    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&Test::trackerDebugCallBack, this, std::placeholders::_1)
    );

    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
}

void Test::identification(cv::Mat& img_bgr)
{
    cv::Mat img_thre = lights_.preprocess(img_bgr);
    debug_test_.mark("preprocess");

    lights_.detectArmors(img_thre, img_bgr);
    drawArmors(img_show_, lights_.getArmors());
    debug_test_.mark("detectArmors");

    armors_ = lights_.getArmors();

    // 收集 rejected ROI（录制模式）
    if (debug_test_.isRecordingRois()) {
        debug_test_.feedRejected(lights_.getRejectedNumberRois());
    }

    debug_test_.debugLights(lights_.getLights());
    debug_test_.debugNumberClassification(lights_.getArmors());
    debug_test_.debugPreprocessing(img_bgr, lights_.getPreprocessDebug());
}

void Test::solvePose()
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

void Test::publish()
{
    double time_sec = static_cast<double>(debug_test_.getFrameCount()) / fps_;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(time_sec);
    stamp.nanosec = static_cast<uint32_t>((time_sec - stamp.sec) * 1e9);

    // 发布装甲板数据
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = stamp;
    armor_plates_msg.header.frame_id = "camera_link";
    armor_plates_msg.armor_plates = armor_plates_;
    armor_plates_pub_->publish(armor_plates_msg);
}

void Test::save()
{
    debug_test_.save();
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        double time_sec = static_cast<double>(debug_test_.getFrameCount()) / fps_;
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(time_sec);
        stamp.nanosec = static_cast<uint32_t>((time_sec - stamp.sec) * 1e9);
        img_buffs_.push_back({stamp, img_show_.clone()});
        if (img_buffs_.size() > 10) img_buffs_.pop_front();
    }
}

void Test::show()
{
    debug_test_.draw(img_show_);

    if (debug_test_.shouldShow()) {
        cv::Mat show_img;
        cv::resize(img_show_, show_img, cv::Size(), 0.5, 0.5);
        cv::imshow("Identification", show_img);
    }

    debug_test_.show();
}

void Test::closeTrackerDebugFile()
{
    debug_test_.closeTrackerDebugFile();
}


Test::Test(std::string video_path) : Node("test_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
    init(video_path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Test>(argv[1]);
    std::thread spin_thread([&](){rclcpp::spin(node);});
    node->run();
    rclcpp::shutdown();
    if(spin_thread.joinable()) spin_thread.join();
    node->closeTrackerDebugFile();
    return 0;
}

void Test::initDebug()
{
    DebugBaseParams base_params;
    base_params.debug_timecontrol_ = this->declare_parameter<bool>("debug_base", false);
    base_params.debug_lights_ = this->declare_parameter<bool>("debug_identification", false);
    base_params.debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    base_params.debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    base_params.delay_time = this->declare_parameter<int>("delay_time", 20);
    base_params.stats_interval = this->declare_parameter<int>("stats_interval", 50);

    DebugTestParams test_params;
    test_params.headless = this->declare_parameter<bool>("headless", false);
    test_params.debug_frame = this->declare_parameter<bool>("debug_frame", false);
    test_params.debug_frame_count = this->declare_parameter<int>("debug_frame_count", 100);

    debug_test_ = DebugTest(base_params, test_params);

    if (base_params.debug_lights_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
    if (base_params.debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
    if (base_params.debug_number_classification_) RCLCPP_INFO(this->get_logger(), "数字识别DEBUG模式开启");
    if (test_params.debug_frame) RCLCPP_INFO(this->get_logger(), "帧调试模式开启，将在第 %d 帧结束", test_params.debug_frame_count);
    if (test_params.headless) RCLCPP_INFO(this->get_logger(), "无头模式：跳过所有 GUI 窗口");
}
void Test::initDetector()
{
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    float number_threshold = static_cast<float>(this->declare_parameter<double>("number_threshold", 0.15));
    LightParams light_params {
        .min_contours_area_ = 30,
        .min_contours_ratio_ = 0.06f,
        .max_contours_ratio_ = 0.5f
    };
    ArmorParams armor_params = {
        .max_angle_diff_ = static_cast<float>(this->declare_parameter<double>("max_angle_diff", 10.0)),
        .min_length_ratio_ = static_cast<float>(this->declare_parameter<double>("min_length_ratio", 0.70)),
       .min_x_diff_ratio_ = static_cast<float>(this->declare_parameter<double>("min_x_diff_ratio", 0.75)),
       .max_y_diff_ratio_ = static_cast<float>(this->declare_parameter<double>("max_y_diff_ratio", 1.0)),
       .max_distance_ratio_ = static_cast<float>(this->declare_parameter<double>("max_distance_ratio", 0.8)),
       .min_distance_ratio_ = static_cast<float>(this->declare_parameter<double>("min_distance_ratio", 0.1)),
        .target_color_ = this->declare_parameter<std::string>("target_color", "BLUE")
    };
    lights_ = Detector(package_share_dir, number_threshold,
                       light_params, armor_params,
                       this->declare_parameter<int>("threshold_value", 160),
                       this->declare_parameter<int>("color_threshold", 100));
}
void Test::initPoseSolver()
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        2374.54248, 0., 698.85288,
        0., 2377.53648, 520.8649,
        0., 0., 1.);
    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) <<
        -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
    pose_solver_ = PoseSolver(camera_matrix, distortion_coefficients);
}