// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Test.hpp"
#include <rclcpp/logging.hpp>

#include <cmath>

void Test::run()
{
    if (!headless_) {
        gui_worker_.start();
    }

    cv::Mat frame;
    while (rclcpp::ok()) {
        c_ >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "视频播放结束");
            gui_worker_.stop();
            return;
        }
        img_show_ = frame.clone();
        debug_test_.onFrameStart();

        // Test 模式用视频相对时间
        double video_time = debug_test_.getFrameCount() / fps_;
        read_stamp_.sec = static_cast<int>(video_time);
        read_stamp_.nanosec = static_cast<uint32_t>((video_time - read_stamp_.sec) * 1e9);

        identification(frame);
        solvePose();
        debug_test_.mark("solvePose");
        publish();
        debug_test_.mark("publish");
        save();
        debug_test_.mark("save");
        show();
        debug_test_.mark("show");

        debug_test_.onFrameEnd();

        KeyEvent event = headless_ ? KeyEvent{} : gui_worker_.consumeKey();
        if (control(event)) break;

        if (debug_test_.shouldExit()) {
            RCLCPP_INFO(this->get_logger(), "帧调试：已播放到第 %d 帧，结束", debug_test_.getDebugFrameCount());
            gui_worker_.stop();
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    gui_worker_.stop();
}

bool Test::control(const KeyEvent& event)
{
    debug_test_.control(event);

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

    if (debug_test_.isDebugTimeControl()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(debug_test_.getDelayTimeMs()));
    }

    return false;
}

void Test::trackerDebugCallBack(const TrackerDebug::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(tracker_debug_queue_mutex_);
        tracker_debug_msgs_.clear();
        tracker_debug_msgs_.push_back(msg);
    }
    tracker_debug_cv_.notify_one();
}

void Test::processTrackerDebug(const TrackerDebug::SharedPtr msg)
{
    std::deque<Record> images_buffs;
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        if (img_buffs_.empty()) return;
        images_buffs = img_buffs_;
    }
    auto to_ns = [](const auto& s) { return (int64_t)s.sec * 1000000000LL + s.nanosec; };
    int64_t msg_ns = to_ns(msg->header.stamp);
    // 最近邻匹配
    auto it = images_buffs.begin();
    int64_t best_diff = std::abs(to_ns(it->img_stamp) - msg_ns);
    for (auto jt = std::next(it); jt != images_buffs.end(); ++jt) {
        int64_t diff = std::abs(to_ns(jt->img_stamp) - msg_ns);
        if (diff < best_diff) {
            best_diff = diff;
            it = jt;
        }
    }
    if (best_diff > 50000000) return;  // 超过 50ms 放弃

    // infoTrackerDebugMsg(msg);

    cv::Mat debug_img = it->img.clone();

    debug_tracker_.drawTragetPoints(debug_img, *msg, it->gimbal);
    debug_tracker_.drawPredictedCar(debug_img, *msg, it->gimbal);

    if (!headless_ && debug_test_.shouldShow()) {
        debug_tracker_.pushTrackerDebugFrame(debug_img);
    }

    if (debug_test_.isDebugFrameMode()) {
        std::string log_dir = "Debug/Tracker/" + test_name_ + "/ekf/temp";
        debug_test_.saveTrackerDebug(log_dir, *msg);
        if (++tracker_debug_count_ >= debug_test_.getDebugFrameCount()) {
            RCLCPP_INFO(this->get_logger(), "Tracker 已收到 %d 条消息，结束", tracker_debug_count_);
            rclcpp::shutdown();
        }
    }
}

void Test::trackerDebugWorker()
{
    while (rclcpp::ok()) {
        TrackerDebug::SharedPtr msg;
        {
            std::unique_lock<std::mutex> lock(tracker_debug_queue_mutex_);
            tracker_debug_cv_.wait(lock, [this]() {
                return !tracker_debug_worker_running_ || !tracker_debug_msgs_.empty();
            });
            // 防止虚假唤醒
            if (!tracker_debug_worker_running_ && tracker_debug_msgs_.empty()) return;
            msg = tracker_debug_msgs_.back();
            tracker_debug_msgs_.clear();
        }
        if (msg) processTrackerDebug(msg);
    }
}

void Test::stopTrackerDebugWorker()
{
    {
        std::lock_guard<std::mutex> lock(tracker_debug_queue_mutex_);
        tracker_debug_worker_running_ = false;
        tracker_debug_msgs_.clear();
    }
    tracker_debug_cv_.notify_all();
    if (tracker_debug_thread_.joinable()) tracker_debug_thread_.join();
}

void Test::init(const std::string& video_path)
{
    c_.open(video_path);
    if (!c_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_path.c_str());
        rclcpp::shutdown();
        return;
    }
    std::filesystem::path vp(video_path);
    test_name_ = vp.stem().string();
    RCLCPP_INFO(this->get_logger(), "测试名称: %s", test_name_.c_str());

    double test_gimbal_yaw_deg = 0.0f;
    double test_gimbal_pitch_deg = 0.0f;
    test_gimbal_.yaw_abs = static_cast<float>(test_gimbal_yaw_deg * M_PI / 180.0);
    test_gimbal_.pitch_abs = static_cast<float>(test_gimbal_pitch_deg * M_PI / 180.0);

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
    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", rclcpp::SensorDataQoS());

    initDebug();
    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
}

void Test::identification(cv::Mat& img_bgr)
{
    cv::Mat img_thre = lights_.preprocess(img_bgr);
    debug_test_.mark("preprocess");

    lights_.detectArmors(img_thre, img_bgr);
    GuiWorker::drawArmors(img_show_, lights_.getArmors());
    debug_test_.mark("detectArmors");

    armors_ = lights_.getArmors();
    /////// DEBUG /////
    if (debug_test_.isRecordingRois()) {
        debug_test_.feedRejected(lights_.getRejectedNumberRois());
    }

    debug_test_.debugLights(lights_.getLights());
    debug_test_.debugNumberClassification(lights_.getArmors());
    debug_test_.debugPreprocessing(img_bgr, lights_.getPreprocessDebug());
}

void Test::solvePose()
{
    pose_solver_.solve(armors_);
}

void Test::publish()
{
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = read_stamp_;
    armor_plates_msg.header.frame_id = "camera_link";
    armor_plates_msg.armor_plates.reserve(armors_.size());
    for (const auto& armor : armors_) {
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
        armor_plates_msg.armor_plates.push_back(armor_plate);
    }
    armor_plates_msg.gimbal_yaw_abs = test_gimbal_.yaw_abs;
    armor_plates_msg.gimbal_pitch_abs = test_gimbal_.pitch_abs;
    armor_plates_pub_->publish(armor_plates_msg);
}

void Test::save()
{
    debug_test_.save();
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        img_buffs_.push_back({read_stamp_, img_show_.clone(), test_gimbal_});
        if (img_buffs_.size() > 50) img_buffs_.pop_front();
    }
}

void Test::show()
{
    debug_test_.draw(img_show_);

    if (!headless_ && debug_test_.shouldShow()) {
        gui_worker_.pushFrame(DebugWindow::IDENTIFICATION, img_show_);
    }

    debug_test_.show();
    auto frames = debug_test_.getDisplayFrames();
    for (const auto& [name, img] : frames) {
        gui_worker_.pushFrame(name, img);
    }
    debug_test_.mark("show");
}

void Test::closeTrackerDebugFile()
{
    stopTrackerDebugWorker();
    debug_test_.closeTrackerDebugFile();
}

Test::Test(std::string video_path) : Node("test_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
    init(video_path);
}

Test::~Test()
{
    stopTrackerDebugWorker();
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
    base_params.debug_timecontrol_ = this->declare_parameter<bool>("debug_timecontrol", false);
    base_params.debug_lights_ = this->declare_parameter<bool>("debug_lights", false);
    base_params.debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    base_params.debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    base_params.delay_time = this->declare_parameter<int>("delay_time", 20);
    base_params.stats_interval = this->declare_parameter<int>("stats_interval", 50);

    DebugTestParams test_params;
    test_params.headless = this->declare_parameter<bool>("headless", false);
    test_params.debug_frame = this->declare_parameter<bool>("debug_frame", false);
    test_params.debug_frame_count = this->declare_parameter<int>("debug_frame_count", 100);

    headless_ = test_params.headless;
    debug_test_ = DebugTest(base_params, test_params);

    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&Test::trackerDebugCallBack, this, std::placeholders::_1)
    );
    tracker_debug_worker_running_ = true;
    tracker_debug_thread_ = std::thread(&Test::trackerDebugWorker, this);
    if (base_params.debug_timecontrol_) RCLCPP_INFO(this->get_logger(), "时间控制DEBUG模式开启");
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
