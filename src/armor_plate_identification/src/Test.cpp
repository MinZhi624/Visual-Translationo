// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Test.hpp"
#include "armor_plate_identification/debug/YawSearchBenchmark.hpp"
#include <armor_plate_interfaces/ArmorTypes.hpp>
#include <armor_plate_interfaces/ArmorPose.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/exceptions.hpp>

#include <cmath>

static bool isValidProjection(const cv::Point2f & px)
{
    return std::isfinite(px.x) && std::isfinite(px.y) && px.x >= 0 && px.y >= 0;
}

static void drawArmorRect(cv::Mat & img, const std::vector<cv::Point2f> & points,
                          const cv::Scalar & color, int id)
{
    if (points.size() != 4) return;
    for (const auto & p : points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
    }
    for (int i = 0; i < 4; ++i) {
        cv::line(img, points[i], points[(i + 1) % 4], color, 2, cv::LINE_AA);
    }
    cv::Point2f center = (points[0] + points[2]) * 0.5f;
    cv::putText(img, std::to_string(id), center, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                cv::LINE_AA);
}

int Test::run()
{
    if (!headless_) {
        gui_worker_.start();
    }

    cv::Mat frame;
    while (rclcpp::ok()) {
        if (yaw_benchmark_ && yaw_benchmark_->shouldStop()) {
            RCLCPP_INFO(this->get_logger(), "Yaw benchmark 达到最大样本数，结束");
            break;
        }

        c_ >> frame;
        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "视频播放结束");
            break;
        }
        img_show_ = std::move(frame);

        if (yaw_benchmark_) {
            yaw_benchmark_->onFrameStart(raw_frame_index_);
        }
        ++raw_frame_index_;

        debug_test_.onFrameStart();

        // Test 模式用视频相对时间
        double video_time = debug_test_.getFrameCount() / fps_;
        read_stamp_.sec = static_cast<int>(video_time);
        read_stamp_.nanosec = static_cast<uint32_t>((video_time - read_stamp_.sec) * 1e9);

        identification(img_show_);
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
            break;
        }
    }
    RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    flushRealtimeFrame();
    rclcpp::shutdown();
    stopTrackerDebugWorker();
    stopPlannerDebugWorker();
    stopOverlayWorker();
    tracker_debug_queue_.clear();
    planner_debug_queue_.clear();
    overlay_queue_.clear();
    if (keyframe_cache_) keyframe_cache_->clear();
    gui_worker_.stop();
    return 0;
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
    tracker_debug_queue_.push(msg);
}

void Test::processTrackerDebug(const TrackerDebug::SharedPtr msg)
{
    int64_t timestamp_ns = msg->header.stamp.sec * 1000000000LL +
        msg->header.stamp.nanosec;

    auto record = keyframe_cache_->submitTrackerDebug(timestamp_ns, msg);
    enqueueOverlay(std::move(record));
}

void Test::trackerDebugWorker()
{
    while (rclcpp::ok() && tracker_debug_worker_running_) {
        TrackerDebug::SharedPtr msg;
        if (!tracker_debug_queue_.pop(msg, std::chrono::milliseconds(100))) continue;
        if (msg) processTrackerDebug(msg);
    }
}

void Test::stopTrackerDebugWorker()
{
    tracker_debug_worker_running_ = false;
    if (tracker_debug_thread_.joinable()) tracker_debug_thread_.join();
}

void Test::plannerDebugCallBack(const PlannerDebug::SharedPtr msg)
{
    planner_debug_queue_.push(msg);
}

void Test::processPlannerDebug(const PlannerDebug::SharedPtr msg)
{
    int64_t timestamp_ns = msg->header.stamp.sec * 1000000000LL +
        msg->header.stamp.nanosec;

    auto record = keyframe_cache_->submitPlannerDebug(timestamp_ns, msg);
    enqueueOverlay(std::move(record));
}

void Test::enqueueOverlay(std::unique_ptr<KeyFrameRecord> record)
{
    if (record) overlay_queue_.push(std::move(record));
}

void Test::submitFrameToCache(std::unique_ptr<KeyFrame> frame)
{
    if (!frame || !keyframe_cache_) return;
    enqueueOverlay(keyframe_cache_->submitFrame(std::move(frame)));
}

void Test::flushRealtimeFrame()
{
    if (headless_) return;
    submitFrameToCache(gui_worker_.takeKeyFrame(DebugWindow::IDENTIFICATION));
}

void Test::plannerDebugWorker()
{
    while (rclcpp::ok() && planner_debug_worker_running_) {
        PlannerDebug::SharedPtr msg;
        if (!planner_debug_queue_.pop(msg, std::chrono::milliseconds(100))) continue;
        if (msg) processPlannerDebug(msg);
    }
}

void Test::stopPlannerDebugWorker()
{
    planner_debug_worker_running_ = false;
    if (planner_debug_thread_.joinable()) planner_debug_thread_.join();
}

void Test::overlayWorker()
{
    while (overlay_worker_running_.load() || !overlay_queue_.empty()) {
        std::unique_ptr<KeyFrameRecord> record;
        if (!overlay_queue_.pop(record, std::chrono::milliseconds(100))) continue;
        if (record) compositeDebugOverlay(std::move(record));
    }
}

void Test::stopOverlayWorker()
{
    overlay_worker_running_ = false;
    if (overlay_thread_.joinable()) overlay_thread_.join();
}

void Test::compositeDebugOverlay(std::unique_ptr<KeyFrameRecord> record)
{
    if (!record || !record->frame || record->frame->image.empty()) return;

    cv::Mat & debug_img = record->frame->image;
    const GimbalData & gimbal = record->frame->gimbal;

    // --- Tracker overlay ---
    if (record->tracker_debug) {
        const auto & td = *record->tracker_debug;

        // 绘制车体旋转中心（红色实心圆）
        Eigen::Vector3d center_world(td.center_world.x, td.center_world.y, td.center_world.z);
        cv::Point2f center_px = pose_solver_.xyzWorldToPixel(center_world, gimbal);
        if (isValidProjection(center_px)) {
            cv::circle(debug_img, center_px, 7, cv::Scalar(0, 0, 255), -1);
        }

        // 绘制 4 个预测装甲板（黄色矩形）
        ArmorName name_enum = intToArmorName(td.armor_name);
        ArmorType type = armorNameToType(name_enum);
        for (std::size_t i = 0; i < td.predicted_armor_points_world.size(); ++i) {
            const auto & armor = td.predicted_armor_points_world[i];
            Eigen::Vector3d armor_world(armor.x, armor.y, armor.z);

            cv::Point2f armor_px = pose_solver_.xyzWorldToPixel(armor_world, gimbal);
            if (!isValidProjection(armor_px)) continue;

            ArmorPose armor_pose;
            armor_pose.xyz_world = armor_world;
            armor_pose.yaw = (i < td.predicted_armor_yaws_world.size())
                                 ? td.predicted_armor_yaws_world[i]
                                 : 0.0;
            armor_pose.name = name_enum;
            armor_pose.type = type;

            auto corners = pose_solver_.reprojectArmor(armor_pose, gimbal);
            drawArmorRect(debug_img, corners, cv::Scalar(0, 255, 255), static_cast<int>(i));
        }
    }

    // --- Planner overlay ---
    if (record->planner_debug) {
        const auto & pd = *record->planner_debug;
        if (!pd.is_valid) {
            cv::putText(debug_img, "PLANNER INVALID", cv::Point(10, 55),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        } else {
            Eigen::Vector3d orig_world(pd.original_point_world.x, pd.original_point_world.y,
                                       pd.original_point_world.z);
            cv::Point2f orig_px = pose_solver_.xyzWorldToPixel(orig_world, gimbal);

            Eigen::Vector3d pred_world(pd.predicted_point_world.x, pd.predicted_point_world.y,
                                       pd.predicted_point_world.z);
            cv::Point2f pred_px = pose_solver_.xyzWorldToPixel(pred_world, gimbal);

            Eigen::Vector3d comp_world(pd.compensated_point_world.x, pd.compensated_point_world.y,
                                       pd.compensated_point_world.z);
            cv::Point2f comp_px = pose_solver_.xyzWorldToPixel(comp_world, gimbal);

            bool orig_ok = isValidProjection(orig_px);
            bool pred_ok = isValidProjection(pred_px);
            bool comp_ok = isValidProjection(comp_px);

            // 绿色=原始，黄色=预测，蓝色=补偿
            if (orig_ok) cv::circle(debug_img, orig_px, 5, cv::Scalar(0, 255, 0), -1);
            if (pred_ok) cv::circle(debug_img, pred_px, 5, cv::Scalar(0, 255, 255), -1);
            if (comp_ok) cv::circle(debug_img, comp_px, 5, cv::Scalar(255, 0, 0), -1);

            // 绿线：原始→预测
            if (orig_ok && pred_ok) cv::line(debug_img, orig_px, pred_px, cv::Scalar(0, 255, 0), 2);
            // 黄线：预测→补偿
            if (pred_ok && comp_ok)
                cv::line(debug_img, pred_px, comp_px, cv::Scalar(0, 255, 255), 2);
        }
    }

    if (!headless_ && debug_test_.shouldShow()) {
        gui_worker_.exchangeKeyFrame(DebugWindow::TRACKER_DEBUG, std::move(record->frame));
    }
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
    gimbal_angle_pub_ = this->create_publisher<armor_plate_interfaces::msg::GimbalAngle>("gimbal_angle", rclcpp::SensorDataQoS());

    keyframe_cache_ = std::make_unique<KeyFrameCache>(100);
    overlay_worker_running_ = true;
    overlay_thread_ = std::thread(&Test::overlayWorker, this);

    initDebug();
    initYawBenchmark();
    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
}

void Test::identification(cv::Mat& img_bgr)
{
    cv::Mat img_thre = lights_.preprocess(img_bgr);
    debug_test_.mark("preprocess");

    lights_.detectArmors(img_thre, img_bgr);
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
        armor_plate.x_world = armor.xyz_world_.x();
        armor_plate.y_world = armor.xyz_world_.y();
        armor_plate.z_world = armor.xyz_world_.z();
        armor_plate.yaw_world = armor.ypr_world_.x();
        armor_plate.number = static_cast<int>(armor.name_);
        armor_plate.image_distance_to_center = armor.image_distance_to_center_;
        armor_plates_msg.armor_plates.push_back(armor_plate);
    }
    armor_plates_msg.gimbal_yaw_abs = test_gimbal_.yaw_abs;
    armor_plates_msg.gimbal_pitch_abs = test_gimbal_.pitch_abs;
    armor_plates_pub_->publish(armor_plates_msg);

    // 同步发布 GimbalAngle，供 Planner 节点使用
    armor_plate_interfaces::msg::GimbalAngle gimbal_msg;
    gimbal_msg.stamp = read_stamp_;
    gimbal_msg.yaw_abs = test_gimbal_.yaw_abs;
    gimbal_msg.pitch_abs = test_gimbal_.pitch_abs;
    gimbal_angle_pub_->publish(gimbal_msg);
}

void Test::save()
{
    debug_test_.save();
}

void Test::show()
{
    GuiWorker::drawArmors(img_show_, lights_.getArmors());
    debug_test_.draw(img_show_);

    debug_test_.show();
    auto frames = debug_test_.getDisplayFrames();
    for (const auto& [name, img] : frames) {
        gui_worker_.pushFrame(name, img);
    }

    auto frame = std::make_unique<KeyFrame>();
    frame->timestamp_ns = read_stamp_.sec * 1000000000LL + read_stamp_.nanosec;
    frame->image = std::move(img_show_);
    frame->gimbal = test_gimbal_;

    if (headless_) {
        submitFrameToCache(std::move(frame));
    } else {
        auto previous = gui_worker_.exchangeKeyFrame(
            DebugWindow::IDENTIFICATION, std::move(frame), 0.5);
        submitFrameToCache(std::move(previous));
    }
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
    pose_solver_.setYawSearchObserver(nullptr);
    stopTrackerDebugWorker();
    stopPlannerDebugWorker();
    stopOverlayWorker();
    tracker_debug_queue_.clear();
    planner_debug_queue_.clear();
    overlay_queue_.clear();
    if (keyframe_cache_) keyframe_cache_->clear();
    gui_worker_.stop();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Test>(argv[1]);
    std::thread spin_thread([&](){rclcpp::spin(node);});
    int status = node->run();
    node->stopTrackerDebugWorker();
    node->stopPlannerDebugWorker();
    rclcpp::shutdown();
    if(spin_thread.joinable()) spin_thread.join();
    node->closeTrackerDebugFile();
    bool finalized = node->finalizeYawBenchmark();
    return (status == 0 && finalized) ? 0 : (status != 0 ? status : 1);
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

    planner_debug_sub_ = this->create_subscription<PlannerDebug>(
        "planner_debug", rclcpp::SensorDataQoS(),
        std::bind(&Test::plannerDebugCallBack, this, std::placeholders::_1)
    );
    planner_debug_worker_running_ = true;
    planner_debug_thread_ = std::thread(&Test::plannerDebugWorker, this);
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

void Test::initYawBenchmark()
{
    bool enabled = this->declare_parameter<bool>("yaw_benchmark_enabled", false);
    if (!enabled) {
        return;
    }

    auto throw_invalid = [this](const std::string& msg) {
        RCLCPP_ERROR(this->get_logger(), "Benchmark 参数校验失败: %s", msg.c_str());
        throw rclcpp::exceptions::InvalidParameterValueException(msg);
    };

    std::string output_csv = this->declare_parameter<std::string>("yaw_benchmark_output_csv", "");
    if (output_csv.empty()) {
        throw_invalid("yaw_benchmark_output_csv 必须非空");
    }

    std::vector<double> steps_deg = this->declare_parameter<std::vector<double>>(
        "yaw_benchmark_steps_deg", {0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0});
    for (double s : steps_deg) {
        if (!std::isfinite(s) || s <= 0.0 || s > 60.0) {
            throw_invalid("yaw_benchmark_steps_deg 每项必须在 (0, 60] 内");
        }
    }

    std::vector<int64_t> iterations_raw = this->declare_parameter<std::vector<int64_t>>(
        "yaw_benchmark_iterations", {5, 8, 10, 12, 15});
    std::vector<int> iterations;
    iterations.reserve(iterations_raw.size());
    for (int64_t it : iterations_raw) {
        if (it < 0 || it > 1000) {
            throw_invalid("yaw_benchmark_iterations 每项必须在 [0, 1000] 内");
        }
        iterations.push_back(static_cast<int>(it));
    }

    int sample_stride = this->declare_parameter<int>("yaw_benchmark_sample_stride", 5);
    if (sample_stride <= 0) {
        throw_invalid("yaw_benchmark_sample_stride 必须 > 0");
    }

    int max_samples = this->declare_parameter<int>("yaw_benchmark_max_samples", 2000);
    if (max_samples < 0) {
        throw_invalid("yaw_benchmark_max_samples 必须 >= 0");
    }

    int warmup_samples = this->declare_parameter<int>("yaw_benchmark_warmup_samples", 50);
    if (warmup_samples < 0) {
        throw_invalid("yaw_benchmark_warmup_samples 必须 >= 0");
    }

    namespace adb = armor_plate_identification::debug;
    adb::YawSearchBenchmarkParams params;
    params.enabled = true;
    params.steps_deg = std::move(steps_deg);
    params.iterations = std::move(iterations);
    params.sample_stride = sample_stride;
    params.max_samples = max_samples;
    params.warmup_samples = warmup_samples;
    params.output_csv = std::move(output_csv);
    params.video_name = test_name_;

    yaw_benchmark_ = std::make_unique<adb::YawSearchBenchmark>(this, params);
    pose_solver_.setYawSearchObserver(yaw_benchmark_.get());
    RCLCPP_INFO(this->get_logger(), "Yaw 搜索 benchmark 已启用：stride=%d max=%d warmup=%d",
                sample_stride, max_samples, warmup_samples);
}

bool Test::finalizeYawBenchmark()
{
    if (!yaw_benchmark_) {
        return true;
    }
    return yaw_benchmark_->finalize();
}

