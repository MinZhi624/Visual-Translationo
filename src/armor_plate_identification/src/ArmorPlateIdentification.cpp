#include "armor_plate_identification/ArmorPlateIdentification.hpp"
#include <armor_plate_interfaces/ArmorTypes.hpp>
#include <armor_plate_interfaces/ArmorPose.hpp>
#include <cmath>

static bool isValidProjection(const cv::Point2f & px)
{
    return std::isfinite(px.x) && std::isfinite(px.y) && px.x >= 0 && px.y >= 0;
}

static void drawArmorRect(cv::Mat & img, const std::vector<cv::Point2f> & points,
                          const cv::Scalar & color, int id)
{
    if (points.size() != 4) return;
    // 验证所有角点 finite
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

void ArmorPlateIdentification::run()
{
    if (!headless_) {
        gui_worker_.start();
    }

    while (rclcpp::ok()) {
        Frame frame;
        if (!frame_queue_.pop(frame, std::chrono::milliseconds(100))) continue;
        read_stamp_ = convertSteadyToRosTime(frame.timestamp);

        img_show_ = std::move(frame.img);
        debug_base_.onFrameStart();

        identification(img_show_);
        solvePose();
        publish();
        save();
        show();

        debug_base_.onFrameEnd();
        KeyEvent event = headless_ ? KeyEvent{} : gui_worker_.consumeKey();
        if (control(event)) break;
    }
    stopCameraCaptureWorker();
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

void ArmorPlateIdentification::plannerDebugCallBack(const PlannerDebug::SharedPtr msg)
{
    planner_debug_queue_.push(msg);
}

void ArmorPlateIdentification::processTrackerDebug(const TrackerDebug::SharedPtr msg)
{
    int64_t timestamp_ns = msg->header.stamp.sec * 1000000000LL +
      msg->header.stamp.nanosec;

    auto record = keyframe_cache_->submitTrackerDebug(timestamp_ns, msg);
    enqueueOverlay(std::move(record));
}

void ArmorPlateIdentification::processPlannerDebug(const PlannerDebug::SharedPtr msg)
{
    int64_t timestamp_ns = msg->header.stamp.sec * 1000000000LL +
      msg->header.stamp.nanosec;

    auto record = keyframe_cache_->submitPlannerDebug(timestamp_ns, msg);
    enqueueOverlay(std::move(record));
}

void ArmorPlateIdentification::enqueueOverlay(std::unique_ptr<KeyFrameRecord> record)
{
    if (record) overlay_queue_.push(std::move(record));
}

void ArmorPlateIdentification::submitFrameToCache(std::unique_ptr<KeyFrame> frame)
{
    if (!frame || !keyframe_cache_) return;
    enqueueOverlay(keyframe_cache_->submitFrame(std::move(frame)));
}

void ArmorPlateIdentification::flushRealtimeFrame()
{
    if (headless_) return;
    submitFrameToCache(gui_worker_.takeKeyFrame(DebugWindow::IDENTIFICATION));
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

void ArmorPlateIdentification::plannerDebugWorker()
{
    while (rclcpp::ok() && planner_debug_worker_running_) {
        PlannerDebug::SharedPtr msg;
        if (!planner_debug_queue_.pop(msg, std::chrono::milliseconds(100))) continue;
        if (msg) processPlannerDebug(msg);
    }
}

void ArmorPlateIdentification::stopPlannerDebugWorker()
{
    planner_debug_worker_running_ = false;
    if (planner_debug_thread_.joinable()) planner_debug_thread_.join();
}

void ArmorPlateIdentification::overlayWorker()
{
    while (overlay_worker_running_.load() || !overlay_queue_.empty()) {
        std::unique_ptr<KeyFrameRecord> record;
        if (!overlay_queue_.pop(record, std::chrono::milliseconds(100))) continue;
        if (record) compositeDebugOverlay(std::move(record));
    }
}

void ArmorPlateIdentification::stopOverlayWorker()
{
    overlay_worker_running_ = false;
    if (overlay_thread_.joinable()) overlay_thread_.join();
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
        frame_queue_.push(std::move(frame));
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

    keyframe_cache_ = std::make_unique<KeyFrameCache>();
    overlay_worker_running_ = true;
    overlay_thread_ = std::thread(&ArmorPlateIdentification::overlayWorker, this);

    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlateIdentification::trackerDebugCallBack, this, std::placeholders::_1)
    );
    tracker_debug_worker_running_ = true;
    tracker_debug_thread_ = std::thread(&ArmorPlateIdentification::trackerDebugWorker, this);

    planner_debug_sub_ = this->create_subscription<PlannerDebug>(
        "planner_debug", rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlateIdentification::plannerDebugCallBack, this, std::placeholders::_1)
    );
    planner_debug_worker_running_ = true;
    planner_debug_thread_ = std::thread(&ArmorPlateIdentification::plannerDebugWorker, this);

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
    // 计算插值
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
}

void ArmorPlateIdentification::show()
{
    GuiWorker::drawArmors(img_show_, lights_.getArmors());
    debug_base_.draw(img_show_);

    debug_base_.show();
    auto frames = debug_base_.getDisplayFrames();
    for (const auto& [name, img] : frames) {
        gui_worker_.pushFrame(name, img);
    }

    auto frame = std::make_unique<KeyFrame>();
    frame->timestamp_ns = read_stamp_.sec * 1000000000LL + read_stamp_.nanosec;
    frame->image = std::move(img_show_);
    frame->gimbal = matched_gimbal_;

    if (headless_) {
        submitFrameToCache(std::move(frame));
    } else {
        auto previous = gui_worker_.exchangeKeyFrame(
            DebugWindow::IDENTIFICATION, std::move(frame), 0.5);
        submitFrameToCache(std::move(previous));
    }
}

void ArmorPlateIdentification::compositeDebugOverlay(std::unique_ptr<KeyFrameRecord> record)
{
    if (!record || !record->frame || record->frame->image.empty()) return;

    cv::Mat & debug_img = record->frame->image;
    const GimbalData & gimbal = record->frame->gimbal;

    // --- Tracker overlay ---
    if (record->tracker_debug) {
        const auto & td = *record->tracker_debug;

        // Draw EKF car rotation center as RED filled circle
        Eigen::Vector3d center_world(td.center_world.x, td.center_world.y, td.center_world.z);
        cv::Point2f center_px = pose_solver_.xyzWorldToPixel(center_world, gimbal);
        if (isValidProjection(center_px)) {
            cv::circle(debug_img, center_px, 5, cv::Scalar(0, 0, 255), -1);
        }

        // Draw 4 predicted armors as YELLOW rectangles using reprojectArmor
        ArmorName name_enum = intToArmorName(td.armor_name);
        ArmorType type = armorNameToType(name_enum);
        for (std::size_t i = 0; i < td.predicted_armor_points_world.size(); ++i) {
            const auto & armor = td.predicted_armor_points_world[i];
            Eigen::Vector3d armor_world(armor.x, armor.y, armor.z);

            // Check if the center projects into the image before drawing the rectangle
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

        // GREEN = original, YELLOW = predicted, BLUE = compensated
        if (orig_ok) cv::circle(debug_img, orig_px, 5, cv::Scalar(0, 255, 0), -1);
        if (pred_ok) cv::circle(debug_img, pred_px, 5, cv::Scalar(0, 255, 255), -1);
        if (comp_ok) cv::circle(debug_img, comp_px, 5, cv::Scalar(255, 0, 0), -1);

        // GREEN line: original -> predicted
        if (orig_ok && pred_ok) cv::line(debug_img, orig_px, pred_px, cv::Scalar(0, 255, 0), 2);
        // YELLOW line: predicted -> compensated
        if (pred_ok && comp_ok)
            cv::line(debug_img, pred_px, comp_px, cv::Scalar(0, 255, 255), 2);

        } // if (pd.is_valid)
    }

    if (!headless_) {
        gui_worker_.exchangeKeyFrame(DebugWindow::TRACKER_DEBUG, std::move(record->frame));
    }
}

ArmorPlateIdentification::ArmorPlateIdentification() : Node("armor_plate_identification_node")
{
    init();
}

ArmorPlateIdentification::~ArmorPlateIdentification()
{
    stopCameraCaptureWorker();
    stopTrackerDebugWorker();
    stopPlannerDebugWorker();
    stopOverlayWorker();
    gui_worker_.stop();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorPlateIdentification>();
    std::thread spin_thread([&]() { rclcpp::spin(node); });
    node->run();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
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

    return now_ros - rclcpp::Duration::from_nanoseconds(elapsed_ns);
}
