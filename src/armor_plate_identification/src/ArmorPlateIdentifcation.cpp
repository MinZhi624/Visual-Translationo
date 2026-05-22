#include "armor_plate_identification/ArmorPlateIdentification.hpp"

void ArmorPlateIdentification::init()
{
    target_color_ = this->declare_parameter<std::string>("target_color", "BLUE");

    // ===== 初始化数字识别（通过 Detector 内部的 classifier） ===== //
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    std::string model_relative_path = this->declare_parameter<std::string>("model_path", "");
    std::string model_path = package_share_dir + "/" + model_relative_path;
    float number_threshold = static_cast<float>(this->declare_parameter<double>("number_threshold", 0.15));
    // ==== Detector 参数 ==== //
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

    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);

    camera_type_ = this->declare_parameter<std::string>("camera_type", "galaxy");
    if (camera_type_ == "galaxy") {
        camera_frame_id_ = "camera_optical_frame";
    } else {
        camera_frame_id_ = "camera_link";
    }
    double exposure_time = this->declare_parameter<double>("exposure_time", 3500.0);
    double gain = this->declare_parameter<double>("gain", 1.0);
    if (!camera_driver_.init(camera_type_, exposure_time, gain)) {
        RCLCPP_FATAL(this->get_logger(), "相机初始化失败，程序退出");
        rclcpp::shutdown();
        return;
    }
    // ===== PoseSolver ===== //
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
    // ===== DEUBG ===== //
    debug_base_ = this->declare_parameter<bool>("debug_base", false);
    debug_identification_ = this->declare_parameter<bool>("debug_identification", false);
    debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    int delay_time = this->declare_parameter<int>("delay_time", 0);
    debug_controller_.setPlayDelayMs(delay_time);
    // 订阅 Tracker 回传的调试数据
    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&ArmorPlateIdentification::TrackerDebugCallBack, this, std::placeholders::_1)
    );
    // ====== 打印信息 ====== //
    RCLCPP_INFO(this->get_logger(), "识别节点已启动，相机类型: %s, frame_id: %s", camera_type_.c_str(), camera_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停");
    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
    if (debug_identification_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
    if (debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
    if (debug_number_classification_) RCLCPP_INFO(this->get_logger(), "数字识别DEBUG模式开启");

    if (debug_base_) {
        RCLCPP_INFO(this->get_logger(), "DEBUG模式：+/-调速度  P-暂停  ESC-退出");
    }
}

void ArmorPlateIdentification::Identification(cv::Mat& img_bgr)
{
    auto t_id0 = std::chrono::steady_clock::now();

    // 双通道预处理（调试图像可选）
    PreprocessDebug* debug_ptr = debug_preprocessing_ ? &preprocess_debug_ : nullptr;
    cv::Mat img_thre = lights_.preprocess(img_bgr, debug_ptr);

    auto t_id2 = std::chrono::steady_clock::now();

    // 灯条匹配
    lights_.detectArmors(img_thre, img_bgr);
    drawArmors(img_show_, lights_.getArmors());
    armors_ = lights_.getArmors();

    // 收集 rejected ROI（录制模式）
    if (debug_number_classification_ && roi_collector_.isRecording()) {
        roi_collector_.feedRejected(lights_.getRejectedNumberRois());
    }
    auto t_id3 = std::chrono::steady_clock::now();
    // 细分耗时累计
    id_split_sum_ += std::chrono::duration<double, std::milli>(t_id2 - t_id0).count();
    id_detect_sum_ += std::chrono::duration<double, std::milli>(t_id3 - t_id2).count();
    ////////////////////// DEBUG ////////////////////////
    if (debug_preprocessing_) {
        // 标记碎片数到 blue_dim_thre 上
        cv::Mat blue_debug;
        cv::cvtColor(preprocess_debug_.blue_dim_thre, blue_debug, cv::COLOR_GRAY2BGR);
        for (const auto& [rect, count] : preprocess_debug_.fragment_info) {
            std::string num = std::to_string(count);
            cv::Scalar color = (count == 1) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::putText(blue_debug, num, cv::Point(rect.x + 5, rect.y + 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
            cv::rectangle(blue_debug, rect, color, 1);
        }
        std::vector<cv::Mat> images = {img_bgr, blue_debug, preprocess_debug_.gray_thre, preprocess_debug_.merged_thre};
        std::vector<std::string> labels = {"Original", "BLUE_dim (fragments)", "GRAY_thre", "Merged"};
        showMultiImages("PreProcessions-View", images, labels);
    }
    if (debug_identification_) {
        drawAllLights(img_show_, lights_.getLights());
        debug_controller_.drawDebugInfo(img_show_, debug_base_);
    }
}

void ArmorPlateIdentification::SolvePose()
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

void ArmorPlateIdentification::NumberClassify()
{
    ////////// DEBUG /////////
    if (debug_number_classification_) {
        drawAllNumberTest(img_show_, armors_);
    }
}

void ArmorPlateIdentification::Publish()
{
    // 发布装甲板数据
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = read_stamp_;
    armor_plates_msg.header.frame_id = camera_frame_id_;
    armor_plates_msg.armor_plates = armor_plates_;
    armor_plates_pub_->publish(armor_plates_msg);
}

void ArmorPlateIdentification::controlParams()
{
    int key = cv::waitKey(1);
    if (key == -1) return;
    if (key == 27 || key == 'q' || key == 'Q')
    {
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return;
    }
    if (key == 'p' || key == 'P') {
        RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
        cv::waitKey(0);
        return;
    }
    if (key == 's' || key == 'S') {
        roi_collector_.toggleRecording();
    }
    if (debug_base_) {
        if (debug_controller_.handleKey(key, this->get_logger())) {
            return;
        }
    }
}

void ArmorPlateIdentification::ImageShow()
{
    debug_controller_.drawProcessTime(img_show_, process_time_ms_);
    if (debug_base_) {
        debug_controller_.drawDelay(img_show_);
    }
    if (debug_number_classification_) {
        roi_collector_.show();
    }
    cv::Mat show_img;
    cv::resize(img_show_, show_img, cv::Size(), 0.5, 0.5);
    cv::imshow("Identifacation", show_img);
}

void ArmorPlateIdentification::Save()
{
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        img_buffs_.push_back({read_stamp_ ,img_show_.clone()});
        if (img_buffs_.size() > 10) img_buffs_.pop_front();
    }
}

void ArmorPlateIdentification::TrackerDebugCallBack(const TrackerDebug::SharedPtr msg)
{
    std::deque<ImageSave> images_buffs;
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        if (img_buffs_.empty()) return;
        images_buffs = img_buffs_;
    }
    // 寻找时间头最接近的帧（容差 1ms）
    auto it = std::find_if(images_buffs.begin(), images_buffs.end(), [msg](const ImageSave& image_save) {
        const auto& t1 = image_save.img_stamp;
        const auto& t2 = msg->header.stamp;
        int64_t diff_ns = std::abs(
            (int64_t)t1.sec * 1000000000LL + (int64_t)t1.nanosec -
            (int64_t)t2.sec * 1000000000LL - (int64_t)t2.nanosec);
        return diff_ns < 1000000; // 1ms 容差
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

ArmorPlateIdentification::ArmorPlateIdentification() : Node("armor_plate_identification_node"), camera_driver_(this)
{
    init();
}

ArmorPlateIdentification::~ArmorPlateIdentification()
{
    camera_driver_.close();
    cv::destroyAllWindows();
}

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

        auto t_start = std::chrono::steady_clock::now();
        img_show_ = frame.clone();
        auto t0 = std::chrono::steady_clock::now();
        Identification(frame);
        auto t1 = std::chrono::steady_clock::now();
        NumberClassify();
        SolvePose();
        Publish();
        Save();
        auto t_end = std::chrono::steady_clock::now();
        process_time_ms_ = static_cast<float>(std::chrono::duration<double, std::milli>(t_end - t_start).count());

        float id_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        process_time_sum_ += process_time_ms_;
        id_time_sum_ += id_ms;
        process_time_count_++;
        if (process_time_count_ >= 50) {
            RCLCPP_INFO(this->get_logger(), "【平均用时】总:%.1fms  识别:%.1fms  [预处理:%.1fms  detectArmors:%.1fms]",
                process_time_sum_ / 50.0f, id_time_sum_ / 50.0f,
                id_split_sum_ / 50.0f, id_detect_sum_ / 50.0f);
            process_time_sum_ = 0.0f;
            id_time_sum_ = 0.0f;
            id_split_sum_ = 0.0f;
            id_detect_sum_ = 0.0f;
            process_time_count_ = 0;
        }

        ImageShow();
        controlParams();
        if (debug_base_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
        }
    }
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
