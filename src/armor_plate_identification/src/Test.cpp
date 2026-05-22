// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Test.hpp"
#include <rclcpp/logging.hpp>
void Test::run()
{
    cv::Mat frame;
    while(rclcpp::ok())
    {
        c_ >> frame;
        if (frame.empty()) {
            closeTrackerDebugFile();
            RCLCPP_INFO(this->get_logger(), "视频播放结束");
            rclcpp::shutdown();
            return;
        }
        img_show_ = frame.clone();
        auto t_start = std::chrono::steady_clock::now();

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

        if (!headless_) {
            imageShow();
            controlParams();
            if (debug_base_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(debug_controller_.getPlayDelayMs()));
            }
        }
        frame_count_++;

        // 帧调试：播放到指定帧数时结束
        if (debug_frame_ && frame_count_ >= debug_frame_count_) {
            closeTrackerDebugFile();
            RCLCPP_INFO(this->get_logger(), "帧调试：已播放到第 %d 帧，结束", debug_frame_count_);
            cv::destroyAllWindows();
            rclcpp::shutdown();
            return;
        }
    }
    closeTrackerDebugFile();
    RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    cv::destroyAllWindows();
    rclcpp::shutdown();
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
    // 从视频路径提取文件名作为测试名称（去掉扩展名）
    std::filesystem::path vp(video_path);
    test_name_ = vp.stem().string();
    RCLCPP_INFO(this->get_logger(), "测试名称: %s", test_name_.c_str());
    // 获取视频FPS
    double fps = c_.get(cv::CAP_PROP_FPS);
    if (fps <= 0) {
        RCLCPP_WARN(this->get_logger(), "无法获取视频FPS，使用默认值: 50.0");
        fps_ = 50.0;
    } else {
        RCLCPP_INFO(this->get_logger(), "视频FPS: %.2f", fps);
        fps_ = fps;
    }
    // ===== 初始化数字识别（通过 Detector 内部的 classifier） ===== //
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    float number_threshold = static_cast<float>(this->declare_parameter<double>("number_threshold", 0.15));
    // ==== Detector 参数 ==== //
    /*
        c++20 的特性，可以直接使用结构体初始化，不需要再写一堆赋值语句
    */
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
    // ===== 初始化PoseSolver ===== //
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        2374.54248, 0., 698.85288,
        0., 2377.53648, 520.8649,
        0., 0., 1.);
    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) <<
        -0.059743, 0.355479, -0.000625, 0.001595, 0.000000);
    pose_solver_ = PoseSolver(camera_matrix, distortion_coefficients);
    // ===== 初始化发布器 ===== //
    armor_plates_pub_ = this->create_publisher<ArmorPlates>("armor_plates", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // ===== DEBUG ===== //
    debug_base_ = this->declare_parameter<bool>("debug_base", false);
    debug_identification_ = this->declare_parameter<bool>("debug_identification", false);
    debug_preprocessing_ = this->declare_parameter<bool>("debug_preprocessing", false);
    debug_number_classification_ = this->declare_parameter<bool>("debug_number_classification", false);
    debug_frame_ = this->declare_parameter<bool>("debug_frame", false);
    debug_frame_count_ = this->declare_parameter<int>("debug_frame_count", 100);
    headless_ = this->declare_parameter<bool>("headless", false);
    int delay_time = this->declare_parameter<int>("delay_time", 20);
    debug_controller_.setPlayDelayMs(delay_time);
    tracker_debug_sub_ = this->create_subscription<TrackerDebug>(
        "tracker_debug", 10,
        std::bind(&Test::TrackerDebugCallBack, this, std::placeholders::_1)
    );

    if (target_color_ == "BLUE") RCLCPP_INFO(this->get_logger(), "目标颜色为蓝色");
    if (target_color_ == "RED") RCLCPP_INFO(this->get_logger(), "目标颜色为红色");
    if (debug_identification_) RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
    if (debug_preprocessing_) RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
    if (debug_number_classification_) RCLCPP_INFO(this->get_logger(), "数字识别DEBUG模式开启");
    if (debug_frame_) RCLCPP_INFO(this->get_logger(), "帧调试模式开启，将在第 %d 帧结束", debug_frame_count_);
    if (headless_) RCLCPP_INFO(this->get_logger(), "无头模式：跳过所有 GUI 窗口");
}

void Test::Identification(cv::Mat& img_bgr)
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
        if (!headless_) {
            std::vector<cv::Mat> images = {img_bgr, blue_debug, preprocess_debug_.gray_thre, preprocess_debug_.merged_thre};
            std::vector<std::string> labels = {"Original", "BLUE_dim (fragments)", "GRAY_thre", "Merged"};
            showMultiImages("PreProcessions-View", images, labels);
        }
    }
    if (debug_identification_) {
        drawAllLights(img_show_, lights_.getLights());
        debug_controller_.drawDebugInfo(img_show_, debug_base_);
    }
}

void Test::NumberClassify()
{
    ////////// DEBUG /////////
    if (debug_number_classification_) {
        drawAllNumberTest(img_show_, armors_);
    }
}

void Test::SolvePose()
{
    std::vector<ArmorPlate> armor_plates;
    for (auto& armor : armors_) {
        pose_solver_.solve(armor);
        ArmorPlate armor_plate;
        armor_plate.pose.position.x = armor.xyz_gimbal_.x();
        armor_plate.pose.position.y = armor.xyz_gimbal_.y();
        armor_plate.pose.position.z = armor.xyz_gimbal_.z();
        armor_plate.pose.orientation.x = armor.q_gimbal_.x();
        armor_plate.pose.orientation.y = armor.q_gimbal_.y();
        armor_plate.pose.orientation.z = armor.q_gimbal_.z();
        armor_plate.pose.orientation.w = armor.q_gimbal_.w();
        armor_plate.number = static_cast<int>(armor.name_);
        armor_plate.image_distance_to_center = armor.image_distance_to_center_;
        armor_plates.push_back(armor_plate);
    }
    armor_plates_ = armor_plates;
}

void Test::Publish()
{
    double time_sec = static_cast<double>(frame_count_) / fps_;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(time_sec);
    stamp.nanosec = static_cast<uint32_t>((time_sec - stamp.sec) * 1e9);

    // 发布装甲板数据
    ArmorPlates armor_plates_msg;
    armor_plates_msg.header.stamp = stamp;
    armor_plates_msg.header.frame_id = "camera_link";
    armor_plates_msg.armor_plates = armor_plates_;
    armor_plates_pub_->publish(armor_plates_msg);

    // 发布相机坐标系下的TF（使用solvePnP解算的完整旋转）
    for (size_t i = 0; i < armor_plates_.size(); ++i) {
        const auto& plate = armor_plates_[i];
        const auto& q = armors_[i].q_gimbal_;
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "camera_link";
        tf.child_frame_id = "armor_" + std::to_string(i);
        tf.transform.translation.x = plate.pose.position.x;
        tf.transform.translation.y = plate.pose.position.y;
        tf.transform.translation.z = plate.pose.position.z;
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(tf);
    }
}

void Test::controlParams()
{
    int key = cv::waitKey(1);
    if (key == -1) return;

    // ESC / q / Q：退出
    if (key == 27 || key == 'q' || key == 'Q') {
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return;
    }
    // P：暂停
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

void Test::imageShow()
{
    // 绘制处理用时
    debug_controller_.drawProcessTime(img_show_, process_time_ms_);
    if (debug_base_) {
        debug_controller_.drawDelay(img_show_);
    }
    if (debug_number_classification_) {
        roi_collector_.show();
    }
    // 显示图像
    cv::Mat show_img;
    cv::resize(img_show_, show_img, cv::Size(), 0.5, 0.5);
    cv::imshow("Identifacation", img_show_);
}

void Test::Save()
{
    {
        std::lock_guard<std::mutex> tracker_debug_lock(tracker_debug_mutex_);
        double time_sec = static_cast<double>(frame_count_) / fps_;
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(time_sec);
        stamp.nanosec = static_cast<uint32_t>((time_sec - stamp.sec) * 1e9);
        img_buffs_.push_back({stamp, img_show_.clone()});
        if (img_buffs_.size() > 10) img_buffs_.pop_front();
    }
}

void Test::TrackerDebugCallBack(const TrackerDebug::SharedPtr msg)
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
    if (!headless_) {
        cv::Mat show_img;
        cv::resize(debug_img, show_img, cv::Size(), 0.5, 0.5);
        cv::imshow("Tracker Debug", show_img);
        cv::waitKey(1);
    }

    // 输出 TrackerDebug 数据
    // infoTrackerDebugMsg(rclcpp::get_logger("TRACKER_DEBUG"), *msg);

    std::string method = msg->method;
    if (method.empty()) method = "unknown";
    std::string share_dir = ament_index_cpp::get_package_share_directory("armor_plate_identification");
    std::string workspace_dir = std::filesystem::path(share_dir).parent_path().parent_path().parent_path().parent_path().string();
    std::string log_dir = workspace_dir + "/Debug/Tracker/" + test_name_ + "/" + method + "/temp";
    saveTrackerDebugToFile(log_dir, *msg);
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
    if(spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
