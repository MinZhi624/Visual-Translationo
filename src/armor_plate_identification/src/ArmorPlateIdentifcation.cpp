#include "armor_plate_identification/CameraDriver.hpp"
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/DrawTarget.hpp"
#include "armor_plate_identification/TestFunc.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    ArmorCameraCapture camera;
    cv::Mat img_show;
    PairedLights lights;

    // 可调参数
    int exposure_time = 300;  // 初始曝光时间（微秒）
    int gain = 20;             // 初始增益

    rclcpp::TimerBase::SharedPtr timer_;

#ifdef DEBUG_BASE
    int play_delay_ms_ = 30; // 播放延迟，越大越慢
#endif
    int x = 10, y = 30, line_h = 25;

#ifdef DEBUG_INDENTIFICATION
    int selected_param_ = 0; // 0~5 对应6个匹配参数
    const std::vector<std::string> param_names_ = {
        "MAX_ANGLE_DIFF",
        "MAX_Y_DIFF_RATIO",
        "MIN_DISTANCE_RATIO",
        "MAX_DISTANCE_RATIO",
        "MIN_LENGTH_RATIO",
        "MIN_X_DIFF_RATIO"
    };
#endif

    // 初始化
    void init()
    {
        // 打开相机
        if (!camera.open()) {
            RCLCPP_ERROR(this->get_logger(), "相机打开失败,请检查相机是否连接正确！");
            return;
        }
        
        // 设置为灯条识别优化的低曝光模式
        if (camera.setLowExposureForLightBar(exposure_time, gain)) {
            RCLCPP_INFO(this->get_logger(), "低曝光模式设置成功，当前曝光: %.0f us, 增益: %d", 
                        camera.getExposureTime(), camera.getGain());
        } else {
            RCLCPP_WARN(this->get_logger(), "低曝光模式设置失败");
        }

        // 匹配参数初始化
        lights.MAX_ANGLE_DIFF = 10.0f;
        lights.MIN_LENGTH_RATIO = 0.7f;
        lights.MIN_X_DIFF_RATIO = 1.75f;
        lights.MAX_Y_DIFF_RATIO = 0.4f;
        lights.MAX_DISTANCE_RATIO = 0.4f;
        lights.MIN_DISTANCE_RATIO = 0.1f;
        
        // 创建定时器，5秒发布一次信息
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 
            std::bind(&ArmorPlateIdentification::info, this));
        
        // 操作说明
        RCLCPP_INFO(this->get_logger(), "相机启动成功");
        RCLCPP_INFO(this->get_logger(), "通用控制：ESC-退出  P-暂停  W/S-曝光  A/D-增益");
        
#if defined(DEBUG_INDENTIFICATION) && defined(DEBUG_BASE)
        RCLCPP_INFO(this->get_logger(), "DEBUG模式：1-6选参数  T/G调值  +/-调速度");
#elif defined(DEBUG_INDENTIFICATION)
        RCLCPP_INFO(this->get_logger(), "DEBUG模式：1-6选参数  T/G调值");
#elif defined(DEBUG_BASE)
        RCLCPP_INFO(this->get_logger(), "DEBUG模式：+/-调速度");
#endif
    }

    // 定时器回调，发布信息
    void info()
    {
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %.2f, MAX_Y_DIFF_RATIO: %.2f, MIN_DISTANCE_RATIO: %.2f, MAX_DISTANCE_RATIO: %.2f, MIN_LENGTH_RATIO: %.2f, MIN_X_DIFF_RATIO: %.2f",
        lights.MAX_ANGLE_DIFF, lights.MAX_Y_DIFF_RATIO, lights.MIN_DISTANCE_RATIO, lights.MAX_DISTANCE_RATIO, lights.MIN_LENGTH_RATIO, lights.MIN_X_DIFF_RATIO
        );
#endif
    }

    // 识别
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights.findPairedLights(img_thre);
        lights.drawPairedLights(img_show);
        
        // 图像显示逻辑
#ifdef DEBUG_PREPROCESSING
        // DEBUG_PREPROCESSING 模式：显示预处理四图
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);
#else
        // 非 DEBUG_PREPROCESSING 模式：也显示基础四图（便于观察）
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("Identify-View", images, labels);
#endif
#ifdef DEBUG_INDENTIFICATION
        // 在img_show显示6个参数
        for (int i = 0; i < 6; ++i) {
            float val = 0.0f;
            switch (i) {
                case 0: val = lights.MAX_ANGLE_DIFF; break;
                case 1: val = lights.MAX_Y_DIFF_RATIO; break;
                case 2: val = lights.MIN_DISTANCE_RATIO; break;
                case 3: val = lights.MAX_DISTANCE_RATIO; break;
                case 4: val = lights.MIN_LENGTH_RATIO; break;
                case 5: val = lights.MIN_X_DIFF_RATIO; break;
            }
            std::string text = param_names_[i] + ": " + std::to_string(val);
            // 保留2位小数
            text = text.substr(0, text.find('.') + 3);
            cv::Scalar color = (i == selected_param_) ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 255);
            cv::putText(img_show, text, cv::Point(x, y + i * line_h),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }
        std::string help_text;
        lights.drawAllLights(img_show);
#if defined(DEBUG_BASE)
        help_text = "1-6:select  T/G:adj  +/-:speed  P:pause  ESC:exit";
#else
        help_text = "1-6:select  T/G:adj  P:pause  ESC:exit";
#endif
        cv::putText(img_show, help_text, cv::Point(x, y + 6 * line_h + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
#endif

#ifdef DEBUG_BASE
        std::string speed_text = "Delay: " + std::to_string(play_delay_ms_) + " ms";
        cv::putText(img_show, speed_text, cv::Point(x, y + 7 * line_h + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
#endif
    }

    // 控制参数函数
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;
        
        // ESC：退出（最高优先级）
        if (key == 27) 
        {
            camera.release();
            cv::destroyAllWindows();
            exit(0);
        }
        
        // P：暂停（通用功能）
        if (key == 'p' || key == 'P') {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
            return;
        }

#ifdef DEBUG_BASE
        // +/-：速度控制（DEBUG_BASE模式）
        if (key == '+' || key == '=') {
            play_delay_ms_ = std::max(play_delay_ms_ - 10, 0);
            RCLCPP_INFO(this->get_logger(), "播放延迟: %d ms", play_delay_ms_);
            return;
        }
        if (key == '-' || key == '_') {
            play_delay_ms_ += 10;
            RCLCPP_INFO(this->get_logger(), "播放延迟: %d ms", play_delay_ms_);
            return;
        }
#endif

#ifdef DEBUG_INDENTIFICATION
        // DEBUG_INDENTIFICATION 模式：调节灯条匹配参数
        
        // 1-6：选择参数
        if (key >= '1' && key <= '6') {
            selected_param_ = key - '1';
            RCLCPP_INFO(this->get_logger(), "选中参数: %s", param_names_[selected_param_].c_str());
            return;
        }
        
        // T/G：调节参数值
        bool increase = (key == 't' || key == 'T');
        bool decrease = (key == 'g' || key == 'G');
        
        if (increase || decrease) {
            float dir = increase ? 1.0f : -1.0f;
            switch (selected_param_) {
                case 0: // MAX_ANGLE_DIFF (角度差，范围 0-30 度)
                    lights.MAX_ANGLE_DIFF = std::clamp(lights.MAX_ANGLE_DIFF + dir * 0.5f, 0.0f, 30.0f);
                    break;
                case 1: // MAX_Y_DIFF_RATIO
                    lights.MAX_Y_DIFF_RATIO = std::max(lights.MAX_Y_DIFF_RATIO + dir * 0.05f, 0.0f);
                    break;
                case 2: // MIN_DISTANCE_RATIO
                    lights.MIN_DISTANCE_RATIO = std::max(lights.MIN_DISTANCE_RATIO + dir * 0.1f, 0.0f);
                    break;
                case 3: // MAX_DISTANCE_RATIO
                    lights.MAX_DISTANCE_RATIO = std::max(lights.MAX_DISTANCE_RATIO + dir * 0.1f, 0.0f);
                    break;
                case 4: // MIN_LENGTH_RATIO
                    lights.MIN_LENGTH_RATIO = std::clamp(lights.MIN_LENGTH_RATIO + dir * 0.05f, 0.0f, 1.0f);
                    break;
                case 5: // MIN_X_DIFF_RATIO
                    lights.MIN_X_DIFF_RATIO = std::max(lights.MIN_X_DIFF_RATIO + dir * 0.05f, 0.0f);
                    break;
            }
            return;
        }
#endif

        // 相机参数控制（所有模式都支持）
        // W/S：曝光时间
        // 非 DEBUG_INDENTIFICATION 模式：调节相机参数
        
        // W/S：曝光时间
        if (key == 'w' || key == 'W') {
            exposure_time = std::min(exposure_time + 100, 10000);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
            return;
        }
        if (key == 's' || key == 'S') {
            exposure_time = std::max(exposure_time - 100, 100);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
            return;
        }
        
        // A/D：增益
        if (key == 'a' || key == 'A') {
            gain = std::min(gain + 10, 400);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
            return;
        }
        if (key == 'd' || key == 'D') {
            gain = std::max(gain - 10, 0);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
            return;
        }
    }

    // 可视化
    void ImageShow()
    {
        // 获取图像尺寸
        int img_h = img_show.rows;
        int img_w = img_show.cols;
        
        // 在 img_show 左下角显示相机参数
        std::string params_text = "Exp: " + std::to_string(exposure_time) + "us  Gain: " + std::to_string(gain);
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(params_text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
        // 左下角位置（留 10px 边距）
        cv::Point pos(10, img_h - 10);
        cv::putText(img_show, params_text, pos,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        // 显示带参数的检测结果图
        cv::imshow("Detection Result", img_show);
    }
    
public:
    ArmorPlateIdentification() : Node("armor_plate_identification_node_cpp")
    {
        init();
        cv::Mat frame;
        while (true)
        {
            if (!camera.read(frame)) break;
            if (!rclcpp::ok()) break;
            
            // 图像处理
            img_show = frame.clone();
            Identification(frame);
            
            // 可视化
            ImageShow();
            controlParams();

            // 让 ROS2 处理定时器等事件
            rclcpp::spin_some(this->get_node_base_interface());

            // 控制速度
#ifdef DEBUG_BASE
            std::this_thread::sleep_for(std::chrono::milliseconds(play_delay_ms_));
#endif
        }
        camera.release();
        cv::destroyAllWindows();
    }
    
    ~ArmorPlateIdentification()
    {
        camera.release();
        cv::destroyAllWindows();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlateIdentification>());
    rclcpp::shutdown();
    return 0;
}
