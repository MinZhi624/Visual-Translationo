#include "armor_plate_identification/CameraDriver.hpp"
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
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
    int x = 10, y = 30, line_h = 25;
#endif

    DebugParamController debug_controller_;

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
        lights.MIN_LENGTH_RATIO = 0.5f;
        lights.MIN_X_DIFF_RATIO = 0.75f;
        lights.MAX_Y_DIFF_RATIO = 1.0f;
        lights.MAX_DISTANCE_RATIO = 0.8f;
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
#endif
#ifdef DEBUG_INDENTIFICATION
        debug_controller_.drawParams(img_show, lights, x, y, line_h);
        lights.drawAllLights(img_show);
        debug_controller_.drawDebugInfo(img_show, play_delay_ms_, true, x, y, line_h);
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
        if (debug_controller_.handleKey(key, lights, play_delay_ms_, this->get_logger())) {
            return;
        }
#endif
        // 相机参数控制
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
        // 在 img_show 左下角显示相机参数
        std::string params_text = "Exp: " + std::to_string(exposure_time) + "us  Gain: " + std::to_string(gain);
        // 左下角位置（留 10px 边距）
        cv::Point pos(10, img_show.rows - 10);
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
