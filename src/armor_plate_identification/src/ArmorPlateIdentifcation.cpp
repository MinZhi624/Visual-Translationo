#include "armor_plate_identification/CameraDriver.hpp"
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/TestFunc.hpp"

#include <opencv2/highgui.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

class ArmorPlateIdentification : public rclcpp::Node
{
private:
    ArmorCameraCapture camera;
    cv::Mat img_show;

    // 可调参数
    int exposure_time = 300;  // 初始曝光时间（微秒）
    int gain = 80;             // 初始增益

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
        
        // 操作说明
        RCLCPP_INFO(this->get_logger(), "相机启动成功");
        RCLCPP_INFO(this->get_logger(), "控制键：W/S-增减曝光  A/D-增减增益 Q/ESC-退出");
    }
    // 识别
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        PairedLights lights;
        lights.findPairedLights(img_thre);
        
        // ===== 测试 ==== 
        // 标注画图
        lights.drawPairedLights(img_show);
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        
        // 四图拼接显示
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("Identify-View", images, labels);
        // ==============
    }
    // 控制参数函数
    void controlParams()
    {
        // 键盘控制
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') 
        {
            camera.release();
            cv::destroyAllWindows();
            exit(0);
        }
        // 曝光时间调整 (100us 步进)
        else if (key == 'w' || key == 'W') {
            exposure_time = std::min(exposure_time + 100, 10000);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
        }
        else if (key == 's' || key == 'S') {
            exposure_time = std::max(exposure_time - 100, 100);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "曝光时间: %d us", exposure_time);
        }
        // 增益调整 (10 步进)
        else if (key == 'a' || key == 'A') {
            gain = std::min(gain + 10, 400);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
        }
        else if (key == 'd' || key == 'D') {
            gain = std::max(gain - 10, 0);
            camera.setLowExposureForLightBar(exposure_time, gain);
            RCLCPP_INFO(this->get_logger(), "增益: %d", gain);
        }
    }
    // 可视化
    void ImageShow()
    {
        // 在 img_show 左上角显示相机参数
        std::string params_text = "Exposure: " + std::to_string(exposure_time) + 
                                  " us  Gain: " + std::to_string(gain);
        cv::putText(img_show, params_text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        
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
