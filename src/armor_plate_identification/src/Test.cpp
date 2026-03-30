// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/DrawTarget.hpp"
#include "armor_plate_identification/TestFunc.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include <algorithm>

class Test : public rclcpp::Node
{
private:
    cv::VideoCapture c;
    cv::Mat img_show;
    PairedLights lights;

    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        std::vector<std::vector<cv::Point>> valued_contours = lights.findLightsContours(img_thre);
        std::vector<std::vector<cv::Point2f>> all_lights = lights.findLightLines(valued_contours);
        // ===== 测试 ==== 
        
        // 预处理四图拼接显示
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);

        lights.findPairedLights(img_thre);
        lights.drawPairedLights(img_show);
        // ===============

        
    }
public:
    Test() : Node("test_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
        // 官方测试视频
        //c.open("/home/minzhi/ws05_fourth_assessment/src/armor_plate_identification/video/Blue.mp4");
        // 实拍视屏
        c.open("/home/minzhi/ws05_fourth_assessment/1.avi");

        cv::Mat frame;
        while(true)
        {
            c >> frame;
            if (frame.empty()) return;
            if (!rclcpp::ok()) return;
            img_show = frame.clone();
            // 图像处理
            Identification(frame);

            // 图片展示
            cv::imshow("img_show", img_show);

            // 按键控制
            int key = cv::waitKey(1);
            if (key == 27) break;
            else if (key == 'a') cv::waitKey(0);
        }
        RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Test>();
    rclcpp::shutdown();
    return 0;
}
