// 这个主要是一个测试文件，在没有相机的时候测试
#include "armor_plate_identification/Recognize.hpp"
#include "armor_plate_identification/PairedLights.hpp"
#include "armor_plate_identification/DrawTarget.hpp"
#include "armor_plate_identification/TestFunc.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <thread>
#include <chrono>

class Test : public rclcpp::Node
{
private:
    cv::VideoCapture c;
    cv::Mat img_show;
    PairedLights lights;
    int selected_param_ = 0; // 0~5 对应6个匹配参数
    int play_delay_ms_ = 100; // 播放延迟，越大越慢
    const std::vector<std::string> param_names_ = {
        "MAX_ANGLE_DIFF",
        "MAX_Y_DIFF_RATIO",
        "MIN_DISTANCE_RATIO",
        "MAX_DISTANCE_RATIO",
        "MIN_LENGTH_RATIO",
        "MIN_X_DIFF_RATIO"
    };

    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights.findPairedLights(img_thre);
        // ===== 测试 ==== 
        // 预处理四图拼接显示
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);
        lights.drawPairedLights(img_show);

        // 在img_show显示6个参数
        int x = 10, y = 30, line_h = 25;
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
            cv::Scalar color = (i == selected_param_) ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 255);
            cv::putText(img_show, text, cv::Point(x, y + i * line_h),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }
        cv::putText(img_show, "1-6:select  W/S:adj  +/-:speed  A:pause  ESC:exit",
                    cv::Point(x, y + 6 * line_h + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        std::string speed_text = "Delay: " + std::to_string(play_delay_ms_) + " ms";
        cv::putText(img_show, speed_text, cv::Point(x, y + 7 * line_h + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        // ===============
    }
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == 27) { rclcpp::shutdown(); return; }
        else if (key == 'a' || key == 'A') { cv::waitKey(0); return; }

        // 1-6 切换选中参数
        if (key >= '1' && key <= '6') {
            selected_param_ = key - '1';
            return;
        }

        // 速度控制（必须在 W/S 判断之前）
        if (key == '+' || key == '=') {
            play_delay_ms_ = std::max(play_delay_ms_ - 30, 0);
            return;
        } else if (key == '-' || key == '_') {
            play_delay_ms_ += 30;
            return;
        }

        bool up = (key == 'w' || key == 'W');
        bool down = (key == 's' || key == 'S');
        if (!up && !down) return;

        float dir = up ? 1.0f : -1.0f;
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
    }
public:
    Test(std::string video_path) : Node("test_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
        c.open(video_path);
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
            controlParams();
            // 控制速度
            std::this_thread::sleep_for(std::chrono::milliseconds(play_delay_ms_));
        }
        RCLCPP_INFO(this->get_logger(), "测试节点已经结束");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请输入正确的参数，输入视屏地址");
        return 1;
    }
    auto node = std::make_shared<Test>(argv[1]);
    rclcpp::shutdown();
    return 0;
}
