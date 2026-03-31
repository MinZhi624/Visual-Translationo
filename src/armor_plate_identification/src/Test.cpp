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

    rclcpp::TimerBase::SharedPtr timer_;

#ifdef DEBUG_BASE
    int play_delay_ms_ = 100; // 播放延迟，越大越慢
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

    void init(std::string video_path)
    {
        // 相机初始化
        c.open(video_path);
        // 匹配参数初始化
        lights.MAX_ANGLE_DIFF = 10.0f;
        lights.MIN_LENGTH_RATIO = 0.7f;
        lights.MIN_X_DIFF_RATIO = 1.75f;
        lights.MAX_Y_DIFF_RATIO = 0.4f;
        lights.MAX_DISTANCE_RATIO = 0.4f;
        lights.MIN_DISTANCE_RATIO = 0.1f;
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),  std::bind(&Test::info, this));
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "灯条匹配识别DEBUG模式开启");
#endif
#ifdef DEBUG_PREPROCESSING
        RCLCPP_INFO(this->get_logger(), "图像预处理DEBUG模式开启");
#endif
    }
    void info()
    {
        // 计时器 5s发布一次信息
#ifdef DEBUG_INDENTIFICATION
        RCLCPP_INFO(this->get_logger(), "MAX_ANGLE_DIFF: %f, MAX_Y_DIFF_RATIO: %f, MIN_DISTANCE_RATIO: %f, MAX_DISTANCE_RATIO: %f, MIN_LENGTH_RATIO: %f, MIN_X_DIFF_RATIO: %f",
        lights.MAX_ANGLE_DIFF, lights.MAX_Y_DIFF_RATIO, lights.MIN_DISTANCE_RATIO, lights.MAX_DISTANCE_RATIO, lights.MIN_LENGTH_RATIO, lights.MIN_X_DIFF_RATIO
        );
#endif
    }
    void Identification(cv::Mat& image)
    {
        cv::Mat mask = findTargetColor(image);
        cv::Mat img_thre = preProcessing(mask);
        
        // 直接调用 findPairedLights 完成检测和匹配
        lights.findPairedLights(img_thre);
        lights.drawPairedLights(img_show);
    
        
        // ===== 测试 ==== 
#ifdef DEBUG_PREPROCESSING
        // 预处理四图拼接显示
        // 绘制目标区域
        cv::Mat img_target;
        cv::bitwise_and(image, image, img_target, img_thre);
        std::vector<cv::Mat> images = {image, mask, img_thre, img_target};
        std::vector<std::string> labels = {"Original", "Color Mask", "Preprocessed", "Target Region"};
        showMultiImages("PreProcessions-View", images, labels);
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
        lights.drawAllLights(img_show);
#if defined(DEBUG_BASE)
        cv::putText(img_show, "1-6:select  T/G:adj  +/-:speed  P:pause  ESC:exit",
                    cv::Point(x, y + 6 * line_h + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
#else
        cv::putText(img_show, "1-6:select  T/G:adj  P:pause  ESC:exit",
                    cv::Point(x, y + 6 * line_h + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
#endif
#endif
#ifdef DEBUG_BASE
    std::string speed_text = "Delay: " + std::to_string(play_delay_ms_) + " ms";
    cv::putText(img_show, speed_text, cv::Point(x, y + 7 * line_h + 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
#endif
    }
    void controlParams()
    {
        int key = cv::waitKey(1);
        if (key == -1) return;

        // ESC：退出
        if (key == 27) {
            rclcpp::shutdown();
            return;
        }

        // P：暂停
        if (key == 'p' || key == 'P') {
            RCLCPP_INFO(this->get_logger(), "暂停，按任意键继续...");
            cv::waitKey(0);
            return;
        }

#ifdef DEBUG_BASE
        // +/-：速度控制
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
                case 0: // MAX_ANGLE_DIFF
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
    }
public:
    Test(std::string video_path) : Node("test_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "测试节点已经启动");
        init(video_path);
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
            // 打印
            rclcpp::spin_some(this->get_node_base_interface());
            // 控制速度
#ifdef DEBUG_BASE
            std::this_thread::sleep_for(std::chrono::milliseconds(play_delay_ms_));
#endif
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
