#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "armor_plate_identification/PairedLights.hpp"

///  @brief 将多个图像拼接显示在一个窗口中（2x2 布局）
///  @param window_name 窗口名称
///  @param images 图像向量，支持 3 或 4 张图
///  @param labels 标签文本（可选）
void showMultiImages(const std::string& window_name, 
                     const std::vector<cv::Mat>& images,
                     const std::vector<std::string>& labels = {});

///  @brief 调试参数控制器，封装 DEBUG_INDENTIFICATION / DEBUG_BASE 公共逻辑
class DebugParamController {
public:
    DebugParamController();

    ///  @brief 处理调试按键（1-6 选参数、T/G 调值、+/- 调播放速度）
    ///  @param key cv::waitKey 返回的按键值
    ///  @param lights 要调节的灯条匹配参数对象
    ///  @param play_delay_ms 播放延迟（引用，可能被修改）
    ///  @param logger ROS2 日志器
    ///  @return true 表示按键已被消费
    bool handleKey(int key, PairedLights& lights, int& play_delay_ms, const rclcpp::Logger& logger);

    ///  @brief 在图像左上角绘制 6 个可调参数及其当前值
    void drawParams(cv::Mat& img, const PairedLights& lights, int x, int y, int line_h);

    ///  @brief 在图像上绘制帮助文字与播放延迟
    ///  @param show_speed_control 是否显示 +/- speed 提示和 Delay 数值
    void drawDebugInfo(cv::Mat& img, int play_delay_ms, bool show_speed_control, int x, int y, int line_h);

private:
    int selected_param_;
    std::vector<std::string> param_names_;
};
