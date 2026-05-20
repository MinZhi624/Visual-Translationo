#pragma once
#include <opencv2/core.hpp>
#include "rclcpp/logging.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

/** @brief ROI 收集器：按 S 键后连续收集 N 帧的号码 ROI，自动保存 */
class NumberRoiCollector {
public:
    void startCollect(int frames = 10);
    void feed(const std::vector<cv::Mat>& rois);
    bool isCollecting() const { return remaining_ > 0; }
    bool isDone() const { return remaining_ == 0 && !collected_.empty(); }
    void save(const std::string& dir);
    void show();
private:
    int remaining_ = 0;
    std::vector<cv::Mat> collected_;
    static int global_counter_;
};

/**
 * @brief 输出 TrackerDebug 消息到 ROS 日志
 * @param logger ROS2 日志器
 * @param msg TrackerDebug 消息
 */
void infoTrackerDebugMsg(const rclcpp::Logger& logger,
                         const armor_plate_interfaces::msg::TrackerDebug& msg);

/**
 * @brief 保存 TrackerDebug 消息到文件
 * @param log_dir 日志目录
 * @param msg TrackerDebug 消息
 */
void saveTrackerDebugToFile(const std::string& log_dir,
                            const armor_plate_interfaces::msg::TrackerDebug& msg);

/** @brief 关闭 TrackerDebug 日志文件 */
void closeTrackerDebugFile();

void drawNumberTest(cv::Mat& img, const Armor& armor);
void drawAllNumberTest(cv::Mat& img, const std::vector<Armor>& armors);

/**
 * @brief 将多个图像拼接显示在一个窗口中（2x2 布局）
 * @param window_name 窗口名称
 * @param imgs 图像向量，支持 3 或 4 张图
 * @param labels 标签文本（可选）
 */
void showMultiImages(const std::string& window_name,
                     const std::vector<cv::Mat>& imgs,
                     const std::vector<std::string>& labels = {});

/** @brief 调试参数控制器，封装 DEBUG_INDENTIFICATION / DEBUG_BASE 公共逻辑 */
class DebugParamController {
public:
    DebugParamController();

    /**
     * @brief 处理调试按键（1-6 选参数、T/G 调值、+/- 调播放速度）
     * @param key cv::waitKey 返回的按键值
     * @param lights 要调节的灯条匹配参数对象
     * @param logger ROS2 日志器
     * @return true 表示按键已被消费
     */
    bool handleKey(int key, Detector& lights, const rclcpp::Logger& logger);

    /**
     * @brief 在图像左上角第一行绘制处理用时
     * @param img 要绘制的图像
     * @param process_time_ms 图像从获取到处理完成的用时（毫秒），传负数则不显示
     */
    void drawProcessTime(cv::Mat& img, float process_time_ms);

    /**
     * @brief 在 process_time 正下方绘制当前播放延迟
     * @param img 要绘制的图像
     */
    void drawDelay(cv::Mat& img);

    /**
     * @brief 在图像左上角绘制 6 个可调参数（从第三行开始）
     * @param img 要绘制的图像
     * @param lights 灯条匹配参数对象
     */
    void drawParams(cv::Mat& img, const Detector& lights);

    /**
     * @brief 在图像上绘制键盘帮助文字
     * @param img 要绘制的图像
     * @param show_speed_control 是否显示 +/- speed 提示
     */
    void drawDebugInfo(cv::Mat& img, bool show_speed_control);

    /** @brief 获取当前播放延迟（ms） */
    int getPlayDelayMs() const { return play_delay_ms_; }

    /**
     * @brief 手动设置播放延迟（ms）
     * @param ms 延迟毫秒数
     */
    void setPlayDelayMs(int ms) { play_delay_ms_ = std::max(ms, 0); }

private:
    int selected_param_;
    std::vector<std::string> param_names_;
    int play_delay_ms_ = 0;   // 播放延迟，越大越慢
    int x_ = 10;              // 绘制起点 x
    int y_ = 30;              // 绘制起点 y
    int line_h_ = 25;         // 行高
};
