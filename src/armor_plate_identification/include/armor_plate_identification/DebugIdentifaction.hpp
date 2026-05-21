#pragma once
#include <opencv2/core.hpp>
#include "rclcpp/logging.hpp"
#include "armor_plate_identification/Detector.hpp"
#include "armor_plate_interfaces/msg/tracker_debug.hpp"

/** @brief ROI 收集器：toggle 录制模式，收集 rejected 的号码 ROI，自动保存 */
class NumberRoiCollector {
public:
    void toggleRecording();
    void startRecording();
    void stopRecording();
    bool isRecording() const { return recording_; }

    /** @brief 录制中追加 rejected ROI，返回新增数量 */
    int feedRejected(const std::vector<cv::Mat>& rois);

    void save(const std::string& dir);
    void show();
    size_t count() const { return collected_.size(); }

private:
    bool recording_ = false;
    std::vector<cv::Mat> collected_;
    static int global_counter_;
    static int batch_counter_;
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

/** @brief 在图像上绘制所有装甲板（交叉线） */
void drawArmors(cv::Mat& img, const std::vector<Armor>& armors);
/** @brief 在图像上绘制所有灯条 */
void drawAllLights(cv::Mat& img, const std::vector<Light>& lights);
/** @brief 绘制旋转矩形 */
void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);
/** @brief 用四个点绘制旋转矩形 */
void drawRotatedRect(cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3, const cv::Point2f& p4, const cv::Scalar& color = cv::Scalar(207, 216, 129), int thickness = 2);

/**
 * @brief 将多个图像拼接显示在一个窗口中（2x2 布局）
 * @param window_name 窗口名称
 * @param imgs 图像向量，支持 3 或 4 张图
 * @param labels 标签文本（可选）
 */
void showMultiImages(const std::string& window_name,
                     const std::vector<cv::Mat>& imgs,
                     const std::vector<std::string>& labels = {});

/** @brief 调试控制器，封装播放速度控制和绘制 */
class DebugParamController {
public:
    /**
     * @brief 处理调试按键（+/- 调播放速度）
     * @param key cv::waitKey 返回的按键值
     * @param logger ROS2 日志器
     * @return true 表示按键已被消费
     */
    bool handleKey(int key, const rclcpp::Logger& logger);

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
    int play_delay_ms_ = 0;   // 播放延迟，越大越慢
    int x_ = 10;              // 绘制起点 x
    int y_ = 30;              // 绘制起点 y
    int line_h_ = 25;         // 行高
};
