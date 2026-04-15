#pragma once
#include "armor_plate_tracker/MyKalmanFilter.hpp"
#include <opencv2/core.hpp>

struct TrackingOverlayData {
    bool has_result = false;
    bool is_lost = false;
    cv::Vec3d measured_position;
    float measured_yaw = 0.0f;
    float measured_pitch = 0.0f;
    float filter_yaw = 0.0f;
    float filter_pitch = 0.0f;
    float distance = 0.0f;
};

class Tracker
{
private:
    // 原始模板滤波器（用于重置时复制）
    MyKalmanFilter yaw_kf_origin_;
    MyKalmanFilter pitch_kf_origin_;
    
    // 实际使用的滤波器
    MyKalmanFilter yaw_kf_;
    MyKalmanFilter pitch_kf_;
    
    // 当前滤波结果
    float yaw_;
    float pitch_;
    
    // 当前测量值（原始值）
    float measured_yaw_;
    float measured_pitch_;
    
    // 当前测量对应的原始位置（tvec，单位米）
    cv::Vec3d tvec_;
    
    // 时间相关
    double last_update_time_;   // 上次更新时间（秒）
    double last_detection_time_;// 上次检测到目标的时间（秒）
    
    // 跟踪状态
    bool initialized_;          // 是否已初始化
    double max_lost_time_;      // 最大允许丢失时间（秒），默认0.5s
    bool is_lost_;              // 是否处于丢失状态
    
    // 突变检测阈值（度）
    float yaw_mutation_threshold_;
    float pitch_mutation_threshold_;
    
    // 初始化滤波器（内部调用）
    void initFilter(MyKalmanFilter& kf);
    
    // 根据dt更新状态转移矩阵
    void updateTransitionMatrix(MyKalmanFilter& kf, double dt);
    
    // 选择最佳匹配目标
    bool selectBestMatch(const std::vector<cv::Vec3d>& positions,
                         const std::vector<float>& image_distances,
                         float& out_yaw, float& out_pitch,
                         cv::Vec3d& out_position);
    
    // 检查是否突变
    bool isMutation(float measured_yaw, float measured_pitch);
    
    // 重置滤波器
    void resetFilter();
    
    // 检查是否丢失太久
    bool isLostTooLong(double current_time) const;

    // 从tvec计算yaw/pitch/distance（放在Tracker内部）
    static float calculateYaw(float tx, float ty, float tz);
    static float calculatePitch(float tx, float ty, float tz);
    static float calculateDistance(float tx, float ty, float tz);

public:
    // 默认构造函数
    Tracker();
    
    // 初始化
    void Init();
    
    // 设置最大丢失时间（秒）
    void setMaxLostTime(double seconds);
    
    // 设置突变阈值
    void setMutationThreshold(float yaw_thresh, float pitch_thresh);
    
    // 主更新函数
    // positions: 检测到的所有装甲板的pose.position（即tvec，单位米）
    // image_distances: 对应的image_distance_to_center
    // current_time: 当前时间戳（秒）
    void Update(const std::vector<cv::Vec3d>& positions,
                const std::vector<float>& image_distances,
                double current_time);
    
    // 获取滤波后的值
    float getYaw() const { return yaw_; }
    float getPitch() const { return pitch_; }
    
    // 获取原始测量值
    float getMeasuredYaw() const { return measured_yaw_; }
    float getMeasuredPitch() const { return measured_pitch_; }
    
    // 获取原始测量对应的tvec位置
    cv::Vec3d getMeasuredPosition() const { return tvec_; }
    
    // 获取是否丢失目标
    bool isLost() const { return is_lost_; }
    
    // 获取丢失时间（秒）
    double getLostTime(double current_time) const;
    
    // 获取上次更新时间
    double getLastUpdateTime() const { return last_update_time_; }
};
