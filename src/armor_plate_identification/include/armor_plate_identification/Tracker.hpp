#pragma once
#include "armor_plate_identification/MyKalmanFilter.hpp"
#include <vector>
#include <cmath>

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
    bool selectBestMatch(const std::vector<float>& yaw_list,
                         const std::vector<float>& pitch_list,
                         float& out_yaw, float& out_pitch);
    
    // 检查是否突变
    bool isMutation(float measured_yaw, float measured_pitch);
    
    // 重置滤波器
    void resetFilter();
    
    // 检查是否丢失太久
    bool isLostTooLong(double current_time) const;

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
    // yaw_list, pitch_list: 检测到的所有装甲板的yaw和pitch（一一对应）
    // current_time: 当前时间戳（秒）
    void Update(const std::vector<float>& yaw_list, 
                const std::vector<float>& pitch_list,
                double current_time);
    
    // 获取滤波后的值
    float getYaw() const { return yaw_; }
    float getPitch() const { return pitch_; }
    
    // 获取是否丢失目标
    bool isLost() const { return is_lost_; }
    
    // 获取丢失时间（秒）
    double getLostTime(double current_time) const;
    
    // 获取上次更新时间
    double getLastUpdateTime() const { return last_update_time_; }
};
