#pragma once
#include "armor_plate_tracker/MyKalmanFilter.hpp"

#include "armor_plate_interfaces/msg/armor_plates.hpp"
#include "armor_plate_interfaces/msg/armor_plate.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using armor_plate_interfaces::msg::ArmorPlate;
using armor_plate_interfaces::msg::ArmorPlates;
using geometry_msgs::msg::PoseStamped;


struct AngleRecord {
    builtin_interfaces::msg::Time time;
    // 单位是弧度
    float yaw_abs;
    float pitch_abs;
};


struct TrackingOverlayData {
    bool has_result = false;
    bool is_lost = false;
    Eigen::Vector3d measured_position;
    float measured_yaw = 0.0f;
    float measured_pitch = 0.0f;
    float filter_yaw = 0.0f;
    float filter_pitch = 0.0f;
    float distance = 0.0f;
};

class Tracker
{
private:
    // 原始模板滤波器（用于重置时复制） -- 世界坐标系
    MyKalmanFilter x_kf_origin_;
    MyKalmanFilter y_kf_origin_;
    MyKalmanFilter z_kf_origin_;
    
    // 实际使用的滤波器
    MyKalmanFilter x_kf_;
    MyKalmanFilter y_kf_;
    MyKalmanFilter z_kf_;
    
    // 当前滤波结果 -- 相机坐标系(增量角)
    float yaw_;
    float pitch_;
    // 绝对角度
    float yaw_abs_;
    float pitch_abs_;
    Eigen::Matrix3d R_w_c_; // R_{w<-c}
    Eigen::Matrix3d R_c_w_; // R_{c<-w}
    Eigen::Quaterniond q_w_c_;
    // 时间相关
    double last_update_time_;   // 上次更新时间（秒）
    double last_detection_time_;// 上次检测到目标的时间（秒）
    // 跟踪状态
    bool initialized_;          // 是否已初始化
    double max_lost_time_;      // 最大允许丢失时间（秒），默认0.5s
    bool is_lost_;              // 是否处于丢失状态
    // 突变检测阈值（度）
    float yaw_mutation_threshold_;
    float last_armor_pose_yaw_;
    // 当前帧选中的原始测量值（相机系）
    Eigen::Vector3d measured_position_camera_;
    float measured_yaw_;
    float measured_pitch_;
    // 获得世界坐标系下的点
    PoseStamped measured_position_world;
    PoseStamped filter_position_world;
    // 获得相机坐标系下的点
    Eigen::Vector3d measured_position_camera;
    Eigen::Vector3d filter_position_camera;
    // 初始化滤波器（内部调用）
    void initFilter(MyKalmanFilter& kf);
    
    // 根据dt更新状态转移矩阵
    void updateTransitionMatrix(MyKalmanFilter& kf, double dt);
    
    // 选择最佳匹配目标
    void selectBestMatch(const std::vector<ArmorPlate>& armor_plates, ArmorPlate& target_armor);
    
    // 检查是否突变
    bool isMutation(const float& armor_pose_yaw);
    
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
    void setMutationThreshold(float yaw_thresh);
    
    
    void Update(const std::vector<ArmorPlate>& armor_plates,
                double current_time,
                float yaw_abs, float pitch_abs
    );
    
    // 获取滤波后的值
    float getYaw() const { return yaw_; }
    float getPitch() const { return pitch_; }
    // 获取原始测量值（选中目标，相机系）
    float getMeasuredYaw() const { return measured_yaw_; }
    float getMeasuredPitch() const { return measured_pitch_; }
    Eigen::Vector3d getMeasuredPosition() const { return measured_position_camera_; }

    // 获取是否丢失目标
    bool isLost() const { return is_lost_; }
    // 获取上次更新时间
    double getLastUpdateTime() const { return last_update_time_; }

    // 获得世界坐标系下的点 (PoseStamped)
    PoseStamped getMeasuredPositionWorld() const { return measured_position_world; }
    PoseStamped getFilterPositionWorld() const { return filter_position_world; }
    // 获得相机坐标系下的点 (Eigen::Vector3d)
    Eigen::Vector3d getMeasuredPositionCamera() const { return measured_position_camera; }
    Eigen::Vector3d getFilterPositionCamera() const { return filter_position_camera; }
};

// 工具函数
PoseStamped poseFromEigen(const Eigen::Vector3d& tvec, const Eigen::Quaterniond& q);
float normalizeRadAngle(float rad);
float calculatePoseYaw(const Eigen::Quaterniond &q);
float calculateYaw(const Eigen::Vector3d& tvec);
float calculatePitch(const Eigen::Vector3d& tvec);
float calculateDistance(const Eigen::Vector3d& tvec);