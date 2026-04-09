#include "armor_plate_identification/Tracker.hpp"
#include <algorithm>
#include <limits>

// 默认构造函数
Tracker::Tracker()
    : yaw_kf_origin_(3, 1), pitch_kf_origin_(3, 1)
    , yaw_kf_(3, 1), pitch_kf_(3, 1)
    , yaw_(0.0f), pitch_(0.0f)
    , last_update_time_(0.0)
    , last_detection_time_(0.0)
    , initialized_(false)
    , max_lost_time_(0.5)  // 默认最大丢失0.5秒
    , is_lost_(false)
    , yaw_mutation_threshold_(3.0f)   // yaw突变阈值3度
    , pitch_mutation_threshold_(2.0f) // pitch突变阈值2度
{
}

// 初始化滤波器参数
void Tracker::Init()
{
    // 初始化yaw滤波器
    initFilter(yaw_kf_origin_);
    initFilter(yaw_kf_);
    
    // 初始化pitch滤波器
    initFilter(pitch_kf_origin_);
    initFilter(pitch_kf_);
    
    initialized_ = false;
    last_update_time_ = 0.0;
    last_detection_time_ = 0.0;
    is_lost_ = false;
    yaw_ = 0.0f;
    pitch_ = 0.0f;
}

// 设置最大丢失时间（秒）
void Tracker::setMaxLostTime(double seconds)
{
    max_lost_time_ = seconds;
}

// 设置突变阈值
void Tracker::setMutationThreshold(float yaw_thresh, float pitch_thresh)
{
    yaw_mutation_threshold_ = yaw_thresh;
    pitch_mutation_threshold_ = pitch_thresh;
}

// 初始化单个滤波器（三阶模型：位置-速度-加速度）
// 注意：这里不设置dt相关的转移矩阵，等到Update时根据实际dt设置
void Tracker::initFilter(MyKalmanFilter& kf)
{
    // 测量矩阵 H (1x3) - 只观测位置
    Eigen::MatrixXf H(1, 3);
    H << 1.0f, 0.0f, 0.0f;
    kf.setMeasurementMatrix(H);
    
    // 过程噪声协方差 Q (3x3)
    Eigen::MatrixXf Q(3, 3);
    Q << 0.01f, 0.0f, 0.0f,
         0.0f, 0.1f, 0.0f,
         0.0f, 0.0f, 1.0f;
    kf.setProcessNoiseCov(Q);
    
    // 测量噪声协方差 R (1x1)
    Eigen::MatrixXf R(1, 1);
    R << 0.05f;
    kf.setMeasurementNoiseCov(R);
    
    // 初始化状态（设为0）
    Eigen::MatrixXf state(3, 1);
    state << 0.0f, 0.0f, 0.0f;
    kf.setStatePost(state);
    kf.setStatePre(state);
    
    // 初始化误差协方差 P (单位矩阵）
    Eigen::MatrixXf P(3, 3);
    P << 1.0f, 0.0f, 0.0f,
         0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 1.0f;
    kf.setErrorCovPost(P);
}

// 根据dt更新状态转移矩阵
void Tracker::updateTransitionMatrix(MyKalmanFilter& kf, double dt)
{
    Eigen::MatrixXf A(3, 3);
    float dt_f = static_cast<float>(dt);
    float dt2_half = 0.5f * dt_f * dt_f;
    A << 1.0f, dt_f, dt2_half,
         0.0f, 1.0f, dt_f,
         0.0f, 0.0f, 1.0f;
    kf.setTransitionMatrix(A);
}

// 选择最佳匹配目标
bool Tracker::selectBestMatch(const std::vector<float>& yaw_list,
                              const std::vector<float>& pitch_list,
                              float& out_yaw, float& out_pitch)
{
    if (yaw_list.empty() || pitch_list.empty() || 
        yaw_list.size() != pitch_list.size()) {
        return false;
    }
    
    // 如果只有一个目标，直接选择
    if (yaw_list.size() == 1) {
        out_yaw = yaw_list[0];
        out_pitch = pitch_list[0];
        return true;
    }
    
    // 多个目标：选择与当前预测最接近的
    float pred_yaw, pred_pitch;
    
    if (initialized_) {
        // 使用当前状态作为预测
        pred_yaw = yaw_kf_.getStatePost()(0, 0);
        pred_pitch = pitch_kf_.getStatePost()(0, 0);
    } else {
        // 未初始化时，选择最接近0的目标
        pred_yaw = 0.0f;
        pred_pitch = 0.0f;
    }
    
    float min_dist_sq = std::numeric_limits<float>::max();
    size_t best_idx = 0;
    
    for (size_t i = 0; i < yaw_list.size(); ++i) {
        // 计算欧氏距离的平方（避免开方运算）
        float dy = yaw_list[i] - pred_yaw;
        float dp = pitch_list[i] - pred_pitch;
        float dist_sq = dy * dy + dp * dp;
        
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    
    out_yaw = yaw_list[best_idx];
    out_pitch = pitch_list[best_idx];
    return true;
}

// 检查是否突变
bool Tracker::isMutation(float measured_yaw, float measured_pitch)
{
    if (!initialized_) {
        return false;
    }
    
    float dy = std::abs(measured_yaw - yaw_);
    float dp = std::abs(measured_pitch - pitch_);
    
    // 处理角度环绕（-180到180）
    if (dy > 180.0f) dy = 360.0f - dy;
    
    return (dy > yaw_mutation_threshold_) || (dp > pitch_mutation_threshold_);
}

// 重置滤波器
void Tracker::resetFilter()
{
    // 复制原始模板滤波器
    yaw_kf_ = yaw_kf_origin_;
    pitch_kf_ = pitch_kf_origin_;
    initialized_ = false;
    is_lost_ = false;
}

// 检查是否丢失太久
bool Tracker::isLostTooLong(double current_time) const
{
    double lost_duration = current_time - last_detection_time_;
    return lost_duration > max_lost_time_;
}

// 获取丢失时间（秒）
double Tracker::getLostTime(double current_time) const
{
    if (!is_lost_) return 0.0;
    return current_time - last_detection_time_;
}

// 主更新函数
void Tracker::Update(const std::vector<float>& yaw_list,
                     const std::vector<float>& pitch_list,
                     double current_time)
{
    // 计算dt（与上次更新的时间差）
    double dt = 0.02;  // 默认20ms
    if (last_update_time_ > 0.0) {
        dt = current_time - last_update_time_;
        // 限制dt范围，防止异常值
        if (dt <= 0.0) dt = 0.001;  // 最小1ms
        if (dt > 1.0) dt = 1.0;     // 最大1s
    }
    last_update_time_ = current_time;
    
    // 更新状态转移矩阵中的dt
    updateTransitionMatrix(yaw_kf_, dt);
    updateTransitionMatrix(pitch_kf_, dt);
    
    // 检查是否有检测结果
    bool has_detection = !yaw_list.empty() && !pitch_list.empty() &&
                         yaw_list.size() == pitch_list.size();
    
    if (!has_detection) {
        // 没有检测到目标，进入丢失状态
        is_lost_ = true;
        
        if (isLostTooLong(current_time)) {
            // 丢失太久，重置滤波器
            resetFilter();
            yaw_ = 0.0f;
            pitch_ = 0.0f;
        } else {
            // 短暂丢失，继续预测
            if (initialized_) {
                yaw_kf_.predict();
                pitch_kf_.predict();
                yaw_ = yaw_kf_.getStatePost()(0, 0);
                pitch_ = pitch_kf_.getStatePost()(0, 0);
            }
        }
        return;
    }
    
    // 有检测结果，选择最佳匹配
    float target_yaw, target_pitch;
    if (!selectBestMatch(yaw_list, pitch_list, target_yaw, target_pitch)) {
        return;
    }
    
    // 检查是否突变
    if (isMutation(target_yaw, target_pitch)) {
        // 发生突变，重置滤波器
        resetFilter();
    }
    
    // 如果是第一次初始化，设置初始状态
    if (!initialized_) {
        Eigen::MatrixXf init_state(3, 1);
        init_state << target_yaw, 0.0f, 0.0f;
        yaw_kf_.setStatePost(init_state);
        
        init_state << target_pitch, 0.0f, 0.0f;
        pitch_kf_.setStatePost(init_state);
        
        yaw_ = target_yaw;
        pitch_ = target_pitch;
        initialized_ = true;
        is_lost_ = false;
        last_detection_time_ = current_time;
        return;
    }
    
    // 正常更新流程
    // 1. Predict
    yaw_kf_.predict();
    pitch_kf_.predict();
    
    // 2. Correct
    Eigen::MatrixXf measurement(1, 1);
    measurement << target_yaw;
    Eigen::MatrixXf yaw_state = yaw_kf_.correct(measurement);
    
    measurement << target_pitch;
    Eigen::MatrixXf pitch_state = pitch_kf_.correct(measurement);
    
    // 3. 提取滤波后的位置
    yaw_ = yaw_state(0, 0);
    pitch_ = pitch_state(0, 0);
    
    // 更新检测到目标的时间和状态
    last_detection_time_ = current_time;
    is_lost_ = false;
}
