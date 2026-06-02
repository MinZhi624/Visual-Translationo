#include "armor_plate_tracker/MyExtendedKalmanFilter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

/*
    EKF 状态定义:
    state[0]  = x_c
    state[1]  = v_x
    state[2]  = y_c
    state[3]  = v_y
    state[4]  = z_c
    state[5]  = v_z
    state[6]  = yaw
    state[7]  = omega
    state[8]  = r
    state[9]  = l
    state[10] = h

    x_c, y_c, z_c 是目标旋转中心。
    yaw 是目标自转角。
    omega 是目标自转角速度。
    r 是 0/2 号装甲板半径。
    r + l 是 1/3 号装甲板半径。
    z_c + h 是 1/3 号装甲板高度。
*/
static double normalizeRadAngle(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

MyExtendedKalmanFilter::MyExtendedKalmanFilter()
{
    state_pre_ = Eigen::Vector<double, 11>::Zero();
    state_post_ = Eigen::Vector<double, 11>::Zero();

    error_cov_pre_ = Eigen::Matrix<double, 11, 11>::Identity();
    error_cov_post_ = Eigen::Matrix<double, 11, 11>::Identity();

    state_transition_matrix_ = Eigen::Matrix<double, 11, 11>::Identity();
    observation_jacobian_ = Eigen::Matrix<double, 4, 11>::Zero();

    process_noise_cov_ = Eigen::Matrix<double, 11, 11>::Zero();
    process_noise_cov_.diagonal() <<
        0.001, 0.01, 0.001, 0.01, 0.001, 0.01, 0.001, 0.01, 0.0, 0.0, 0.0;

    observation_noise_cov_ = Eigen::Matrix<double, 4, 4>::Zero();
    observation_noise_cov_.diagonal() <<
        4e-3, 4e-3, 1, 9e-2;

    kalman_gain_ = Eigen::Matrix<double, 11, 4>::Zero();
    origin_observation_ = Eigen::Vector<double, 4>::Zero();
    filtered_observation_ = Eigen::Vector<double, 4>::Zero();
}

void MyExtendedKalmanFilter::initialize(
    const Eigen::Vector<double, 11>& state_pre,
    const Eigen::Matrix<double, 11, 11>& error_cov_pre)
{
    state_pre_ = state_pre;
    state_post_ = state_pre;
    error_cov_pre_ = error_cov_pre;
    error_cov_post_ = error_cov_pre;
}

void MyExtendedKalmanFilter::predict()
{
    /*
        预测方程如下:
        x_c^k     = x_c^{k-1} + v_x^{k-1} * dt
        v_x^k     = v_x^{k-1}
        y_c^k     = y_c^{k-1} + v_y^{k-1} * dt
        v_y^k     = v_y^{k-1}
        z_c^k     = z_c^{k-1} + v_z^{k-1} * dt
        v_z^k     = v_z^{k-1}
        yaw^k     = yaw^{k-1} + omega^{k-1} * dt
        omega^k   = omega^{k-1}
        r^k       = r^{k-1}
        l^k       = l^{k-1}
        h^k       = h^{k-1}
    */
    state_pre_ = state_transition_matrix_ * state_post_;
    error_cov_pre_ = state_transition_matrix_ * error_cov_post_ * state_transition_matrix_.transpose()
                   + process_noise_cov_;

    state_pre_[6] = normalizeRadAngle(state_pre_[6]);

    // 没有 correct 的帧也要推进后验，否则连续丢帧时状态不会继续预测。
    state_post_ = state_pre_;
    error_cov_post_ = error_cov_pre_;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::correct(const Eigen::Vector<double, 4>& measurement, int armor_id)
{
    armor_id_ = armor_id;
    origin_observation_ = measurement;
    observation_jacobian_ = calculateObservationJacobian();

    /*
        自适应 R 矩阵:
        核心原因，当装甲板侧过来的时候，位姿解算会不准，详细看Compare/Identification里面的图片.
        会呈现出来椭圆的的现象。尤其是当我角点识别不好的preprocess的情况下  
        z[0] = yaw_to_armor
        z[1] = pitch_to_armor
        z[2] = distance_to_armor
        z[3] = armor_yaw

        delta_angle = armor_yaw - yaw_to_armor
        R_yaw      = 4e-3
        R_pitch    = 4e-3
        R_distance = log(abs(delta_angle) + 1) + 1
        R_angle    = log(abs(distance_to_armor) + 1) / 200 + 9e-2
    */
    double delta_angle = normalizeRadAngle(measurement[3] - measurement[0]);
    observation_noise_cov_.diagonal() <<
        4e-3,
        4e-3,
        std::log(std::abs(delta_angle) + 1.0) + 1.0,
        std::log(std::abs(measurement[2]) + 1.0) / 200.0 + 9e-2;

    auto predicted_obs = measurementFunction(state_pre_);
    Eigen::Matrix<double, 4, 4> innovation_cov =
        observation_jacobian_ * error_cov_pre_ * observation_jacobian_.transpose()
        + observation_noise_cov_;

    kalman_gain_ = error_cov_pre_ * observation_jacobian_.transpose() * innovation_cov.inverse();

    Eigen::Vector<double, 4> residual = measurement - predicted_obs;
    // yaw 与 armor_yaw 是角度量，残差必须落回 [-pi, pi]，避免跨 pi 时跳变。
    residual[0] = normalizeRadAngle(residual[0]);
    residual[3] = normalizeRadAngle(residual[3]);

    state_post_ = state_pre_ + kalman_gain_ * residual;
    checkValue();

    Eigen::Matrix<double, 11, 11> identity = Eigen::Matrix<double, 11, 11>::Identity();
    Eigen::Matrix<double, 11, 11> temp = identity - kalman_gain_ * observation_jacobian_;
    error_cov_post_ = temp * error_cov_pre_ * temp.transpose()
                    + kalman_gain_ * observation_noise_cov_ * kalman_gain_.transpose();

    filtered_observation_ = measurementFunction(state_post_);
    return filtered_observation_;
}

void MyExtendedKalmanFilter::updateProcessNoiseCov(const double & dt)
{
    /*
        分段白噪声加速度模型如下:
        Q11 = dt^4 / 4
        Q12 = dt^3 / 2
        Q21 = dt^3 / 2
        Q22 = dt^2

        x/v_x 使用平动加速度方差 a_var。
        y/v_y 使用平动加速度方差 a_var。
        z/v_z 使用平动加速度方差 a_var。
        yaw/omega 使用角加速度方差 yaw_a_var。
    */
    const double safe_dt = std::max(0.0, dt);
    const double a_var = 400.0;
    const double yaw_a_var = 1600.0;

    const double Q11 = safe_dt * safe_dt * safe_dt * safe_dt / 4.0;
    const double Q12 = safe_dt * safe_dt * safe_dt / 2.0;
    const double Q21 = safe_dt * safe_dt * safe_dt / 2.0;
    const double Q22 = safe_dt * safe_dt;

    process_noise_cov_ <<
        a_var * Q11, a_var * Q12,           0,           0,           0,           0,               0,               0, 0, 0, 0,
        a_var * Q21, a_var * Q22,           0,           0,           0,           0,               0,               0, 0, 0, 0,
                  0,           0, a_var * Q11, a_var * Q12,           0,           0,               0,               0, 0, 0, 0,
                  0,           0, a_var * Q21, a_var * Q22,           0,           0,               0,               0, 0, 0, 0,
                  0,           0,           0,           0, a_var * Q11, a_var * Q12,               0,               0, 0, 0, 0,
                  0,           0,           0,           0, a_var * Q21, a_var * Q22,               0,               0, 0, 0, 0,
                  0,           0,           0,           0,           0,           0, yaw_a_var * Q11, yaw_a_var * Q12, 0, 0, 0,
                  0,           0,           0,           0,           0,           0, yaw_a_var * Q21, yaw_a_var * Q22, 0, 0, 0,
                  0,           0,           0,           0,           0,           0,               0,               0, 0, 0, 0,
                  0,           0,           0,           0,           0,           0,               0,               0, 0, 0, 0,
                  0,           0,           0,           0,           0,           0,               0,               0, 0, 0, 0;
}

void MyExtendedKalmanFilter::updateStateTransitionMatrix(const double& dt)
{
    /*
        状态转移矩阵对应的方程如下:
        x_c^k     = x_c^{k-1} + v_x^{k-1} * dt
        v_x^k     = v_x^{k-1}
        y_c^k     = y_c^{k-1} + v_y^{k-1} * dt
        v_y^k     = v_y^{k-1}
        z_c^k     = z_c^{k-1} + v_z^{k-1} * dt
        v_z^k     = v_z^{k-1}
        yaw^k     = yaw^{k-1} + omega^{k-1} * dt
        omega^k   = omega^{k-1}
        r^k       = r^{k-1}
        l^k       = l^{k-1}
        h^k       = h^{k-1}
    */
    state_transition_matrix_ <<
        1, dt, 0,  0, 0,  0, 0,  0, 0, 0, 0,
        0,  1, 0,  0, 0,  0, 0,  0, 0, 0, 0,
        0,  0, 1, dt, 0,  0, 0,  0, 0, 0, 0,
        0,  0, 0,  1, 0,  0, 0,  0, 0, 0, 0,
        0,  0, 0,  0, 1, dt, 0,  0, 0, 0, 0,
        0,  0, 0,  0, 0,  1, 0,  0, 0, 0, 0,
        0,  0, 0,  0, 0,  0, 1, dt, 0, 0, 0,
        0,  0, 0,  0, 0,  0, 0,  1, 0, 0, 0,
        0,  0, 0,  0, 0,  0, 0,  0, 1, 0, 0,
        0,  0, 0,  0, 0,  0, 0,  0, 0, 1, 0,
        0,  0, 0,  0, 0,  0, 0,  0, 0, 0, 1;
}

Eigen::Matrix<double, 4, 11> MyExtendedKalmanFilter::calculateObservationJacobian()
{
    /*
        观测雅可比链式法则如下:
        H = d(ypda) / d(state)
        H = d(ypda) / d(xyza) * d(xyza) / d(state)
    */
    auto xyza_state_jacobian = calculateStateToXYZAJacobian(state_pre_, armor_id_);
    auto xyza_armor = measurementFunctionStateToXYZA(state_pre_, armor_id_);
    auto ypda_xyza_jacobian = calculateXYZAToYPDAJacobian(xyza_armor);
    return ypda_xyza_jacobian * xyza_state_jacobian;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunction(const Eigen::Vector<double, 11>& state)
{
    auto xyza_state = measurementFunctionStateToXYZA(state, armor_id_);
    auto ypda_xyza = measurementFunctionXYZAToYPDA(xyza_state);
    return ypda_xyza;
}

void MyExtendedKalmanFilter::checkValue()
{
    /*
        约束如下:
        yaw = normalize(yaw)
        0.05 <= r <= 0.5
        0.05 <= r + l <= 0.5
    */
    state_post_[6] = normalizeRadAngle(state_post_[6]);

    state_post_[8] = std::max(0.05, state_post_[8]);
    state_post_[8] = std::min(0.5, state_post_[8]);

    double second_radius = state_post_[8] + state_post_[9];
    second_radius = std::max(0.05, second_radius);
    second_radius = std::min(0.5, second_radius);
    state_post_[9] = second_radius - state_post_[8];
}

Eigen::Matrix<double, 4, 11> MyExtendedKalmanFilter::calculateStateToXYZAJacobian(
    const Eigen::Vector<double, 11>& state, int armor_id)
{
    /*
        装甲板几何模型如下:
        angle_i = yaw + i * PI / 2
        id = 0 或 id = 2 时 radius = r
        id = 1 或 id = 3 时 radius = r + l
        id = 0 或 id = 2 时 armor_z = z_c
        id = 1 或 id = 3 时 armor_z = z_c + h
        armor_x = x_c - radius * cos(angle_i)
        armor_y = y_c - radius * sin(angle_i)
        armor_angle = angle_i

        对 state 的雅可比关系如下:
        d armor_x / d x_c = 1
        d armor_x / d yaw = radius * sin(angle_i)
        d armor_x / d r = -cos(angle_i)
        d armor_x / d l = -cos(angle_i), 仅 id = 1 或 id = 3 时成立
        d armor_y / d y_c = 1
        d armor_y / d yaw = -radius * cos(angle_i)
        d armor_y / d r = -sin(angle_i)
        d armor_y / d l = -sin(angle_i), 仅 id = 1 或 id = 3 时成立
        d armor_z / d z_c = 1
        d armor_z / d h = 1, 仅 id = 1 或 id = 3 时成立
        d armor_angle / d yaw = 1
    */
    Eigen::Matrix<double, 4, 11> xyza_state_jacobian = Eigen::Matrix<double, 4, 11>::Zero();

    double car_yaw = state[6] + armor_id * M_PI / 2.0;
    bool use_l_h = (armor_id == 1 || armor_id == 3);
    double radius = use_l_h ? state[8] + state[9] : state[8];

    double cos_yaw = std::cos(car_yaw);
    double sin_yaw = std::sin(car_yaw);

    xyza_state_jacobian <<
        1, 0, 0, 0, 0, 0,  radius * sin_yaw, 0, -cos_yaw, use_l_h ? -cos_yaw : 0.0,                    0,
        0, 0, 1, 0, 0, 0, -radius * cos_yaw, 0, -sin_yaw, use_l_h ? -sin_yaw : 0.0,                    0,
        0, 0, 0, 0, 1, 0,                 0, 0,        0,                        0, use_l_h ? 1.0 : 0.0,
        0, 0, 0, 0, 0, 0,                 1, 0,        0,                        0,                    0;

    return xyza_state_jacobian;
}

Eigen::Matrix4d MyExtendedKalmanFilter::calculateXYZAToYPDAJacobian(const Eigen::Vector<double, 4> & xyza)
{
    /*
        xyza -> ypda 方程如下:
        yaw = atan2(y, x)
        pitch = atan2(z, sqrt(x^2 + y^2))
        distance = sqrt(x^2 + y^2 + z^2)
        armor_yaw = angle

        对 xyza 的雅可比关系如下:
        d yaw / d x = -y / (x^2 + y^2)
        d yaw / d y = x / (x^2 + y^2)
        d yaw / d z = 0
        d pitch / d x = -(x * z) / ((x^2 + y^2 + z^2) * sqrt(x^2 + y^2))
        d pitch / d y = -(y * z) / ((x^2 + y^2 + z^2) * sqrt(x^2 + y^2))
        d pitch / d z = sqrt(x^2 + y^2) / (x^2 + y^2 + z^2)
        d distance / d x = x / distance
        d distance / d y = y / distance
        d distance / d z = z / distance
        d armor_yaw / d angle = 1
    */
    double x = xyza[0];
    double y = xyza[1];
    double z = xyza[2];

    double xy2 = x * x + y * y;
    double xy_norm = std::sqrt(xy2);
    double dist2 = xy2 + z * z;
    double dist = std::sqrt(dist2);

    Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();
    if (xy2 < 1e-9 || dist2 < 1e-9) {
        jacobian(3, 3) = 1.0;
        return jacobian;
    }

    double dyaw_dx = -y / xy2;
    double dyaw_dy = x / xy2;

    double dpitch_dx = -(x * z) / (dist2 * xy_norm);
    double dpitch_dy = -(y * z) / (dist2 * xy_norm);
    double dpitch_dz = xy_norm / dist2;

    double ddistance_dx = x / dist;
    double ddistance_dy = y / dist;
    double ddistance_dz = z / dist;

    jacobian <<
        dyaw_dx,      dyaw_dy,      0.0,           0.0,
        dpitch_dx,    dpitch_dy,    dpitch_dz,     0.0,
        ddistance_dx, ddistance_dy, ddistance_dz,  0.0,
        0.0,          0.0,          0.0,           1.0;
    return jacobian;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunctionStateToXYZA(
    const Eigen::Vector<double, 11>& state, int armor_id)
{
    double x_c = state[0];
    double y_c = state[2];
    double z_c = state[4];
    double yaw = state[6];
    double r = state[8];
    double l = state[9];
    double h = state[10];

    /*
        h_state_to_xyza(state, armor_id) 方程如下:
        car_yaw = yaw + armor_id * PI / 2
        id = 0 或 id = 2 时 radius = r
        id = 1 或 id = 3 时 radius = r + l
        id = 0 或 id = 2 时 armor_z = z_c
        id = 1 或 id = 3 时 armor_z = z_c + h
        armor_x = x_c - radius * cos(car_yaw)
        armor_y = y_c - radius * sin(car_yaw)
        armor_angle = car_yaw
    */
    double car_yaw = yaw + armor_id * M_PI / 2.0;
    bool use_l_h = (armor_id == 1 || armor_id == 3);
    double radius = use_l_h ? r + l : r;
    double armor_z = use_l_h ? z_c + h : z_c;

    Eigen::Vector<double, 4> observation;
    observation <<
        x_c - radius * std::cos(car_yaw),
        y_c - radius * std::sin(car_yaw),
        armor_z,
        normalizeRadAngle(car_yaw);
    return observation;
}

Eigen::Vector<double, 4> MyExtendedKalmanFilter::measurementFunctionXYZAToYPDA(const Eigen::Vector<double, 4>& xyza)
{
    double x = xyza[0];
    double y = xyza[1];
    double z = xyza[2];

    double yaw = std::atan2(y, x);
    double pitch = std::atan2(z, std::sqrt(x * x + y * y));
    double distance = Eigen::Vector3d{x, y, z}.norm();
    return {
        yaw,
        pitch,
        distance,
        normalizeRadAngle(xyza[3])
    };
}
