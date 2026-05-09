#include "armor_plate_tracker/CenterFilter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

CenterFilter::CenterFilter()
    : window_size_(12)
    , min_frames_(8)
    , center_(Eigen::Vector2d::Zero())
    , velocity_(Eigen::Vector2d::Zero())
    , r_(0.26)
{
}

void CenterFilter::addFrame(const std::vector<ArmorPlateStamped>& plates, double timestamp)
{
    Frame frame;
    frame.timestamp = timestamp;
    frame.plates = plates;
    frame_history_.push_back(frame);

    // 维护窗口大小（帧数）
    if (frame_history_.size() > window_size_) {
        frame_history_.pop_front();
    }
}

bool CenterFilter::solve()
{
    size_t N = frame_history_.size();
    if (N < min_frames_) {
        return false;
    }

    // 过滤掉 yaw 在 -90° ±2° 范围内的点（r 欠约束）
    const double deg = M_PI * 2 / 180.0;
    const double yaw_neg90 = -M_PI / 2.0;

    // 收集每帧的有效观测
    struct Obs {
        size_t frame_idx;
        const ArmorPlateStamped* plate;
    };
    std::vector<Obs> observations;
    for (size_t f = 0; f < N; ++f) {
        for (const auto& plate : frame_history_[f].plates) {
            if (std::abs(plate.yaw_world - yaw_neg90) > deg) {
                observations.push_back({f, &plate});
            }
        }
    }

    if (observations.size() < 8) {
        RCLCPP_WARN(rclcpp::get_logger("CENTER_FILTER"),
            "LS skipped: only %zu valid observations after filtering -90°", observations.size());
        return false;
    }

    // 状态向量：[x_c0, y_c0, vx0, vy0, x_c1, y_c1, vx1, vy1, ..., r]
    // 每帧 4 个变量 (x_c, y_c, vx, vy) + 1 个全局 r
    const size_t vars_per_frame = 4;
    const size_t num_vars = vars_per_frame * N + 1;
    const size_t r_idx = vars_per_frame * N;

    // 帧状态在向量中的起始位置
    auto frame_idx = [&](size_t f) -> size_t { return vars_per_frame * f; };

    // 残差行数
    size_t num_vision_rows = 2 * observations.size();
    size_t num_motion_rows = (N > 1) ? 2 * (N - 1) : 0;
    size_t num_rows = num_vision_rows + num_motion_rows;

    // 正则化参数
    const double lambda_r = 0.1;
    const double lambda_motion = 1.0;
    const double r_prior = r_;  // 用上一帧的 r 作为先验，保持连续性

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_rows, num_vars);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(num_rows);

    // --- 观测残差 ---
    // x_obs = x_c_f + r * sin(yaw), y_obs = y_c_f - r * cos(yaw)
    for (size_t i = 0; i < observations.size(); ++i) {
        size_t f = observations[i].frame_idx;
        double yaw = observations[i].plate->yaw_world;
        size_t base = frame_idx(f);

        // x 方程
        A(2 * i, base + 0) = 1.0;      // x_c_f
        A(2 * i, r_idx) = std::sin(yaw); // r
        b(2 * i) = observations[i].plate->position_world.x();

        // y 方程
        A(2 * i + 1, base + 1) = 1.0;        // y_c_f
        A(2 * i + 1, r_idx) = -std::cos(yaw); // r
        b(2 * i + 1) = observations[i].plate->position_world.y();
    }

    // --- 运动残差（匀速模型）---
    // x_c_{k+1} = x_c_k + vx_k * dt  →  x_c_{k+1} - x_c_k - vx_k * dt = 0
    // y_c_{k+1} = y_c_k + vy_k * dt  →  y_c_{k+1} - y_c_k - vy_k * dt = 0
    size_t row = num_vision_rows;
    for (size_t f = 0; f < N - 1; ++f) {
        double dt = frame_history_[f + 1].timestamp - frame_history_[f].timestamp;
        if (dt <= 0.0) dt = 0.025;
        double w = lambda_motion;

        size_t base_k = frame_idx(f);
        size_t base_k1 = frame_idx(f + 1);

        // x 方程：x_c_{k+1} - x_c_k - vx_k * dt = 0
        A(row, base_k1 + 0) = w;       // x_c_{k+1}
        A(row, base_k + 0) = -w;       // -x_c_k
        A(row, base_k + 2) = -w * dt;  // -vx_k * dt
        b(row) = 0.0;
        row++;

        // y 方程：y_c_{k+1} - y_c_k - vy_k * dt = 0
        A(row, base_k1 + 1) = w;       // y_c_{k+1}
        A(row, base_k + 1) = -w;       // -y_c_k
        A(row, base_k + 3) = -w * dt;  // -vy_k * dt
        b(row) = 0.0;
        row++;
    }

    // --- r 正则化 ---
    Eigen::MatrixXd A_reg(num_rows + 1, num_vars);
    Eigen::VectorXd b_reg(num_rows + 1);
    A_reg.topRows(num_rows) = A;
    b_reg.head(num_rows) = b;
    A_reg.row(num_rows).setZero();
    A_reg(num_rows, r_idx) = std::sqrt(lambda_r);
    b_reg(num_rows) = std::sqrt(lambda_r) * r_prior;

    // SVD 求解
    Eigen::VectorXd x = A_reg.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_reg);

    // 提取最新帧的 center, velocity 和 r
    size_t last_base = frame_idx(N - 1);
    Eigen::Vector2d center_latest(x[last_base], x[last_base + 1]);
    Eigen::Vector2d velocity_latest(x[last_base + 2], x[last_base + 3]);
    double r_solved = x[r_idx];

    RCLCPP_INFO(rclcpp::get_logger("CENTER_FILTER"),
        "LS Raw: x_c:%.4f y_c:%.4f vx:%.4f vy:%.4f r_raw:%.4f (used %zu obs, %zu frames)",
        center_latest[0], center_latest[1],
        velocity_latest[0], velocity_latest[1],
        r_solved, observations.size(), N);

    // r 负值回退
    if (r_solved < 0.0) {
        RCLCPP_INFO(rclcpp::get_logger("CENTER_FILTER"),
            "LS r negative (%.4f), fallback to %.4f", r_solved, r_);
        r_solved = r_;
    }

    // r 约束到 [0.12, 0.4]
    if (r_solved < 0.12) r_solved = 0.12;
    if (r_solved > 0.4)  r_solved = 0.4;

    center_ = center_latest;
    velocity_ = velocity_latest;
    r_ = r_solved;

    return true;
}

void CenterFilter::reset()
{
    frame_history_.clear();
    center_.setZero();
    velocity_.setZero();
    r_ = 0.26;
}

Eigen::Vector2d CenterFilter::backCalculateCenter(
    const Eigen::Vector3d& position_world, double yaw_world, double r_default)
{
    double x_c = position_world.x() - r_default * std::sin(yaw_world);
    double y_c = position_world.y() + r_default * std::cos(yaw_world);
    return Eigen::Vector2d(x_c, y_c);
}
