#include "armor_plate_tracker/Traget.hpp"

#include <numeric>

double Traget::normalizeRadAngle(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

void Traget::updateArmorList()
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    double yaw = state[6];

    for (int i = 0; i < 4; ++i) {
        double angle = yaw + i * M_PI / 2.0;
        bool use_l_h = (i == 1 || i == 3);
        double radius = use_l_h ? state[8] + state[9] : state[8];
        double z = use_l_h ? state[4] + state[10] : state[4];
        armor_list_[i] = Eigen::Vector<double, 4>{
            state[0] - radius * std::cos(angle),
            state[2] - radius * std::sin(angle),
            z,
            normalizeRadAngle(angle)
        };
    }
}

size_t Traget::findArmorIdx(const TrackerArmor & armor)
{
    updateArmorList();
    // 找距离观测点最远的预测装甲板（对面那块），后续关联时排除。
    size_t far_idx = 0;
    double max_dist = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        double dx = armor_list_[i].x() - armor.xyz_world_.x();
        double dy = armor_list_[i].y() - armor.xyz_world_.y();
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > max_dist) {
            max_dist = dist;
            far_idx = i;
        }
    }

    /*  不用距离差的原因：
        1. 天然Pose的yaw 和 World中的yaw 单位统一
        2. World中的yaw 其实相当于用了x,y的数据。
    */
    const double armor_pose_yaw_world = armor.ypr_world_.x();
    const double armor_yaw_world = armor.ypd_world_.x();

    size_t best_idx = (far_idx == 0) ? 1 : 0;
    double min_score = 1e10;
    for (size_t i = 0; i < 4; ++i) {
        if (i == far_idx) continue;

        double predicted_armor_yaw = armor_list_[i].w();
        double predicted_bearing_yaw = std::atan2(armor_list_[i].y(), armor_list_[i].x());
        double armor_pose_yaw_diff = std::abs(normalizeRadAngle(armor_pose_yaw_world - predicted_armor_yaw));
        double armor_yaw_diff = std::abs(normalizeRadAngle(armor_yaw_world - predicted_bearing_yaw));
        double score = armor_pose_yaw_diff + armor_yaw_diff;

        if (score < min_score) {
            min_score = score;
            best_idx = i;
        }
    }
    return best_idx;
}

void Traget::predict(double dt)
{
    ekf_.updateProcessNoiseCov(dt);
    ekf_.updateStateTransitionMatrix(dt);
    ekf_.predict();
}

void Traget::update(const TrackerArmor & armor)
{
    size_t armor_index = findArmorIdx(armor);
    Eigen::Vector<double, 4> measurement;
    measurement << armor.ypd_world_.x(), armor.ypd_world_.y(),
                   armor.ypd_world_.z(), armor.ypr_world_.x();
    ekf_.correct(measurement, armor_index);
    selected_armor_id_ = armor_index;
    updateArmorList();
    checkConverge();
    checkDivergence();
}

void Traget::update(const std::vector<TrackerArmor> & armors)
{
    if (armors.empty()) return;
    for (const auto & armor : armors) {
        update(armor);
    }
}

bool Traget::checkConverge()
{
    const std::deque<int> & nis_failures = ekf_.getNISFailures();
    // 这里用WindowSize是为了防止一开始数字太小导致判断错误
    is_converged_ = std::accumulate(nis_failures.begin(), nis_failures.end(), 0) >= (0.4 * MyExtendedKalmanFilter::NIS_WINDOW_SIZE);
    return is_converged_;
}


bool Traget::checkDivergence()
{
    /*
        TODO:
        1. 未来考虑v_z的情况
    */
    const double r = ekf_.getState()[8];
    const double l = ekf_.getState()[9];
    // 判断半径是否在 0.05 ~ 0.5 之间
    bool is_r_vaild = (r >= 0.05 && r <= 0.5);
    bool is_l_vaild = (r + l >= 0.05 && r + l <= 0.5);
    if (is_r_vaild && is_l_vaild) is_divergent_ = false;
    else is_divergent_ = true; 
    return is_divergent_;
}

void Traget::reset()
{
    Eigen::Vector<double, 11> zero_state = Eigen::Vector<double, 11>::Zero();
    Eigen::Matrix<double, 11, 11> identity_P = Eigen::Matrix<double, 11, 11>::Identity();
    ekf_.initialize(zero_state, identity_P);

    is_initialized_ = false;
    selected_armor_id_ = 0;
    armor_list_.fill(Eigen::Vector<double, 4>::Zero());
}

void Traget::init(const TrackerArmor & armor)
{
    float armor_pose_yaw_world = armor.ypr_world_.x();
    const Eigen::Vector3d & xyz_world = armor.xyz_world_;

    /*
        初始化时由当前装甲板反推旋转中心:
        armor_x = x_c - r * cos(yaw)
        armor_y = y_c - r * sin(yaw)
        x_c = armor_x + r * cos(yaw)
        y_c = armor_y + r * sin(yaw)
        z_c = armor_z
        yaw = armor_yaw
        omega = 0
        l = 0
        h = 0
    */
    const double r_init = 0.20;
    double x_c0 = xyz_world.x() + r_init * std::cos(armor_pose_yaw_world);
    double y_c0 = xyz_world.y() + r_init * std::sin(armor_pose_yaw_world);

    Eigen::Vector<double, 11> init_state;
    init_state << x_c0, 0.0, y_c0, 0.0, xyz_world.z(), 0.0, armor_pose_yaw_world, 0.0,
                  r_init, 0.0, 0.0;

    Eigen::Matrix<double, 11, 11> init_P = Eigen::Matrix<double, 11, 11>::Identity();
    init_P.diagonal() << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;

    ekf_.initialize(init_state, init_P);
    is_initialized_ = true;
    selected_armor_id_ = 0;
}


Eigen::Vector3d Traget::getCenterPointWorld() const
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    return Eigen::Vector3d(state[0], state[2], state[4]);
}

Eigen::Vector3d Traget::getCenterVelocity() const
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    return Eigen::Vector3d(state[1], state[3], 0);
}

double Traget::getRadius() const
{
    return ekf_.getState()[8];
}
