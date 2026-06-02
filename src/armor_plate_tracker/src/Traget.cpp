#include "armor_plate_tracker/Traget.hpp"

#include <cmath>
#include <limits>

double Traget::normalizeRadAngle(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

void Traget::updateArmorList()
{
    Eigen::Vector<double, 11> state = ekf_.getStatePost();
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

    // 找距离观测点最远的预测装甲板（对面那块）-- 排除distance结算的影响
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

    // 在其余 3 块中找与观测装甲板 yaw 角度差最小的
    size_t best_idx = (far_idx == 0) ? 1 : 0;
    double min_ang = std::abs(normalizeRadAngle(armor.ypr_world_.x() - armor_list_[best_idx].w()));
    for (size_t i = 0; i < 4; ++i) {
        if (i == far_idx) continue;
        double ang = std::abs(normalizeRadAngle(armor.ypr_world_.x() - armor_list_[i].w()));
        if (ang < min_ang) {
            min_ang = ang;
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
}

void Traget::update(const std::vector<TrackerArmor> & armors)
{
    if (armors.empty()) return;
    for (const auto & armor : armors) {
        update(armor);
    }
}

void Traget::reset()
{
    Eigen::Vector<double, 11> zero_state = Eigen::Vector<double, 11>::Zero();
    Eigen::Matrix<double, 11, 11> identity_P = Eigen::Matrix<double, 11, 11>::Identity();
    ekf_.initialize(zero_state, identity_P);

    initialized_ = false;
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
    initialized_ = true;
    selected_armor_id_ = 0;
}


Eigen::Vector3d Traget::getCenterPointWorld() const
{
    Eigen::Vector<double, 11> state = ekf_.getStatePost();
    return Eigen::Vector3d(state[0], state[2], state[4]);
}

Eigen::Vector3d Traget::getCenterVelocity() const
{
    Eigen::Vector<double, 11> state = ekf_.getStatePost();
    return Eigen::Vector3d(state[1], state[3], 0);
}

double Traget::getRadius() const
{
    return ekf_.getStatePost()[8];
}
