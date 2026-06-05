#include "armor_plate_tracker/Target.hpp"
#include "armor_plate_common/angle.hpp"

#include <algorithm>
#include <armor_plate_common/geometry.hpp>
#include <numeric>

void Target::updateArmorList()
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    double yaw = state[6];
    ArmorType armor_type = armorNameToType(armor_name_);

    for (int i = 0; i < 4; ++i) {
        double armor_angle = yaw + i * M_PI / 2.0;
        bool use_l_h = (i == 1 || i == 3);
        double radius = use_l_h ? state[8] + state[9] : state[8];
        double z = use_l_h ? state[4] + state[10] : state[4];
        armor_list_[i].xyz_world = Eigen::Vector3d(
            state[0] - radius * std::cos(armor_angle),
            state[2] - radius * std::sin(armor_angle),
            z);
        armor_list_[i].yaw = armor_plate_common::normalizeRadAngle(armor_angle);
        armor_list_[i].name = armor_name_;
        armor_list_[i].type = armor_type;
    }
}

size_t Target::findArmorIdx(const TrackerArmor & armor)
{
    updateArmorList();

    // 按预测装甲板到世界原点的距离排序，只取最近的 3 块参与匹配。
    // 不要按照观测点的到各个装甲板的距离来排序，因为pnp解算不准可能删掉排掉正确解导致id失效。
    std::array<std::pair<double, size_t>, 4> distance_index_list;
    for (size_t i = 0; i < 4; ++i) {
        double predicted_distance = armor_list_[i].xyz_world.norm();
        distance_index_list[i] = {predicted_distance, i};
    }
    std::sort(distance_index_list.begin(), distance_index_list.end());

    /*  不用距离差的原因：
        1. 天然Pose的yaw 和 World中的yaw 单位统一
        2. World中的yaw 其实相当于用了x,y的数据。
    */
    const double armor_pose_yaw_world = armor.ypr_world_.x();
    const double armor_yaw_world = armor.ypd_world_.x();

    size_t best_idx = distance_index_list[0].second;
    double min_score = 1e10;
    for (size_t candidate_idx = 0; candidate_idx < 3; ++candidate_idx) {
        size_t i = distance_index_list[candidate_idx].second;

        double predicted_armor_pose_yaw = armor_list_[i].yaw;
        double predicted_armor_yaw = std::atan2(armor_list_[i].xyz_world.y(), armor_list_[i].xyz_world.x());
        double armor_pose_yaw_diff = std::abs(armor_plate_common::normalizeRadAngle(armor_pose_yaw_world - predicted_armor_pose_yaw));
        double armor_yaw_diff = std::abs(armor_plate_common::normalizeRadAngle(armor_yaw_world - predicted_armor_yaw));
        double score = armor_pose_yaw_diff + armor_yaw_diff;
        if (score < min_score) {
            min_score = score;
            best_idx = i;
        }
    }

    return best_idx;
}



Eigen::Vector<double, 4> Target::getArmorObservation(size_t armor_id)
{
    updateArmorList();
    if (armor_id >= armor_list_.size()) {
        armor_id = 0;
    }

    const ArmorPose & armor = armor_list_[armor_id];
    Eigen::Vector<double, 4> observation;
    observation.head<3>() = armor_plate_common::calculateYPD(armor.xyz_world);
    observation.w() = armor.yaw;
    return observation;
}

void Target::predict(double dt)
{
    ekf_.updateProcessNoiseCov(dt);
    ekf_.updateStateTransitionMatrix(dt);
    ekf_.predict();
}

void Target::update(const TrackerArmor & armor)
{
    size_t armor_index = findArmorIdx(armor);
    Eigen::Vector<double, 4> measurement;
    measurement << armor.ypd_world_.x(), armor.ypd_world_.y(),
                   armor.ypd_world_.z(), armor.ypr_world_.x();
    ekf_.correct(measurement, static_cast<int>(armor_index));
    selected_armor_id_ = armor_index;
    updateArmorList();
    checkConverge();
    checkDivergence();
}

void Target::update(const std::vector<TrackerArmor> & armors)
{
    if (armors.empty()) return;
    for (const auto & armor : armors) {
        update(armor);
    }
}

bool Target::checkConverge()
{
    const std::deque<int> & nis_failures = ekf_.getNISFailures();
    // 这里用WindowSize是为了防止一开始数字太小导致判断错误
    // NIS 失败少表示滤波器收敛/正常
    is_converged_ = std::accumulate(nis_failures.begin(), nis_failures.end(), 0) < (0.4 * MyExtendedKalmanFilter::NIS_WINDOW_SIZE);
    return is_converged_;
}


bool Target::checkDivergence()
{
    is_divergent_ = false;
    const double r = ekf_.getState()[8];
    const double l = ekf_.getState()[9];
    // 判断半径是否在 0.05 ~ 0.5 之间
    bool is_r_vaild = (r >= 0.05 && r <= 0.5);
    bool is_l_vaild = (r + l >= 0.05 && r + l <= 0.5);
    if (!(is_r_vaild && is_l_vaild)) is_divergent_ = true;
    return is_divergent_;
}

void Target::reset()
{
    Eigen::Vector<double, 11> zero_state = Eigen::Vector<double, 11>::Zero();
    Eigen::Matrix<double, 11, 11> identity_P = Eigen::Matrix<double, 11, 11>::Identity();
    ekf_.initialize(zero_state, identity_P);

    selected_armor_id_ = 0;
    armor_name_ = ArmorName::NONE;
    armor_list_.fill(ArmorPose{});
}

void Target::init(const TrackerArmor & armor)
{
    armor_name_ = armor.armor_name;

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
    const double r_init = 0.26;
    double x_c0 = xyz_world.x() + r_init * std::cos(armor_pose_yaw_world);
    double y_c0 = xyz_world.y() + r_init * std::sin(armor_pose_yaw_world);

    Eigen::Vector<double, 11> init_state;
    init_state << x_c0, 0.0, y_c0, 0.0, xyz_world.z(), 0.0, armor_pose_yaw_world, 0.0,
                  r_init, 0.0, 0.0;

    Eigen::Matrix<double, 11, 11> init_P = Eigen::Matrix<double, 11, 11>::Identity();
    init_P.diagonal() << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;

    ekf_.initialize(init_state, init_P);
    selected_armor_id_ = 0;
}


Eigen::Vector3d Target::getCenterPointWorld() const
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    return Eigen::Vector3d(state[0], state[2], state[4]);
}

Eigen::Vector3d Target::getCenterVelocity() const
{
    Eigen::Vector<double, 11> state = ekf_.getState();
    return Eigen::Vector3d(state[1], state[3], state[5]);
}

double Target::getRadius() const
{
    return ekf_.getState()[8];
}

double Target::getL() const
{
    return ekf_.getState()[9];
}

double Target::getH() const
{
    return ekf_.getState()[10];
}
