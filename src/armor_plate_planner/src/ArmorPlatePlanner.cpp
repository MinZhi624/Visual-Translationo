#include "armor_plate_planner/ArmorPlatePlanner.hpp"

#include <armor_plate_interfaces/TrackerTypes.hpp>
#include <geometry_msgs/msg/point.hpp>

using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::TrackedTargets;
using armor_plate_interfaces::msg::PlannerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

namespace {

using armor_plate_interfaces::uint8ToTrackerState;

PlannerArmor toPlannerArmor(const armor_plate_interfaces::msg::TrackedArmor& msg, size_t idx) {
    return PlannerArmor{
        ArmorPose{
            {msg.position_world.x, msg.position_world.y, msg.position_world.z},
            msg.yaw_world,
            ArmorName::NONE,
            ArmorType::SMALL
        },
        idx
    };
}

TargetState toTargetState(const armor_plate_interfaces::msg::TrackedTarget& msg) {
    TargetState state;
    state.track_id = msg.track_id;
    state.tracking_state = uint8ToTrackerState(msg.tracking_state);
    state.center_world = {msg.center_world.x, msg.center_world.y, msg.center_world.z};
    state.center_velocity = {msg.center_velocity.x, msg.center_velocity.y, msg.center_velocity.z};
    state.yaw = msg.yaw;
    state.yaw_rate = msg.yaw_rate;
    state.radius = msg.radius;
    state.radius_offset = msg.radius_offset;
    state.height_offset = msg.height_offset;

    state.armors.reserve(msg.armors.size());
    for (size_t i = 0; i < msg.armors.size(); ++i) {
        state.armors.push_back(toPlannerArmor(msg.armors[i], i));
    }
    return state;
}

std::vector<TargetState> toTargetStates(const armor_plate_interfaces::msg::TrackedTargets& msg) {
    std::vector<TargetState> result;
    result.reserve(msg.targets.size());
    for (const auto& t : msg.targets) {
        result.push_back(toTargetState(t));
    }
    return result;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

GimbalData toGimbalData(const GimbalAngle& msg) {
    return GimbalData{msg.yaw_abs, msg.pitch_abs};
}

}  // namespace

ArmorPlatePlanner::ArmorPlatePlanner()
    : Node("armor_plate_planner_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "Armor Plate Planner节点创建成功！");
    init();
}

void ArmorPlatePlanner::targetsCallback(const TrackedTargets::SharedPtr msg)
{
    builtin_interfaces::msg::Time stamp = msg->header.stamp;

    // 消费者：非阻塞取最新云台数据
    GimbalAngle gimbal_msg;
    if (!gimbal_queue_.pop(gimbal_msg, std::chrono::milliseconds(0))) {
        publishInvalidCommand(stamp);
        return;
    }
    GimbalData current_gimbal = toGimbalData(gimbal_msg);

    // 转换为内部数据结构
    auto targets = toTargetStates(*msg);

    //////////  车辆选择 /////////

    // 选择目标（第一版只接受 TRACKING） -- 同时也只是选则一个
    auto index = target_selector_.selectIndex(targets);
    if (!index.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    ////////// 装甲板选择 /////////
    const auto& selected_target = targets[index.value()];
    const auto& current_armors = selected_target.armors;

    if (current_armors.empty()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 在当前帧选择正对射手的装甲板 -- 用来原始图像debug，检查滤波效果
    auto initial_armor = armor_selector_.select(current_armors);
    if (!initial_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 时间外推
    double prediction_time = 0.0;
    auto predicted = target_predictor_.predict(selected_target, prediction_time);

    // 在预测帧中选择装甲板，优先保持同一 armor_id
    auto selected_armor = armor_selector_.select(predicted.armors);
    if (!selected_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 弹道求解
    auto ballistic_result = ballistic_solver_.solve(selected_armor->pose.xyz_world);

    // 构建指令
    CommandContext ctx{
        selected_target,
        *initial_armor,
        *selected_armor,
        ballistic_result,
        current_gimbal,
        prediction_time
    };
    auto [aim_cmd, debug] = buildCommand(ctx, stamp);

    aim_command_pub_->publish(aim_cmd);
    planner_debug_pub_->publish(debug);
}

std::pair<AimCommand, PlannerDebug> ArmorPlatePlanner::buildCommand(
    const CommandContext& ctx,
    const builtin_interfaces::msg::Time& stamp)
{
    AimCommand aim_cmd;
    PlannerDebug debug;

    // 基础信息
    debug.header.stamp = stamp;
    debug.selected_track_id = static_cast<int32_t>(ctx.target.track_id);
    debug.selected_armor_id = static_cast<int32_t>(ctx.selected_armor.index);
    debug.original_point_world = toPoint(ctx.initial_armor.pose.xyz_world);
    debug.predicted_point_world = toPoint(ctx.selected_armor.pose.xyz_world);
    debug.prediction_time = ctx.prediction_time;

    // 朝向分数
    debug.facing_score = armor_selector_.computeFacingScore(
        ctx.selected_armor.pose.xyz_world, ctx.selected_armor.pose.yaw);

    if (ctx.ballistic_result.valid) {
        command_generator_.setGimbalReceived(true);
        auto gimbal_delta = command_generator_.generate(
            ctx.ballistic_result.compensated_point, ctx.gimbal);

        if (std::isfinite(gimbal_delta.delta_yaw) && std::isfinite(gimbal_delta.delta_pitch)) {
            aim_cmd.delta_yaw = gimbal_delta.delta_yaw;
            aim_cmd.delta_pitch = gimbal_delta.delta_pitch;
            aim_cmd.is_valid = true;

            debug.is_valid = true;
            debug.compensated_point_world = toPoint(ctx.ballistic_result.compensated_point);
            debug.flight_time = ctx.ballistic_result.flight_time;
        } else {
            aim_cmd.delta_yaw = 0.0f;
            aim_cmd.delta_pitch = 0.0f;
            aim_cmd.is_valid = false;

            debug.is_valid = false;
            debug.compensated_point_world.x = 0.0;
            debug.compensated_point_world.y = 0.0;
            debug.compensated_point_world.z = 0.0;
            debug.flight_time = 0.0;
        }
    } else {
        aim_cmd.delta_yaw = 0.0f;
        aim_cmd.delta_pitch = 0.0f;
        aim_cmd.is_valid = false;

        debug.is_valid = false;
        debug.compensated_point_world.x = 0.0;
        debug.compensated_point_world.y = 0.0;
        debug.compensated_point_world.z = 0.0;
        debug.flight_time = 0.0;
    }

    return {aim_cmd, debug};
}

void ArmorPlatePlanner::init()
{
    // ===== 参数获取 ===== //
    double bullet_speed = this->declare_parameter<double>("bullet_speed", 25.0);
    double gravity = this->declare_parameter<double>("gravity", 9.81);
    double drag_coeff = this->declare_parameter<double>("drag_coeff", 0.0);
    double max_face_angle = this->declare_parameter<double>("max_armor_face_angle", 1.0472);

    ballistic_solver_ = BallisticSolver(bullet_speed, gravity, drag_coeff);
    armor_selector_ = ArmorSelector(max_face_angle);

    // ===== ROS 相关 ===== //
    targets_sub_ = this->create_subscription<TrackedTargets>(
        "/tracked_targets", rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlatePlanner::targetsCallback, this, std::placeholders::_1));

    gimbal_sub_ = this->create_subscription<GimbalAngle>(
        "/gimbal_angle", rclcpp::SensorDataQoS(),
        std::bind(&ArmorPlatePlanner::gimbalCallback, this, std::placeholders::_1));

    aim_command_pub_ = this->create_publisher<AimCommand>("/aim_command", rclcpp::SensorDataQoS());
    planner_debug_pub_ = this->create_publisher<PlannerDebug>("/planner_debug", rclcpp::SensorDataQoS());
}

void ArmorPlatePlanner::gimbalCallback(const GimbalAngle::SharedPtr msg)
{
    if (!std::isfinite(msg->yaw_abs) || !std::isfinite(msg->pitch_abs)) return;
    gimbal_queue_.push(*msg);
}

void ArmorPlatePlanner::publishInvalidCommand(const builtin_interfaces::msg::Time & stamp)
{
    AimCommand cmd;
    cmd.delta_yaw = 0.0f;
    cmd.delta_pitch = 0.0f;
    cmd.is_valid = false;
    aim_command_pub_->publish(cmd);

    PlannerDebug debug;
    debug.header.stamp = stamp;
    debug.is_valid = false;
    debug.selected_track_id = -1;
    debug.selected_armor_id = -1;
    debug.original_point_world.x = 0.0;
    debug.original_point_world.y = 0.0;
    debug.original_point_world.z = 0.0;
    debug.predicted_point_world.x = 0.0;
    debug.predicted_point_world.y = 0.0;
    debug.predicted_point_world.z = 0.0;
    debug.compensated_point_world.x = 0.0;
    debug.compensated_point_world.y = 0.0;
    debug.compensated_point_world.z = 0.0;
    debug.prediction_time = 0.0;
    debug.flight_time = 0.0;
    debug.facing_score = 0.0;
    planner_debug_pub_->publish(debug);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorPlatePlanner>());
    rclcpp::shutdown();
    return 0;
}
