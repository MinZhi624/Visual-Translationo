#include "armor_plate_planner/PlannerNode.hpp"

using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::TrackedTargets;
using armor_plate_interfaces::msg::PlannerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

PlannerNode::PlannerNode()
    : Node("armor_plate_planner_node")
{
    this->declare_parameter("bullet_speed", 25.0);
    this->declare_parameter("gravity", 9.81);
    this->declare_parameter("max_armor_face_angle", 1.0472);

    double bullet_speed = this->get_parameter("bullet_speed").as_double();
    double gravity = this->get_parameter("gravity").as_double();
    double max_face_angle = this->get_parameter("max_armor_face_angle").as_double();

    ballistic_solver_ = BallisticSolver(bullet_speed, gravity);
    armor_selector_ = ArmorSelector(max_face_angle);

    // 第一版：云台中心即枪口
    shooter_origin_.x = 0.0;
    shooter_origin_.y = 0.0;
    shooter_origin_.z = 0.0;

    targets_sub_ = this->create_subscription<TrackedTargets>(
        "/tracked_targets", rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::targetsCallback, this, std::placeholders::_1));

    gimbal_sub_ = this->create_subscription<GimbalAngle>(
        "/gimbal_angle", rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::gimbalCallback, this, std::placeholders::_1));

    aim_command_pub_ = this->create_publisher<AimCommand>("/aim_command", rclcpp::SensorDataQoS());
    planner_debug_pub_ = this->create_publisher<PlannerDebug>("/planner_debug", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "PlannerNode initialized");
}

void PlannerNode::gimbalCallback(const GimbalAngle::SharedPtr msg)
{
    if (!std::isfinite(msg->yaw_abs) || !std::isfinite(msg->pitch_abs)) {
        gimbal_valid_ = false;
        return;
    }
    latest_gimbal_ = *msg;
    gimbal_received_ = true;
    gimbal_valid_ = true;
}

void PlannerNode::targetsCallback(const TrackedTargets::SharedPtr msg)
{
    builtin_interfaces::msg::Time stamp = msg->header.stamp;

    if (!gimbal_received_ || !gimbal_valid_) {
        publishInvalidCommand(stamp);
        return;
    }

    // 1. 选择目标（第一版只接受 TRACKING）
    auto index = target_selector_.selectIndex(*msg);
    if (!index.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    const auto & selected_target = msg->targets[index.value()];

    // 2. 使用 Tracker 提供的装甲板，不再用 ArmorGenerator 覆盖
    const auto & current_armors = selected_target.armors;
    if (current_armors.empty()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 3. 在当前帧选择正对射手的装甲板
    auto initial_armor = armor_selector_.select(current_armors, shooter_origin_, selected_target.center_world);
    if (!initial_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 4. 时间外推（第一版 dt=0，预测结果与输入一致）
    double prediction_time = 0.0;
    auto predicted = target_predictor_.predict(selected_target, prediction_time);

    // 5. 在预测帧中选择装甲板，优先保持同一 armor_id
    auto selected_armor = armor_selector_.select(predicted.armors, shooter_origin_, predicted.center_world);
    if (!selected_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 6. 弹道求解
    auto ballistic = ballistic_solver_.solve(selected_armor->position_world, shooter_origin_);

    // 7. 只有弹道有效时才生成指令
    AimCommand aim_cmd;
    PlannerDebug debug;
    debug.header.stamp = stamp;
    debug.selected_track_id = static_cast<int32_t>(selected_target.track_id);
    debug.selected_armor_id = selected_armor->armor_id;
    debug.original_point_world = initial_armor->position_world;
    debug.predicted_point_world = selected_armor->position_world;
    debug.prediction_time = prediction_time;
    debug.facing_score = armor_selector_.computeFacingScore(
        selected_armor->position_world, predicted.center_world, shooter_origin_);

    if (ballistic.valid) {
        command_generator_.setGimbalReceived(true);
        auto gimbal_delta = command_generator_.generate(
            ballistic.compensated_point, shooter_origin_, latest_gimbal_);

        // 最终有效性检查
        if (gimbal_valid_ &&
            std::isfinite(gimbal_delta.delta_yaw) &&
            std::isfinite(gimbal_delta.delta_pitch)) {
            aim_cmd.delta_yaw = gimbal_delta.delta_yaw;
            aim_cmd.delta_pitch = gimbal_delta.delta_pitch;
            aim_cmd.is_valid = true;

            debug.is_valid = true;
            debug.compensated_point_world = ballistic.compensated_point;
            debug.flight_time = ballistic.flight_time;
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

    aim_command_pub_->publish(aim_cmd);
    planner_debug_pub_->publish(debug);
}

void PlannerNode::publishInvalidCommand(const builtin_interfaces::msg::Time & stamp)
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
