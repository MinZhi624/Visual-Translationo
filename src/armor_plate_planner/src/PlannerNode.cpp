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
    this->declare_parameter("prediction_time", 0.0);
    this->declare_parameter("shooter_offset_x", 0.0);
    this->declare_parameter("shooter_offset_y", 0.0);
    this->declare_parameter("shooter_offset_z", 0.3);

    double bullet_speed = this->get_parameter("bullet_speed").as_double();
    double gravity = this->get_parameter("gravity").as_double();
    double max_face_angle = this->get_parameter("max_armor_face_angle").as_double();
    double sx = this->get_parameter("shooter_offset_x").as_double();
    double sy = this->get_parameter("shooter_offset_y").as_double();
    double sz = this->get_parameter("shooter_offset_z").as_double();

    ballistic_solver_ = BallisticSolver(bullet_speed, gravity);
    armor_selector_ = ArmorSelector(max_face_angle);

    shooter_origin_.x = sx;
    shooter_origin_.y = sy;
    shooter_origin_.z = sz;

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
    latest_gimbal_ = *msg;
}

void PlannerNode::targetsCallback(const TrackedTargets::SharedPtr msg)
{
    builtin_interfaces::msg::Time stamp = msg->header.stamp;

    auto track_id = target_selector_.select(*msg);
    if (!track_id.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    if (track_id.value() < 0 || track_id.value() >= static_cast<int32_t>(msg->targets.size())) {
        publishInvalidCommand(stamp);
        return;
    }

    const auto & selected_target = msg->targets[track_id.value()];

    auto predicted = target_predictor_.predict(selected_target, 0.0);

    auto armors = armor_generator_.generate(predicted);

    auto selected_armor = armor_selector_.select(armors, shooter_origin_);
    if (!selected_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    auto ballistic = ballistic_solver_.solve(selected_armor->position, shooter_origin_);

    auto gimbal_delta = command_generator_.generate(
        ballistic.compensated_point, shooter_origin_, latest_gimbal_);

    AimCommand aim_cmd;
    aim_cmd.delta_yaw = gimbal_delta.delta_yaw;
    aim_cmd.delta_pitch = gimbal_delta.delta_pitch;
    aim_cmd.is_valid = ballistic.valid;
    aim_command_pub_->publish(aim_cmd);

    PlannerDebug debug;
    debug.header.stamp = stamp;
    debug.selected_track_id = track_id.value();
    debug.selected_armor_id = selected_armor->number;
    debug.original_point_world = selected_armor->position;
    debug.predicted_point_world = selected_armor->position;
    debug.compensated_point_world = ballistic.compensated_point;
    debug.prediction_time = 0.0;
    debug.flight_time = ballistic.flight_time;
    debug.facing_score = 0.0;
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
