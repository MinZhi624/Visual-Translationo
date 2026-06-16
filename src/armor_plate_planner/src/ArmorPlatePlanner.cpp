#include "armor_plate_planner/ArmorPlatePlanner.hpp"

using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::TrackedTargets;
using armor_plate_interfaces::msg::PlannerDebug;
using armor_plate_interfaces::msg::GimbalAngle;

namespace {

GimbalData GimbalAngleToData(const GimbalAngle & msg) {
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

    // 消费者：非阻塞取最新云台数据，转换为 GimbalData
    GimbalAngle gimbal_msg;
    if (!gimbal_queue_.pop(gimbal_msg, std::chrono::milliseconds(0))) {
        publishInvalidCommand(stamp);
        return;
    }
    GimbalData current_gimbal = GimbalAngleToData(gimbal_msg);

    //////////  车辆选择 /////////
     
    // 选择目标（第一版只接受 TRACKING） -- 同时也只是选则一个
    auto index = target_selector_.selectIndex(*msg);
    if (!index.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    ////////// 装甲板选择 /////////
    const auto & selected_target = msg->targets[index.value()];

    const auto & current_armors = selected_target.armors;
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
    /*
        TODO:
        用来实现时间外推
        时间推到现在的情况，然后选择装甲板
        现在没有实现，返回当前结果
    */
    double prediction_time = 0.0;
    auto predicted = target_predictor_.predict(selected_target, prediction_time);

    // 在预测帧中选择装甲板，优先保持同一 armor_id -- 这里先看一下外推效果(还没实现）)
    auto selected_armor = armor_selector_.select(predicted.armors);
    if (!selected_armor.has_value()) {
        publishInvalidCommand(stamp);
        return;
    }

    // 弹道求解
    auto ballistic_result = ballistic_solver_.solve(selected_armor->position_world);

    // 只有弹道有效时才生成指令
    AimCommand aim_cmd;
    PlannerDebug debug;
    debug.header.stamp = stamp;
    debug.selected_track_id = static_cast<int32_t>(selected_target.track_id);
    debug.selected_armor_id = selected_armor->armor_id;
    debug.original_point_world = initial_armor->position_world;
    debug.predicted_point_world = selected_armor->position_world;
    debug.prediction_time = prediction_time;
    debug.facing_score = armor_selector_.computeFacingScore(
        selected_armor->position_world, selected_armor->yaw_world);

    if (ballistic_result.valid) {
        command_generator_.setGimbalReceived(true);
        auto gimbal_delta = command_generator_.generate(
            ballistic_result.compensated_point, current_gimbal);

        // 最终有效性检查
        if (std::isfinite(gimbal_delta.delta_yaw) &&
            std::isfinite(gimbal_delta.delta_pitch)) {
            aim_cmd.delta_yaw = gimbal_delta.delta_yaw;
            aim_cmd.delta_pitch = gimbal_delta.delta_pitch;
            aim_cmd.is_valid = true;

            debug.is_valid = true;
            debug.compensated_point_world = ballistic_result.compensated_point;
            debug.flight_time = ballistic_result.flight_time;
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

void ArmorPlatePlanner::init()
{
    // ===== 参数获取 ===== //
    double bullet_speed = this->declare_parameter<double>("bullet_speed", 25.0);
    double gravity = this->declare_parameter<double>("gravity", 9.81);
    double max_face_angle = this->declare_parameter<double>("max_armor_face_angle", 1.0472);

    ballistic_solver_ = BallisticSolver(bullet_speed, gravity);
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
