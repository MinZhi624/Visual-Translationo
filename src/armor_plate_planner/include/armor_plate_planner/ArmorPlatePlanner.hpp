#pragma once

#include "armor_plate_planner/TargetSelector.hpp"
#include "armor_plate_planner/TargetPredictor.hpp"
#include "armor_plate_planner/ArmorSelector.hpp"
#include "armor_plate_planner/BallisticSolver.hpp"
#include "armor_plate_planner/CommandGenerator.hpp"

#include <armor_plate_interfaces/msg/tracked_targets.hpp>
#include <armor_plate_interfaces/msg/aim_command.hpp>
#include <armor_plate_interfaces/msg/planner_debug.hpp>
#include <armor_plate_interfaces/msg/gimbal_angle.hpp>
#include <armor_plate_interfaces/GimbalData.hpp>

#include <armor_plate_common/thread_safe_queue.hpp>
#include <rclcpp/rclcpp.hpp>

class ArmorPlatePlanner : public rclcpp::Node
{
private:
    TargetSelector target_selector_;
    TargetPredictor target_predictor_;
    ArmorSelector armor_selector_;
    BallisticSolver ballistic_solver_;
    CommandGenerator command_generator_;

    rclcpp::Subscription<armor_plate_interfaces::msg::TrackedTargets>::SharedPtr targets_sub_;
    rclcpp::Subscription<armor_plate_interfaces::msg::GimbalAngle>::SharedPtr gimbal_sub_;

    rclcpp::Publisher<armor_plate_interfaces::msg::AimCommand>::SharedPtr aim_command_pub_;
    rclcpp::Publisher<armor_plate_interfaces::msg::PlannerDebug>::SharedPtr planner_debug_pub_;

    // ===== 云台数据队列（生产者-消费者） ===== //
    ThreadSafeQueue<armor_plate_interfaces::msg::GimbalAngle, true> gimbal_queue_{1};

    // ===== 指令构建上下文 ===== //
    struct CommandContext {
        const TargetState& target;
        const PlannerArmor& initial_armor;
        const PlannerArmor& selected_armor;
        const BallisticResult& ballistic_result;
        const GimbalData& gimbal;
        double prediction_time = 0.0;
    };

    void init();
    void targetsCallback(const armor_plate_interfaces::msg::TrackedTargets::SharedPtr msg);
    void gimbalCallback(const armor_plate_interfaces::msg::GimbalAngle::SharedPtr msg);

    // 构建指令
    std::pair<armor_plate_interfaces::msg::AimCommand, armor_plate_interfaces::msg::PlannerDebug>
    buildCommand(const CommandContext& ctx, const builtin_interfaces::msg::Time& stamp);

    void publishInvalidCommand(const builtin_interfaces::msg::Time& stamp);

public:
    ArmorPlatePlanner();
};
