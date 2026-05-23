#include "task/move_kfs.hpp"
#include "robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>

MoveKFS::MoveKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

MoveKFS::~MoveKFS() {}

std::string MoveKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "move_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    // Support legacy format (7 values: 6 joints + duration) and optional
    // extended format (8 values: 6 joints + duration + pump_switch).
    if (context.data.size() != 7 && context.data.size() != 8) {
        RCLCPP_ERROR(robot->node_->get_logger(), "接收到的移动任务数据维度不正确，预期为7或8，实际为%zu", context.data.size());
        return fail_task("接收到的移动任务数据维度不正确");
    }

    const double duration_ = context.data[6];
    const std::vector<double> joint_angles = {
        context.data[0],
        context.data[1],
        context.data[2],
        context.data[3],
        context.data[4],
        context.data[5],
    };

    // If an optional pump switch is provided, apply it before executing trajectory.
    bool pump_requested = false;
    int pump_enable = false;
    if (context.data.size() == 8) {
        pump_requested = true;
        pump_enable = (static_cast<int>(context.data[7]));
        RCLCPP_INFO(robot->node_->get_logger(), "移动任务包含气泵开关: %d", pump_enable);
        if (!robot->set_air_pump(pump_enable)) {
            RCLCPP_WARN(robot->node_->get_logger(), "设置气泵状态失败 (requested=%d)", pump_enable);
        }
    }

    RCLCPP_INFO(robot->node_->get_logger(), "执行关节空间移动任务");
    if (!robot->execute_joint_space_trajectory(joint_angles, duration_)) {
        return fail_task("执行移动轨迹失败");
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动流程完成");
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "移动流程执行完成");
    }

    // Note: we intentionally do not automatically disable the pump here.
    // If caller provided pump switch and wants it turned off after motion,
    // they should send pump=0 in a subsequent task or rely on task logic.

    return "idel";
}