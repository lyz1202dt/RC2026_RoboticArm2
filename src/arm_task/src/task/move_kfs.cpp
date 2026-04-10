#include "task/move_kfs.hpp"
#include "robot.hpp"
#include <rclcpp/rclcpp.hpp>

MoveKFS::MoveKFS(Robot* context, const std::string name): BaseTask(context, name) {

}

MoveKFS::~MoveKFS() {}

std::string MoveKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(robot->node_->get_logger(), "move_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    return "idel";
}