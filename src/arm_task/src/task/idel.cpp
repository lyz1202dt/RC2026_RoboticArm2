#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>

namespace {
constexpr int32_t kTaskCatchTarget = 2;
constexpr int32_t kTaskPlaceTarget = 3;
constexpr int32_t kTaskMoveTarget = 1;
}

IdelTask::IdelTask(Robot* context,const std::string name) : BaseTask(context,name)
{
    
}

IdelTask::~IdelTask()
{

}

std::string IdelTask::process(const std::string last_task_name)
{
    (void)last_task_name;

    // // TODO: 移动到ready位置
    // std::vector<double> ready_joint_angles;
    // robot->get_named_joint_position("ready", ready_joint_angles);
    // robot->execute_joint_space_trajectory(ready_joint_angles, 1.0);

    if (!robot->wait_for_idle_signal(std::chrono::milliseconds(1000))) {
        RCLCPP_INFO(robot->node_->get_logger(), "idel 等待新任务信号超时，继续等待");
        return "idel";
    }

    Robot::PendingTaskRequest request;
    if (!robot->take_pending_task(request)) {
        RCLCPP_INFO(robot->node_->get_logger(), "获取挂起的任务失败，继续等待");
        return "idel";
    }

    switch (request.task_id) {
        case kTaskCatchTarget:
            RCLCPP_INFO(robot->node_->get_logger(), "idel 收到抓取任务，切换到 catch_kfs");
            return "catch_kfs";
        case kTaskMoveTarget:
            RCLCPP_WARN(robot->node_->get_logger(), "idel 收到移动任务，切换到 move_kfs");
            return "move_kfs";
        case kTaskPlaceTarget:
            RCLCPP_WARN(robot->node_->get_logger(), "idel 收到放置任务，切换到 place_kfs");
            return "place_kfs";
        default:
            RCLCPP_WARN(robot->node_->get_logger(), "未知 task_id=%d，无法分发任务", request.task_id);
            robot->finish_current_task(request.goal_handle, false, "未知 task_id，无法分发任务");
            return "idel";
    }

    return "idel";
}
