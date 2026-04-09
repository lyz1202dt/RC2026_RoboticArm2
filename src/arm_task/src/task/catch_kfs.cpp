#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.h>
#include <thread>

using namespace std::chrono_literals;

CatchKFS::CatchKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

CatchKFS::~CatchKFS() {}


std::string CatchKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(robot->node_->get_logger(), "catch_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    const auto goal_handle = context.goal_handle;
    std::vector<double> ready_joint_angles;
    if (!robot->get_named_joint_position("ready", ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [ready]");
        robot->finish_current_task(goal_handle, false, "未找到命名位姿 ready");
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 1.0)) {
        robot->finish_current_task(goal_handle, false, "抓取前移动到准备位失败");
        return "idel";
    }

    // 2. Wait for object pose from camera
    RCLCPP_INFO(robot->node_->get_logger(), "等待相机提供物体位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    if (context.data.size() != (3 + 4)) {
        RCLCPP_ERROR(
            robot->node_->get_logger(), "接收到的目标位姿数据维度不正确，预期为3+4，实际为%zu", context.data.size());
        robot->finish_current_task(goal_handle, false, "接收到的目标位姿数据维度不正确");
        return "idel";
    }
    //填写位姿数据
    object_pose.header.frame_id = "base_link";
    object_pose.header.stamp = robot->node_->now();
    object_pose.pose.position.x = context.data[0];
    object_pose.pose.position.y = context.data[1];
    object_pose.pose.position.z = context.data[2];
    object_pose.pose.orientation.x = context.data[3];
    object_pose.pose.orientation.y = context.data[4];
    object_pose.pose.orientation.z = context.data[5];
    object_pose.pose.orientation.w = context.data[6];
    

    RCLCPP_INFO(
        robot->node_->get_logger(), "物体在坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    // 强制规定姿态
    tf2::Quaternion quat;
    quat.setRPY(0, (M_PI / 2), 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    if (!robot->set_air_pump(true)) {
        robot->finish_current_task(goal_handle, false, "气泵开启失败");
        return "idel";
    }
    std::this_thread::sleep_for(100ms);

    RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.6)) {
        robot->finish_current_task(goal_handle, false, "执行抓取轨迹失败");
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "向前推进一段距离以确保吸取稳固");
    object_pose.pose.position.x+=0.06;
    quat.setRPY(0, (M_PI / 2)-0.25, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    if (!robot->execute_cartesian_space_trajectory(object_pose, 1.0)) {
        robot->finish_current_task(goal_handle, false, "补推进失败");
        return "idel";
    }

    // 6. Move back to ready position
    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 1.5)) {
        robot->finish_current_task(goal_handle, false, "抓取完成后返回准备位失败");
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    robot->finish_current_task(goal_handle, true, "抓取流程执行完成");

    return "idel";
}
