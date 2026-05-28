#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.h>
#include <thread>
#include <future>

using namespace std::chrono_literals;

CatchKFS::CatchKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

CatchKFS::~CatchKFS() {}


std::string CatchKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    if (!robot->set_grasp_state(false)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=0 失败");
    }

    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "catch_kfs 未获取到活动任务上下文，使用 TF 目标执行遥控抓取");
    }

    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    if (!robot->set_air_pump(1)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪开启失败");
        return "idel";
    }
    // std::this_thread::sleep_for(1s);

    std::vector<double> ready_joint_angles;
    std::string ready_position_name = "catch_gan_ready";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备抓杆位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("移动到准备抓杆位置失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到准备抓杆位置");
    }
    
    // 1. 获取当前末端位姿
    geometry_msgs::msg::PoseStamped current_end_pose;
    if (!robot->get_current_end_pose_from_arm_calc(current_end_pose)) {
        return fail_task("获取当前末端位姿失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功获取当前末端位姿");
        RCLCPP_INFO(robot->node_->get_logger(), "当前末端位置: (%.3f, %.3f, %.3f)； 当前末端姿态: (%.3f, %.3f, %.3f, %.3f)", 
        current_end_pose.pose.position.x, current_end_pose.pose.position.y, current_end_pose.pose.position.z, 
        current_end_pose.pose.orientation.x, current_end_pose.pose.orientation.y, current_end_pose.pose.orientation.z, 
        current_end_pose.pose.orientation.w);
    }

    // 2.1 从 TF 读取目标位置，姿态复用当前末端
    geometry_msgs::msg::PoseStamped target_pose;
    try {
        auto tf = robot->tf_buffer_->lookupTransform(
            robot->base_frame_, robot->object_frame_,
            tf2::TimePointZero, std::chrono::milliseconds(500));
        target_pose.header.frame_id = robot->base_frame_;
        target_pose.header.stamp = robot->node_->now();
        target_pose.pose.position.x = -tf.transform.translation.x;
        target_pose.pose.position.y = tf.transform.translation.y;
        target_pose.pose.position.z = tf.transform.translation.z;
        target_pose.pose.orientation = current_end_pose.pose.orientation;

        RCLCPP_INFO(robot->node_->get_logger(), "目标位置: (%.3f, %.3f, %.3f)； 目标姿态: (%.3f, %.3f, %.3f, %.3f)", 
        target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z, 
        target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, 
        target_pose.pose.orientation.w
        );
    } catch (const tf2::TransformException& ex) {
        return fail_task("获取目标位置TF失败");
    }

    // 2.2 设置目标高度为 grasp_height 参数
    double grasp_height = 0.7;
    robot->node_->get_parameter("grasp_height", grasp_height);
    target_pose.pose.position.z = grasp_height;







    
    // 2.3 启动视觉伺服，等待收敛
    if (!robot->start_visual_servo(target_pose)) {
        return fail_task("启动视觉伺服失败");
    }
    while (rclcpp::ok() && robot->is_visual_servo_active()) {
        if (robot->is_visual_servo_converged(0.03)) {
            RCLCPP_INFO(robot->node_->get_logger(), "视觉伺服已收敛");
            break;
        }
        std::this_thread::sleep_for(50ms);
    }
    if (!robot->is_visual_servo_active()) {
        return fail_task("视觉伺服被外部取消");
    }
    robot->stop_visual_servo();





    // 3. 关闭夹爪
    if (!robot->set_air_pump(0)) {
        return fail_task("关闭夹爪失败");
    }

    ready_position_name = "detach_gan";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备抓杆位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("移动到准备抓杆位置失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到准备抓杆位置");
    }





    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }

    return "idel";
}

