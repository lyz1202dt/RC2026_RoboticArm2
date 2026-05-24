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
    std::string ready_position_name = "catch_gan";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到抓杆位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("移动到抓杆位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到抓杆位置");
    }
    std::this_thread::sleep_for(10s);

     // 等待 1 秒钟，确保机械臂稳定在抓杆位置

    if (!robot->set_air_pump(0)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪关闭失败");
        return "idel";
    }
    std::this_thread::sleep_for(1s);

    ready_position_name = "detach_interm_gan";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到过渡位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 2.0)) { // 1.0
        return fail_task("移动到过渡位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到过渡位置");
    }

    ready_position_name = "detach_gan_1";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到放杆——1位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 2.0)) { // 1.0
        return fail_task("移动到放杆——1位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到放杆——1位置");
    }

    ready_position_name = "detach_gan_2";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到放杆——2位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 2.0)) { // 1.0
        return fail_task("移动到放杆——2位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到放杆——2位置");
    }

    ready_position_name = "detach_gan_end";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    // 同时执行：轨迹在后台运行，气泵同步设置
    auto traj_future = std::async(std::launch::async, [&]() {
        return robot->execute_joint_space_trajectory(ready_joint_angles, 3.0);
    });

    // if (!robot->set_air_pump(2)) {
    //     RCLCPP_ERROR(robot->node_->get_logger(), "夹爪微型开启失败");
    // }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到放杆——end位置");
    if (!traj_future.get()) {
        return fail_task("移动到放杆——end位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到放杆——end位置");
    }

    if (!robot->set_air_pump(0)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪关闭失败");
    }




    // 1. 获取当前机械臂的关节角度（从硬件反馈获取实际值）
    std::vector<double> current_joints;
    if (!robot->get_current_joint_positions(current_joints)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "获取当前关节角度失败");
        return fail_task("获取当前关节角度失败");
    }
    RCLCPP_INFO(robot->node_->get_logger(),
        "当前关节角度: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        current_joints[0], current_joints[1], current_joints[2],
        current_joints[3], current_joints[4], current_joints[5]);

    // 2. 通过关节角度计算当前机械臂的末端位姿，保存到 PoseStamped
    geometry_msgs::msg::PoseStamped current_end_pose;
    std::string fk_message;
    if (!robot->forward_kinematics(current_joints, current_end_pose, &fk_message)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "正运动学求解失败: %s", fk_message.c_str());
        return fail_task("正运动学求解失败: " + fk_message);
    }

    tf2::Quaternion fk_quat;
    tf2::fromMsg(current_end_pose.pose.orientation, fk_quat);
    double fk_roll, fk_pitch, fk_yaw;
    tf2::Matrix3x3(fk_quat).getRPY(fk_roll, fk_pitch, fk_yaw);

    RCLCPP_INFO(robot->node_->get_logger(),
        "当前末端位姿(FK): position=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
        current_end_pose.pose.position.x,
        current_end_pose.pose.position.y,
        current_end_pose.pose.position.z,
        fk_roll, fk_pitch, fk_yaw);

    
    current_end_pose.pose.position.z -= 0.2;
    
    if (!robot->set_air_pump(2)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪微型开启失败");
    }

    if (!robot->execute_cartesian_space_trajectory(current_end_pose, 3.0)) {
        return fail_task("执行笛卡尔空间轨迹失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功执行笛卡尔空间轨迹");
    }

    // std::this_thread::sleep_for(5s);
    if (!robot->set_air_pump(0)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪关闭失败");
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

