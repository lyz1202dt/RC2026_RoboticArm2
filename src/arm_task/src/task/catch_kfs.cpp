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
    }

    if (!robot->set_air_pump(0)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "夹爪关闭失败");
        return "idel";
    }
    // std::this_thread::sleep_for(1s);

    ready_position_name = "detach_interm_gan";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到过渡位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("抓取前移动到过渡位失败");
    }

    ready_position_name = "detach_gan_1";

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到放杆——1位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("移动到放杆——1位失败");
    }

    // TODO （请使用已有的函数实现一下功能，禁止新增函数实现和修改其他文件）:
    // 1. 获取当前机械臂的关节角度
    // 2. 通过关节角度计算当前机械臂的末端位姿，把位置保存到一个 eometry_msgs::msg::PoseStamped 里面












    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }

    return "idel";
}

