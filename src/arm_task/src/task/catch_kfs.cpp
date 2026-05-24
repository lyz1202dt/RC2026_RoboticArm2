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
        return fail_task("移动到抓杆位失败");
    } else {
        RCLCPP_INFO(robot->node_->get_logger(), "成功移动到准备抓杆位置");
    }
    
    // TODO:
    // 1.获取当前末端的位姿（get_current_end_pose_from_arm_calc）。
    // 2.打开夹爪并开始视觉伺服
    //   - 2.1 从tf读取目标位置，姿态复用当前末端姿态，构造目标位姿
    //   - 2.2 设置目标位置高度为一个固定值（比如0.7米）。
    //   - 2.3 执行视觉伺服，等待收敛
    // 3.视觉伺服收敛成功后，关闭夹爪。
    // 要求：
    // 1.全程使用现有函数和api，不要自己另造函数
    // 2.不要修改其他文件







    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }

    return "idel";
}

