#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
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
    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(robot->arm_positions_.at("ready"), 1.0)) {
        return "idel";
    }

    // 2. Wait for object pose from camera
    RCLCPP_INFO(robot->node_->get_logger(), "等待相机提供物体位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    int retry_count = 0;
    // while (!get_object_pose_in_base_frame(object_pose) && retry_count < 50) {
    //     std::this_thread::sleep_for(100ms);
    //     retry_count++;
    // }

    if (retry_count >= 50) {
        RCLCPP_ERROR(robot->node_->get_logger(), "从相机获取目标位姿失败");
        return "idel";
    }

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

    // // 3. Move to approach position (distance above target)
    // RCLCPP_INFO(node_->get_logger(), "移动到接近位置");
    // auto approach_pose = create_approach_pose(object_pose, -0.1);
    // execute_cartesian_space_trajectory(approach_pose, trajectory_duration_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    robot->set_air_pump(true);
    std::this_thread::sleep_for(100ms);
    // 4. Execute visual servo to grasp object
    // RCLCPP_INFO(node_->get_logger(), "开始视觉伺服抓取");
    // execute_visual_servo(object_pose);
    // wait_for_visual_servo_convergence(kVisualServoExitPositionToleranceMeters, kVisualServoConvergenceTimeoutSec);
    // visual_servo_active_ = false;
    // 5. Activate air pump to grasp object

    RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.6)) {
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
        return "idel";
    }

    // 6. Move back to ready position
    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(robot->arm_positions_.at("ready"), 1.5)) {
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");

    return "idel";
}
