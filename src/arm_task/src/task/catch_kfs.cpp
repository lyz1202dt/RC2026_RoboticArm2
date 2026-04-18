#include "task/catch_kfs.hpp"
#include "robot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
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

    if (robot->current_kfs_num_ == 1) {
        RCLCPP_WARN(robot->node_->get_logger(), "当前车上 KFS 数量为 1 ，不能再执行抓取任务。");
        return "idel";
    }


    
    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "catch_kfs 未获取到活动任务上下文，使用 TF 目标执行遥控抓取");
    }

    std::vector<double> ready_joint_angles;
    
    std::string ready_position_name = "ready";
    if (robot->grasp_kfs_num_ == 1 || robot->grasp_kfs_num_ == 2 || 
        robot->grasp_kfs_num_ == 3 || robot->grasp_kfs_num_ == 4 ||
        robot->grasp_kfs_num_ == 5 || robot->grasp_kfs_num_ == 7) {
        ready_position_name = "ready_up200";
    } else if (robot->grasp_kfs_num_ == 11 || robot->grasp_kfs_num_ == 12 || 
               robot->grasp_kfs_num_ == 8 || robot->grasp_kfs_num_ == 9 || robot->grasp_kfs_num_ == 10) {
        ready_position_name = "ready_down200";
    } else if (robot->grasp_kfs_num_ == 13 || robot->grasp_kfs_num_ == 14 || 
               robot->grasp_kfs_num_ == 15) {
        ready_position_name = "ready_up400";
    }

    if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + ready_position_name);
        }
        return "idel";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::string detach_pos_name_ = "ready_interm";
    std::vector<double> detach_pos_;
    if (!robot->get_named_joint_position(detach_pos_name_, detach_pos_)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", detach_pos_name_.c_str());
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + detach_pos_name_);
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到释放位置 [%s]", detach_pos_name_.c_str());
    if (!robot->execute_joint_space_trajectory(detach_pos_, 4)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "移动到释放位置失败");
        }
        return "idel";
    }


    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 4.0)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "抓取前移动到准备位失败");
        }
        return "idel";
    }

    std::this_thread::sleep_for(300ms);

    // 2. 获取目标位姿：优先使用 action 数据，否则使用 TF
    RCLCPP_INFO(robot->node_->get_logger(), "准备获取抓取目标位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    if (has_action_context) {
        if (context.data.size() != (3 + 4)) {
            RCLCPP_ERROR(
                robot->node_->get_logger(), "接收到的目标位姿数据维度不正确，预期为3+4，实际为%zu", context.data.size());
            if (goal_handle) {
                robot->finish_current_task(goal_handle, false, "接收到的目标位姿数据维度不正确");
            }
            return "idel";
        }

        object_pose.header.frame_id = "base_link";
        object_pose.header.stamp = robot->node_->now();
        object_pose.pose.position.x = context.data[0];
        object_pose.pose.position.y = context.data[1];
        object_pose.pose.position.z = context.data[2];
        object_pose.pose.orientation.x = context.data[3];
        object_pose.pose.orientation.y = context.data[4];
        object_pose.pose.orientation.z = context.data[5];
        object_pose.pose.orientation.w = context.data[6];
    } else {
        try {
            if (!robot->tf_buffer_->canTransform("base_link", robot->object_frame_, tf2::TimePointZero, 2s)) {
                RCLCPP_ERROR(robot->node_->get_logger(), "等待 TF base_link -> %s 超时", robot->object_frame_.c_str());
                return "idel";
            }

            const geometry_msgs::msg::TransformStamped target_tf =
                robot->tf_buffer_->lookupTransform("base_link", robot->object_frame_, tf2::TimePointZero);
            object_pose.header.frame_id = "base_link";
            object_pose.header.stamp = robot->node_->now();
            object_pose.pose.position.x = target_tf.transform.translation.x;
            object_pose.pose.position.y = target_tf.transform.translation.y;
            object_pose.pose.position.z = target_tf.transform.translation.z;
            object_pose.pose.orientation = target_tf.transform.rotation;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(robot->node_->get_logger(), "读取 TF 抓取目标失败: %s", e.what());
            return "idel";
        }
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

    if (!robot->set_air_pump(true)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "气泵开启失败");
        }
        return "idel";
    }
    std::this_thread::sleep_for(100ms);

    RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.6)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "执行抓取轨迹失败");
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "向前推进一段距离以确保吸取稳固");
    object_pose.pose.position.x+=0.06;
    // object_pose.pose.position.z+=0.125;
    quat.setRPY(0, (M_PI / 2), 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    if (!robot->execute_cartesian_space_trajectory(object_pose, 1.5)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "补推进失败");
        }
        return "idel";
    }


    robot->current_kfs_num_ += 1;

    // 直线后退0.3m
    object_pose.pose.position.x-=0.15;
    if (!robot->execute_cartesian_space_trajectory(object_pose, 1.5)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "后退失败");
        }
        return "idel";
    }



    

    // 3. 移动到 "kfs" + std::to_string(robot->current_kfs_num_) + "_detach_pos"
    std::string detach_pos_name = "kfs" + std::to_string(robot->current_kfs_num_) + "_detach_pos";
    std::vector<double> detach_pos;
    if (!robot->get_named_joint_position(detach_pos_name, detach_pos)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", detach_pos_name.c_str());
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + detach_pos_name);
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到释放位置 [%s]", detach_pos_name.c_str());
    if (!robot->execute_joint_space_trajectory(detach_pos, 4)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "移动到释放位置失败");
        }
        return "idel";
    }

    // 4. 等待 1s
    std::this_thread::sleep_for(1s);

    // // 5. 关闭气泵
    // RCLCPP_INFO(robot->node_->get_logger(), "关闭气泵");
    // if (!robot->set_air_pump(false)) {
    //     robot->finish_current_task(goal_handle, false, "气泵关闭失败");
    //     return "idel";
    // }






    // // 6. Move back to ready position
    // RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    // if (!robot->execute_joint_space_trajectory(ready_joint_angles, 0.9)) {
    //     if (goal_handle) {
    //         robot->finish_current_task(goal_handle, false, "抓取完成后返回准备位失败");
    //     }
    //     return "idel";
    // }

    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }

    return "idel";
}
