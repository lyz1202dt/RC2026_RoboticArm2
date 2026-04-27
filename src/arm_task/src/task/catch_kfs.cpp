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

// ============================================================
// 辅助结构体：抓取配置参数
// ============================================================
struct GraspConfig {
    double grasp_height;          // 抓取高度
    double down_offset;           // 垂直下压偏移量
    double lateral_offset;        // 横向偏移量
    double forward_offset;        // 前进偏移量
    std::string ready_position;   // 准备位名称
    std::string detach_position;  // 释放位名称
};

// 获取指定抓取高度的配置参数
static GraspConfig get_grasp_config(double grasp_height_param, Robot* robot, bool use_action_context = false) {
    GraspConfig config;

    if (grasp_height_param == 0.0) {
        config.grasp_height = use_action_context ? 0.10 : -0.02;
        config.down_offset = 0.21;
        config.lateral_offset = 0.10;
        config.forward_offset = -0.05;
        config.ready_position = "ready_down200";
        config.detach_position = "kfs_detach";
    } else if (grasp_height_param == 1.0) {
        config.grasp_height = use_action_context ? 0.51 : 0.41;
        config.down_offset = robot->node_->get_parameter("grasp_down_run").as_double();
        config.lateral_offset = robot->node_->get_parameter("grasp_right_run").as_double();
        config.forward_offset = 0.0;
        config.ready_position = "ready";
        config.detach_position = "kfs_detach";
    } else if (grasp_height_param == 2.0) {
        config.grasp_height = use_action_context ? 0.68 : 0.68;
        config.down_offset = robot->node_->get_parameter("grasp_down_run").as_double();
        config.lateral_offset = robot->node_->get_parameter("grasp_right_run").as_double();
        config.forward_offset = 0.0;
        config.ready_position = "ready_up400";
        config.detach_position = "kfs_up400_detach";
    } else {
        RCLCPP_ERROR(robot->node_->get_logger(), "未知的 grasp_height 参数值: %lf", grasp_height_param);
        config.grasp_height = -1.0;  // 无效值标记
    }

    return config;
}

CatchKFS::CatchKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

CatchKFS::~CatchKFS() {}


std::string CatchKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    // ============================================================
    // 前置检查
    // ============================================================
    if (!robot->set_grasp_state(false)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=0 失败");
    }

    if (robot->current_kfs_num_ == 1) {
        RCLCPP_WARN(robot->node_->get_logger(), "当前车上 KFS 数量为 1，不能再执行抓取任务");
        return "idel";
    }

    // 获取任务上下文
    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "catch_kfs 未获取到活动任务上下文，使用 TF 目标执行遥控抓取");
    }

    // 获取抓取高度参数
    const double grasp_height_param = robot->node_->get_parameter("grasp_height").as_double();

    // ============================================================
    // 阶段1: 移动到过渡位置（仅当 grasp_height == 0.0）
    // ============================================================
    if (grasp_height_param == 0.0) {
        std::vector<double> detach_pos;
        if (!robot->get_named_joint_position("ready_interm", detach_pos)) {
            RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [ready_interm]");
            if (goal_handle) {
                robot->finish_current_task(goal_handle, false, "未找到命名位姿 ready_interm");
            }
            return "idel";
        }

        RCLCPP_INFO(robot->node_->get_logger(), "移动到过渡位置 [ready_interm]");
        if (!robot->execute_joint_space_trajectory(detach_pos, 0.8)) {
            if (goal_handle) {
                robot->finish_current_task(goal_handle, false, "移动到过渡位置失败");
            }
            return "idel";
        }
    }

    // ============================================================
    // 阶段2: 获取抓取配置并移动到准备位置
    // ============================================================
    GraspConfig config = get_grasp_config(grasp_height_param, robot);
    if (config.grasp_height < 0) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "无效的 grasp_height 参数");
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "抓取高度配置: %.2f", grasp_height_param);

    std::vector<double> ready_joint_angles;
    if (!robot->get_named_joint_position(config.ready_position, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", config.ready_position.c_str());
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + config.ready_position);
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置 [%s]", config.ready_position.c_str());
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 2.0)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "移动到准备位失败");
        }
        return "idel";
    }

    std::this_thread::sleep_for(5s);

    // ============================================================
    // 阶段3: 获取目标位姿
    // ============================================================
    RCLCPP_INFO(robot->node_->get_logger(), "准备获取抓取目标位姿");
    geometry_msgs::msg::PoseStamped object_pose;

    if (has_action_context) {
        // 使用 Action 上下文数据
        if (context.data.size() != (3 + 4 + 1)) {
            RCLCPP_ERROR(robot->node_->get_logger(),
                "接收到的目标位姿数据维度不正确，预期为3+4+1，实际为%zu", context.data.size());
            if (goal_handle) {
                robot->finish_current_task(goal_handle, false, "接收到的目标位姿数据维度不正确");
            }
            return "idel";
        }

        object_pose.header.frame_id = "base_link";
        object_pose.header.stamp = robot->node_->now();
        object_pose.pose.position.x = context.data[0];
        object_pose.pose.position.y = context.data[1];
        object_pose.pose.position.z = config.grasp_height;
        object_pose.pose.orientation.x = context.data[3];
        object_pose.pose.orientation.y = context.data[4];
        object_pose.pose.orientation.z = context.data[5];
        object_pose.pose.orientation.w = context.data[6];
    } else {
        // 使用 TF 变换数据
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
            object_pose.pose.position.z = config.grasp_height;
            object_pose.pose.orientation = target_tf.transform.rotation;

            RCLCPP_INFO(robot->node_->get_logger(), "=====================================");
            RCLCPP_INFO(robot->node_->get_logger(), "目标位置: [%.3f, %.3f, %.3f]",
                object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);
            RCLCPP_INFO(robot->node_->get_logger(), "目标姿态: [%.3f, %.3f, %.3f, %.3f]",
                object_pose.pose.orientation.x, object_pose.pose.orientation.y,
                object_pose.pose.orientation.z, object_pose.pose.orientation.w);
            RCLCPP_INFO(robot->node_->get_logger(), "=====================================");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(robot->node_->get_logger(), "读取 TF 抓取目标失败: %s", e.what());
            return "idel";
        }
    }

    RCLCPP_INFO(robot->node_->get_logger(), "物体在坐标: [%.3f, %.3f, %.3f]",
        object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

    // ============================================================
    // 阶段4: 执行抓取动作
    // ============================================================
    // 设置目标姿态并应用横向偏移
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.5, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    object_pose.pose.position.x -= config.lateral_offset;

    // 开启气泵
    if (!robot->set_air_pump(true)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "气泵开启失败");
        }
        return "idel";
    }

    // 执行抓取轨迹
    RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.5)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "执行抓取轨迹失败");
        }
        return "idel";
    }

    // 执行按压动作
    quat.setRPY(0, M_PI / 2.2, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    object_pose.pose.position.z -= config.down_offset;
    object_pose.pose.position.x += 0.12 + config.forward_offset;

    RCLCPP_INFO(robot->node_->get_logger(), "执行按压动作");
    if (!robot->execute_cartesian_space_trajectory(object_pose, 3.0)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "执行按压轨迹失败");
        }
        return "idel";
    }

    // 更新 KFS 计数
    robot->current_kfs_num_ += 1;

    // ============================================================
    // 阶段5: 抬起并后退
    // ============================================================
    quat.setRPY(0, M_PI / 1.8, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    object_pose.pose.position.z += 0.07;

    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.4)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "抬起动作失败");
        }
        return "idel";
    }

    // 直线后退
    if (grasp_height_param == 1.0 || grasp_height_param == 2.0) {
        object_pose.pose.position.x -= 0.3;
    }

    if (!robot->execute_cartesian_space_trajectory(object_pose, 0.6)) {
        if (goal_handle) {object_pose.pose.position.x-=0.3;
            robot->finish_current_task(goal_handle, false, "后退动作失败");
        }
        return "idel";
    }

    // ============================================================
    // 阶段6: 移动到释放位置
    // ============================================================
    std::vector<double> detach_pos;
    if (!robot->get_named_joint_position(config.detach_position, detach_pos)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", config.detach_position.c_str());
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + config.detach_position);
        }
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到释放位置 [%s]", config.detach_position.c_str());
    if (!robot->execute_joint_space_trajectory(detach_pos, 1.0)) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, "移动到释放位置失败");
        }
        return "idel";
    }

    // 等待稳定
    std::this_thread::sleep_for(1s);

    // ============================================================
    // 阶段7: 完成抓取流程
    // ============================================================
    RCLCPP_INFO(robot->node_->get_logger(), "抓取流程完成");
    if (!robot->set_grasp_state(true)) {
        RCLCPP_WARN(robot->node_->get_logger(), "设置 grasp_state=1 失败");
    }
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "抓取流程执行完成");
    }

    return "idel";
}
