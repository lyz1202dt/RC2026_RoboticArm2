#include <task/visual_serve.hpp>
#include "robot.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

VisualServe::VisualServe(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

VisualServe::~VisualServe() {}

std::string VisualServe::process(const std::string last_task_name) {
    (void)last_task_name;

    // 获取活动任务上下文
    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;

    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "visual_serve 未获取到活动任务上下文，使用 TF 目标执行视觉伺服");
    }

    // 定义失败处理 lambda
    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    try {
        // 1. 直接从 TF 获取目标位姿
        if (!robot->tf_buffer_->canTransform("base_link", robot->object_frame_, tf2::TimePointZero, 2s)) {
            RCLCPP_ERROR(robot->node_->get_logger(), "等待 TF base_link -> %s 超时", robot->object_frame_.c_str());
            return fail_task("获取目标 TF 变换超时");
        }

        const geometry_msgs::msg::TransformStamped target_tf =
            robot->tf_buffer_->lookupTransform("base_link", robot->object_frame_, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped object_pose;
        object_pose.header.frame_id = "base_link";
        object_pose.header.stamp = robot->node_->now();
        object_pose.pose.position.x = target_tf.transform.translation.x;
        object_pose.pose.position.y = -(target_tf.transform.translation.y + 0.05);
        object_pose.pose.position.z = target_tf.transform.translation.z;

        // 设置姿态为 x 轴向下的抓取姿态
        tf2::Quaternion quat;
        quat.setRPY(0.0, M_PI / 2.0, 0.0);
        object_pose.pose.orientation.w = quat.getW();
        object_pose.pose.orientation.x = quat.getX();
        object_pose.pose.orientation.y = quat.getY();
        object_pose.pose.orientation.z = quat.getZ();

        RCLCPP_INFO(robot->node_->get_logger(), "目标位置: [%.3f, %.3f, %.3f]",
                    object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

        // 2. 使用视觉伺服控制机械臂移动到目标位置
        RCLCPP_INFO(robot->node_->get_logger(), "开始视觉伺服控制");

        // 视觉伺服参数
        const double max_linear_velocity = 0.1;  // 最大线速度 (m/s)
        const double max_angular_velocity = 0.5; // 最大角速度 (rad/s)
        const double position_tolerance = 0.01;  // 位置公差 (m)
        const double orientation_tolerance = 0.05; // 姿态公差 (rad)
        const double timeout = 30.0; // 视觉伺服超时 (s)

        auto start_time = std::chrono::steady_clock::now();
        bool servo_completed = false;

        while (!servo_completed) {
            // 检查超时
            auto elapsed_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            if (elapsed_time > timeout) {
                return fail_task("视觉伺服超时");
            }

            // 获取当前末端执行器位置 (通过 TF 查询)
            try {
                const geometry_msgs::msg::TransformStamped current_tf =
                    robot->tf_buffer_->lookupTransform("base_link", robot->tip_frame_, tf2::TimePointZero);

                // 计算位置误差
                double dx = object_pose.pose.position.x - current_tf.transform.translation.x;
                double dy = object_pose.pose.position.y - current_tf.transform.translation.y;
                double dz = object_pose.pose.position.z - current_tf.transform.translation.z;

                double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

                // 检查是否到达目标位置
                if (position_error < position_tolerance) {
                    RCLCPP_INFO(robot->node_->get_logger(), "视觉伺服完成，位置误差: %.4f m", position_error);
                    servo_completed = true;
                    break;
                }

                // 计算速度命令
                geometry_msgs::msg::Twist velocity;

                // 线速度：简单的比例控制
                double scale = std::min(1.0, max_linear_velocity / (position_error + 1e-6));
                velocity.linear.x = dx * scale;
                velocity.linear.y = dy * scale;
                velocity.linear.z = dz * scale;

                // 约束最大速度
                double linear_norm = std::sqrt(velocity.linear.x * velocity.linear.x +
                                              velocity.linear.y * velocity.linear.y +
                                              velocity.linear.z * velocity.linear.z);
                if (linear_norm > max_linear_velocity) {
                    velocity.linear.x = velocity.linear.x / linear_norm * max_linear_velocity;
                    velocity.linear.y = velocity.linear.y / linear_norm * max_linear_velocity;
                    velocity.linear.z = velocity.linear.z / linear_norm * max_linear_velocity;
                }

                // 角速度：暂时设置为零（可根据需要添加姿态控制）
                velocity.angular.x = 0.0;
                velocity.angular.y = 0.0;
                velocity.angular.z = 0.0;

                // 执行视觉伺服速度命令
                robot->execute_visual_servo(velocity);

                // 日志输出（每 10 次循环输出一次）
                static int loop_count = 0;
                if (loop_count++ % 10 == 0) {
                    RCLCPP_DEBUG(robot->node_->get_logger(),
                                "视觉伺服中... 位置误差: %.4f m, 速度: [%.4f, %.4f, %.4f]",
                                position_error, velocity.linear.x, velocity.linear.y, velocity.linear.z);
                }

                // 控制频率（10Hz）
                std::this_thread::sleep_for(100ms);

            } catch (const std::exception& e) {
                RCLCPP_ERROR(robot->node_->get_logger(), "获取当前位置 TF 失败: %s", e.what());
                return fail_task("获取当前位置失败");
            }
        }

        // 停止机械臂运动
        geometry_msgs::msg::Twist zero_velocity;
        zero_velocity.linear.x = 0.0;
        zero_velocity.linear.y = 0.0;
        zero_velocity.linear.z = 0.0;
        zero_velocity.angular.x = 0.0;
        zero_velocity.angular.y = 0.0;
        zero_velocity.angular.z = 0.0;
        robot->execute_visual_servo(zero_velocity);

        // 任务完成
        if (goal_handle) {
            robot->finish_current_task(goal_handle, true, "视觉伺服成功完成");
        }

        RCLCPP_INFO(robot->node_->get_logger(), "视觉伺服任务执行完成");
        return "idel";

    } catch (const std::exception& e) {
        RCLCPP_ERROR(robot->node_->get_logger(), "视觉伺服执行失败: %s", e.what());
        return fail_task(std::string("视觉伺服异常: ") + e.what());
    }
}
