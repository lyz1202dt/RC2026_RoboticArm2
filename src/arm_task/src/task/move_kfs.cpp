#include "task/move_kfs.hpp"
#include "robot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <vector>

// data[0] = mode
//   mode=0 (关节空间): [0, j1, j2, j3, j4, j5, j6, duration, (pump)]              -> 8 or 9 values
//   mode=1 (笛卡尔空间-仅位置): [1, x, y, z, duration, (pump)]                       -> 5 or 6 values
//   mode=1 (笛卡尔空间-位置+姿态): [1, x, y, z, roll, pitch, yaw, duration, (pump)]  -> 8 or 9 values

MoveKFS::MoveKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

MoveKFS::~MoveKFS() {}

std::string MoveKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    const bool has_action_context = robot->get_active_task_context(context);
    const auto goal_handle = has_action_context ? context.goal_handle : nullptr;
    if (!has_action_context) {
        RCLCPP_WARN(robot->node_->get_logger(), "move_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    }

    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    if (context.data.empty()) {
        RCLCPP_ERROR(robot->node_->get_logger(), "接收到的移动任务数据为空");
        return fail_task("接收到的移动任务数据为空");
    }

    const int mode = static_cast<int>(context.data[0]);

    if (mode == 0) {
        // ---- 关节空间模式 ----
        // [0, j1, j2, j3, j4, j5, j6, duration, (pump)]
        const size_t n = context.data.size();
        if (n != 8 && n != 9) {
            RCLCPP_ERROR(robot->node_->get_logger(),
                "关节空间模式数据维度不正确，预期8或9，实际为%zu", n);
            return fail_task("关节空间模式数据维度不正确");
        }

        const double duration_ = context.data[7];
        const std::vector<double> joint_angles = {
            context.data[1], context.data[2], context.data[3],
            context.data[4], context.data[5], context.data[6],
        };

        if (n == 9) {
            int pump_enable = static_cast<int>(context.data[8]);
            RCLCPP_INFO(robot->node_->get_logger(), "关节空间移动任务包含气泵开关: %d", pump_enable);
            if (!robot->set_air_pump(pump_enable)) {
                RCLCPP_WARN(robot->node_->get_logger(), "设置气泵状态失败 (requested=%d)", pump_enable);
            }
        }

        RCLCPP_INFO(robot->node_->get_logger(), "执行关节空间移动任务");
        if (!robot->execute_joint_space_trajectory(joint_angles, duration_)) {
            return fail_task("执行关节空间移动轨迹失败");
        }

    } else if (mode == 1) {
        // ---- 笛卡尔空间模式 ----
        const size_t n = context.data.size();
        bool has_orientation = false;
        size_t duration_idx = 0;  // duration 所在的 index

        if (n == 5 || n == 6) {
            // [1, x, y, z, duration, (pump)]
            has_orientation = false;
            duration_idx = 4;
        } else if (n == 8 || n == 9) {
            // [1, x, y, z, roll, pitch, yaw, duration, (pump)]
            has_orientation = true;
            duration_idx = 7;
        } else {
            RCLCPP_ERROR(robot->node_->get_logger(),
                "笛卡尔空间模式数据维度不正确，预期5/6或8/9，实际为%zu", n);
            return fail_task("笛卡尔空间模式数据维度不正确");
        }

        const double duration_ = context.data[duration_idx];

        // 可选气泵开关
        if (n == 6 || n == 9) {
            int pump_enable = static_cast<int>(context.data[n - 1]);
            RCLCPP_INFO(robot->node_->get_logger(), "笛卡尔空间移动任务包含气泵开关: %d", pump_enable);
            if (!robot->set_air_pump(pump_enable)) {
                RCLCPP_WARN(robot->node_->get_logger(), "设置气泵状态失败 (requested=%d)", pump_enable);
            }
        }

        // 构建 PoseStamped
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.header.stamp = robot->node_->now();

        target_pose.pose.position.x = context.data[1];
        target_pose.pose.position.y = context.data[2];
        target_pose.pose.position.z = context.data[3];

        if (has_orientation) {
            // 从欧拉角(RPY)转换为四元数
            double roll  = context.data[4];
            double pitch = context.data[5];
            double yaw   = context.data[6];
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            target_pose.pose.orientation.x = q.x();
            target_pose.pose.orientation.y = q.y();
            target_pose.pose.orientation.z = q.z();
            target_pose.pose.orientation.w = q.w();
            RCLCPP_INFO(robot->node_->get_logger(),
                "执行笛卡尔空间移动任务: pos=(%.3f, %.3f, %.3f), rpy=(roll=%.3f, pitch=%.3f, yaw=%.3f), duration=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                roll, pitch, yaw, duration_);
        } else {
            // 未传入姿态，获取当前末端姿态作为默认值
            geometry_msgs::msg::PoseStamped current_pose;
            if (!robot->get_current_end_pose_from_arm_calc(current_pose)) {
                RCLCPP_WARN(robot->node_->get_logger(), "获取当前末端位姿失败，姿态将使用单位四元数");
                target_pose.pose.orientation.w = 1.0;
            } else {
                target_pose.pose.orientation = current_pose.pose.orientation;
            }
            RCLCPP_INFO(robot->node_->get_logger(),
                "执行笛卡尔空间移动任务(使用当前姿态): pos=(%.3f, %.3f, %.3f), ori=(%.3f, %.3f, %.3f, %.3f), duration=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
                target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                target_pose.pose.orientation.z, target_pose.pose.orientation.w,
                duration_);
        }

        if (!robot->execute_cartesian_space_trajectory(target_pose, duration_)) {
            return fail_task("执行笛卡尔空间移动轨迹失败");
        }

    } else {
        RCLCPP_ERROR(robot->node_->get_logger(),
            "未知的移动模式: %d (0=关节空间, 1=笛卡尔空间)", mode);
        return fail_task("未知的移动模式");
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动流程完成");
    if (goal_handle) {
        robot->finish_current_task(goal_handle, true, "移动流程执行完成");
    }

    return "idel";
}