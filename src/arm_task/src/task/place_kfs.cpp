#include "task/place_kfs.hpp"
#include "robot.hpp"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

PlaceKFS::PlaceKFS(Robot* context, const std::string name)
    : BaseTask(context, name) {
}

PlaceKFS::~PlaceKFS() {}

std::string PlaceKFS::process(const std::string last_task_name) {
    (void)last_task_name;

    Robot::ActiveTaskContext context;
    if (!robot->get_active_task_context(context)) {
        RCLCPP_WARN(robot->node_->get_logger(), "place_kfs 未获取到活动任务上下文，返回 idel");
        return "idel";
    };

    const auto goal_handle = context.goal_handle;

    // 1. 移动到 "kfs" + std::to_string(robot->current_kfs_num_) + "_touch_pos"
    std::string touch_pos_name = "kfs" + std::to_string(robot->current_kfs_num_) + "_touch_pos";
    std::vector<double> touch_pos;
    if (!robot->get_named_joint_position(touch_pos_name, touch_pos)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", touch_pos_name.c_str());
        robot->finish_current_task(goal_handle, false, "未找到命名位姿 " + touch_pos_name);
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到触摸位置 [%s]", touch_pos_name.c_str());
    if (!robot->execute_joint_space_trajectory(touch_pos, 1.0)) {
        robot->finish_current_task(goal_handle, false, "移动到触摸位置失败");
        return "idel";
    }

    // 2. 等待 1s
    std::this_thread::sleep_for(1s);

    // 3. 打开气泵
    RCLCPP_INFO(robot->node_->get_logger(), "打开气泵");
    if (!robot->set_air_pump(true)) {
        robot->finish_current_task(goal_handle, false, "气泵开启失败");
        return "idel";
    }

    // 4. 等待 1s
    std::this_thread::sleep_for(1s);

    // 5. 移动到 target_shelf_ 的前方（x 方向减0.40m）
    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = robot->node_->now();
    approach_pose.pose = robot->target_shelf_;
    approach_pose.pose.position.x -= 0.40;

    RCLCPP_INFO(robot->node_->get_logger(), "移动到货架前方位置");
    if (!robot->execute_cartesian_space_trajectory(approach_pose, 1.0)) {
        robot->finish_current_task(goal_handle, false, "移动到货架前方失败");
        return "idel";
    }

    // 6. 等待 1s
    std::this_thread::sleep_for(1s);

    // 7. 沿直线轨迹移动到 target_shelf_
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = robot->node_->now();
    target_pose.pose = robot->target_shelf_;

    RCLCPP_INFO(robot->node_->get_logger(), "沿直线轨迹移动到货架");
    if (!robot->execute_cartesian_space_trajectory(target_pose, 1.0)) {
        robot->finish_current_task(goal_handle, false, "沿直线轨迹移动到货架失败");
        return "idel";
    }

    // 8. 等待 1s
    std::this_thread::sleep_for(1s);

    // 9. 关闭气泵
    RCLCPP_INFO(robot->node_->get_logger(), "关闭气泵");
    if (!robot->set_air_pump(false)) {
        robot->finish_current_task(goal_handle, false, "气泵关闭失败");
        return "idel";
    }

    RCLCPP_INFO(robot->node_->get_logger(), "放置流程完成");
    robot->finish_current_task(goal_handle, true, "放置流程执行完成");

    return "idel";
}