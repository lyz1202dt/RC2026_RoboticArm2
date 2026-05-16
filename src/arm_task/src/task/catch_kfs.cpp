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

    // Helper lambda to safely terminate the current task on failure
    auto fail_task = [&](const std::string& error_msg) {
        if (goal_handle) {
            robot->finish_current_task(goal_handle, false, error_msg);
        }
        return "idel";
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 获取抓取高度：优先使用 action 数据，否则使用 ROS 参数
    // action 发送的是索引 (0.0/1.0/2.0)，需要转换为实际高度值 (0.02/0.41/0.68)




    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    std::vector<double> ready_joint_angles;
    std::string ready_position_name = "ready";
    ready_position_name = "ready";







    

    if (!robot->set_air_pump(true)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "气泵开启失败");
        return "idel";
    }
    std::this_thread::sleep_for(1s);

     if (!robot->get_named_joint_position(ready_position_name, ready_joint_angles)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "未找到命名位姿 [%s]", ready_position_name.c_str());
        return fail_task("未找到命名位姿 " + ready_position_name);
    }

    RCLCPP_INFO(robot->node_->get_logger(), "移动到准备位置");
    if (!robot->execute_joint_space_trajectory(ready_joint_angles, 3.0)) { // 1.0
        return fail_task("抓取前移动到准备位失败");
    }

    std::this_thread::sleep_for(1s);


    if (!robot->set_air_pump(false)) {
        RCLCPP_ERROR(robot->node_->get_logger(), "气泵关闭失败");
        return "idel";
    }
    std::this_thread::sleep_for(10s);













    // 2. 获取目标位姿：优先使用 action 数据，否则使用 TF
    RCLCPP_INFO(robot->node_->get_logger(), "准备获取抓取目标位姿");
    geometry_msgs::msg::PoseStamped object_pose;

    double grasp_right_run_ = 0.0;
    double grasp_right_run_qian_ = 0.0;
    double grasp_duration_ = 0.8;
    double grasp_height = 0.0;
    if (has_action_context) {

        const geometry_msgs::msg::TransformStamped target_tf =
        robot->tf_buffer_->lookupTransform("base_link", robot->object_frame_, tf2::TimePointZero);
        object_pose.header.frame_id = "base_link";
        object_pose.header.stamp = robot->node_->now();
        object_pose.pose.position.x = target_tf.transform.translation.x;
        object_pose.pose.position.y = -(target_tf.transform.translation.y+0.05);
        object_pose.pose.position.z = target_tf.transform.translation.z; // grasp_height;q

        RCLCPP_INFO(robot->node_->get_logger(), "=====================================");
        RCLCPP_INFO(robot->node_->get_logger(), "position.x = %lf", object_pose.pose.position.x);
        RCLCPP_INFO(robot->node_->get_logger(), "position.y = %lf", object_pose.pose.position.y);
        RCLCPP_INFO(robot->node_->get_logger(), "position.z = %lf", object_pose.pose.position.z);
        RCLCPP_INFO(robot->node_->get_logger(), "orientation.x = %lf", object_pose.pose.orientation.x);
        RCLCPP_INFO(robot->node_->get_logger(), "orientation.y = %lf", object_pose.pose.orientation.y);
        RCLCPP_INFO(robot->node_->get_logger(), "orientation.z = %lf", object_pose.pose.orientation.z);
        RCLCPP_INFO(robot->node_->get_logger(), "orientation.w = %lf", object_pose.pose.orientation.w);
        RCLCPP_INFO(robot->node_->get_logger(), "=====================================");

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
            object_pose.pose.position.y = -(target_tf.transform.translation.y+0.05);
            object_pose.pose.position.z = target_tf.transform.translation.z; // grasp_height;q


            RCLCPP_INFO(robot->node_->get_logger(), "=====================================");
            RCLCPP_INFO(robot->node_->get_logger(), "position.x = %lf", object_pose.pose.position.x);
            RCLCPP_INFO(robot->node_->get_logger(), "position.y = %lf", object_pose.pose.position.y);
            RCLCPP_INFO(robot->node_->get_logger(), "position.z = %lf", object_pose.pose.position.z);
            RCLCPP_INFO(robot->node_->get_logger(), "orientation.x = %lf", object_pose.pose.orientation.x);
            RCLCPP_INFO(robot->node_->get_logger(), "orientation.y = %lf", object_pose.pose.orientation.y);
            RCLCPP_INFO(robot->node_->get_logger(), "orientation.z = %lf", object_pose.pose.orientation.z);
            RCLCPP_INFO(robot->node_->get_logger(), "orientation.w = %lf", object_pose.pose.orientation.w);
            RCLCPP_INFO(robot->node_->get_logger(), "=====================================");

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
    quat.setRPY(0.0, M_PI/2.0, 0.0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();
    object_pose.pose.position.x -= grasp_right_run_;


    // // std::this_thread::sleep_for(100ms);

    // RCLCPP_INFO(robot->node_->get_logger(), "执行抓取动作");
    // if (!robot->execute_cartesian_space_trajectory(object_pose, 3.0)) { // 0.8
    //     return fail_task("执行抓取轨迹失败");
    // }

    // TODO: 使用视觉伺服移动到目标位置





    if (!robot->set_air_pump(true)) {
        return fail_task("气泵开启失败");
    }
    std::this_thread::sleep_for(3s);



    robot->current_kfs_num_ += 1;

    


    // // 直线后退0.3m
    // if (grasp_height_ == 0.0 ||
    //     grasp_height_ == 1.0){

    //     quat.setRPY(0, (M_PI / 1.8), M_PI); 
    //     object_pose.pose.orientation.w = quat.getW();
    //     object_pose.pose.orientation.x = quat.getX();
    //     object_pose.pose.orientation.y = quat.getY();
    //     object_pose.pose.orientation.z = quat.getZ();
    //     object_pose.pose.position.z+=0.07;
    //     if (!robot->execute_cartesian_space_trajectory(object_pose, 2.0)) { // 0.8
    //         return fail_task("后退失败");
    //     }


    //     object_pose.pose.position.x-=0.3;

    //     if (!robot->execute_cartesian_space_trajectory(object_pose, 2.0)) { // 0.6
    //         return fail_task("后退失败");
    //     }
    // }
    
    std::string detach_pos_name;
    std::vector<double> detach_pos;
    // 3. 移动到 kfs_detach
        detach_pos_name = "kfs_detach";

    RCLCPP_INFO(robot->node_->get_logger(), "释放位置，400");
    if (!robot->execute_joint_space_trajectory(detach_pos, 3.0)) {
        return fail_task("移动到释放位置失败");
    }

    // 4. 等待 1s
    std::this_thread::sleep_for(1s);

    // // 5. 关闭气泵
    // RCLCPP_INFO(robot->node_->get_logger(), "关闭气泵");
    // if (!robot->set_air_pump(false)) {
    //     robot->finish_current_task(goal_handle, false, "气泵关闭失败");
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

