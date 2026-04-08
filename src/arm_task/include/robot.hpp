#pragma once

#include "task/base_task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <map>


class Robot{
public:
    explicit Robot(rclcpp::Node::SharedPtr node);
    ~Robot();

    rclcpp::Node::SharedPtr node_;
    // TF2 for transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_target_pub_;


    //任务调度器:
    void porcess_task();
    void register_task(std::shared_ptr<BaseTask> task_ptr);
    void init_task_manager(const std::string first_task_name);
    std::shared_ptr<std::thread> task_thread_;
    std::mutex task_manager_mutex_;
    std::condition_variable task_manager_cv_;
    bool task_manager_initialized_{false};
    std::string current_task_name_;
    std::string last_task_name_;
    std::map<std::string, std::shared_ptr<BaseTask>> task_table_;
    
    //机械臂运动控制接口:

    //停止机械臂当前动作
    void stop_arm_motion();
    //执行关节空间轨迹
    bool execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration);
    //执行笛卡尔空间轨迹
    bool execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration);
    //执行视觉伺服控制
    void execute_visual_servo(const geometry_msgs::msg::Twist& velocity);
    //使能气泵
    bool set_air_pump(const bool &enable);
    //从yaml中得到命名位置的关节角度
    bool get_named_joint_position(const std::string& name, std::vector<double>& joint_angles) const;
    void load_named_joint_positions_from_yaml(const std::string& yaml_path);
    
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params);

    // Configuration
    std::string base_frame_{"base_link"};
    std::string camera_frame_{"camera_link"};
    std::string object_frame_{"target_object"};
    std::string tip_frame_{"link6"};
    std::string arm_calc_node_name_{"arm_calc_node"};
    double approach_distance_{0.1};  // meters above target
    double trajectory_duration_{4.0};  // seconds
    double grasp_time_{5.0};  // seconds
    double visual_servo_max_linear_acc_{0.1};
    
    // Joint positions from YAML
    std::map<std::string, std::vector<double>> arm_positions_;
    std::vector<double> ready_position_;  // Preparation position
    
    // Control flags
    std::atomic<bool> shutdown_requested_{false};
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    
    // Remote node clients for parameter setting
    rclcpp::AsyncParametersClient::SharedPtr arm_calc_param_client_;

private:
    void load_arm_positions_from_yaml();
};
