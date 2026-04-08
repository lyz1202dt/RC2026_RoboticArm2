#pragma once

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

namespace arm_task {

class ArmTaskNode : public rclcpp::Node {
public:
    explicit ArmTaskNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmTaskNode();

private:
    //任务执行线程
    void task_execution_thread();
    
    // State machine for task execution
    void execute_task_state_machine();
    void execute_grasp_flow();
    void execute_place_flow();
    void execute_move_to_position(int position_index);
    
    //机械臂运动控制接口

    //停止机械臂的任何动作
    void stop_arm_motion();
    //执行关节空间轨迹
    void execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration);
    //执行笛卡尔空间轨迹
    void execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration);
    //执行视觉伺服控制
    void execute_visual_servo(const geometry_msgs::msg::PoseStamped& target_pose);
    //等待视觉伺服收敛
    bool wait_for_visual_servo_convergence(double position_tolerance_m, double timeout_sec);
    //等待轨迹执行完成
    bool wait_for_trajectory_completion(double timeout_sec);
    

    //辅助方法
    bool get_object_pose_in_base_frame(geometry_msgs::msg::PoseStamped& pose_out);
    void set_parameter_on_remote_node(const std::string& node_name, 
                                       const rclcpp::Parameter& param);
    void load_arm_positions_from_yaml();
    geometry_msgs::msg::PoseStamped create_approach_pose(
        const geometry_msgs::msg::PoseStamped& target_pose, double distance);
    
    // Callbacks
    void on_place_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params);
    
    // Visual servo publishing thread
    void visual_servo_publish_thread();
    
    // TF2 for transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_target_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_target_sub_;
    
    // Parameters
    std::atomic<int32_t> arm_task_mode_{0};  // 0: standby, 1: grasp, 2: place, 1x: move to position x
    std::atomic<bool> task_running_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> visual_servo_active_{false};
    
    // Thread safety
    std::mutex task_mutex_;
    std::mutex pose_mutex_;
    std::mutex visual_servo_state_mutex_;
    std::condition_variable visual_servo_state_cv_;
    std::thread task_thread_;
    std::thread visual_servo_thread_;

    bool visual_servo_result_ready_{false};
    bool visual_servo_succeeded_{false};
    
    // Task data
    geometry_msgs::msg::PoseStamped target_object_pose_;
    geometry_msgs::msg::PoseStamped place_target_pose_;
    bool has_object_pose_{false};
    bool has_place_target_{false};
    
    // Configuration
    std::string base_frame_{"base_link"};
    std::string camera_frame_{"camera_link"};
    std::string object_frame_{"target_object"};
    std::string tip_frame_{"link6"};
    std::string arm_calc_node_name_{"arm_calc_node"};
    double approach_distance_{0.1};  // meters above target
    double trajectory_duration_{4.0};  // seconds
    double grasp_time_{5.0};  // seconds
    double visual_servo_kp_{0.1};
    double visual_servo_max_linear_acc_{0.1};
    int air_pump_pin_{0};  // Parameter service index for air pump control
    
    // Joint positions from YAML
    std::map<int, std::vector<double>> arm_positions_;
    std::vector<double> ready_position_;  // Preparation position
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    
    // Remote node clients for parameter setting
    rclcpp::AsyncParametersClient::SharedPtr arm_calc_param_client_;
};

}  // namespace arm_task
