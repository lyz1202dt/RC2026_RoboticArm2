#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/arm_cmd.hpp>
#include <robot_interfaces/msg/vis.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace arm_task {

class ArmTaskNode : public rclcpp::Node {
public:
    explicit ArmTaskNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmTaskNode() override;

private:
    enum class ArmSide : int32_t {
        kLeft = 0,
        kRight = 1,
    };

    void load_arm_positions_from_yaml();
    void task_execution_thread();
    void execute_task_state_machine(int32_t task_mode, int preset_position_id);

    void execute_grasp_flow(ArmSide side);
    void execute_place_flow(ArmSide side);
    void execute_move_to_position(int preset_position_id);
    void execute_fixed_release_flow();

    void execute_joint_space_trajectory(ArmSide side, const std::vector<double>& joint_angles);
    void execute_cartesian_space_trajectory(ArmSide side, const geometry_msgs::msg::PoseStamped& target_pose);
    void stop_arm_motion(ArmSide side);
    void publish_arm_command(ArmSide side, int32_t motion_mode, const std::vector<double>& position, double duration_sec);

    bool get_latest_object_pose(geometry_msgs::msg::PoseStamped& pose_out);
    geometry_msgs::msg::PoseStamped create_approach_pose(const geometry_msgs::msg::PoseStamped& target_pose, double distance) const;
    void set_air_pump_enabled(bool enabled);
    void reset_task_parameter();
    void mark_task_finished();

    void vision_callback(const robot_interfaces::msg::Vis& msg);
    void on_place_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params);

    static bool is_supported_task_mode(int32_t task_mode);
    static const char* side_name(ArmSide side);

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<robot_interfaces::msg::ArmCmd>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<robot_interfaces::msg::Vis>::SharedPtr vision_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_target_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::thread task_thread_;
    bool shutdown_requested_{false};
    bool task_running_{false};
    int32_t requested_task_mode_{0};
    int preset_position_id_{0};
    std::atomic<int32_t> active_task_mode_{0};

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped latest_visual_pose_;
    geometry_msgs::msg::PoseStamped place_target_pose_;
    bool has_visual_pose_{false};
    bool has_place_target_{false};

    std::map<int, std::vector<double>> arm_positions_;
    std::vector<double> ready_position_{0.0, 2.4, 1.3, 1.0};
    std::vector<double> home_position_{0.0, 0.0, 0.0, 0.0};
    std::vector<double> grasp_position_{0.0, 3.14159, 2.45, 2.48};
    std::vector<double> grasp_position_two_{0.0, 3.14159, 2.4, 2.55};
    std::vector<double> place_position_{0.0, 3.14159, 3.1, 3.1};
};

} // namespace arm_task
