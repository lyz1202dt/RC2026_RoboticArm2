#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/arm.hpp>

#include <thread>

class ParameterMeasure
{
public:
  ParameterMeasure(const rclcpp::Node::SharedPtr node);
  ~ParameterMeasure();

private:
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params);
    void jointStateCallback(const robot_interfaces::msg::Arm::SharedPtr msg);
    void start_measure_thread();
    void measure_thread_func();
    std::string build_csv_file_path() const;
    std::vector<float> current_joint_position() const;
    void publish_joint_target(const std::vector<float>& joint_pos) const;
    bool snapshot_joint_state(std::vector<float>& joint_pos, std::vector<float>& joint_vel,
                              std::vector<float>& joint_torque);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr joint_target_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    mutable std::mutex state_mutex_;
    std::condition_variable state_cv_;
    bool has_joint_state_{false};
    std::vector<float> latest_joint_pos_;
    std::vector<float> latest_joint_vel_;
    std::vector<float> latest_joint_torque_;

    std::mutex thread_mutex_;
    std::shared_ptr<std::thread> measure_deal_thread;
    std::atomic_bool measure_running_{false};
    std::atomic_bool exit_requested_{false};
    std::atomic_bool resetting_start_measure_{false};

    std::string model_path_;
    std::string joint_state_topic_;
    std::string joint_target_topic_;
    std::string csv_file_path_;
    std::string trajectory_file_path_;
    int joint_dof_{6};
    double move_to_start_duration_sec_{3.0};
    double control_period_sec_{0.02};
    int discard_initial_samples_{5};
};
