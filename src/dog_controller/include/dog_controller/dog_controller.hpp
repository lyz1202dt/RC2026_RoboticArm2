#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace dog_controller {

class DogController : public controller_interface::ControllerInterface {
public:
    DogController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    void on_target_joint_state(const sensor_msgs::msg::JointState& msg);
    bool configure_joints();
    void configure_gains();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::mutex target_mutex_;
    bool has_target_{false};
    std::string target_topic_{"joint_states"};
    std::vector<std::string> joints_name_;
    std::vector<double> joint_kp_;
    std::vector<double> joint_kd_;
    std::vector<double> target_position_;
    std::vector<double> target_velocity_;
    std::vector<double> target_effort_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    double joint_torque_filter_gate_{0.8};
    double joint_omega_filter_gate_{0.8};
    double command_effort_limit_{80.0};
};

}  // namespace dog_controller
