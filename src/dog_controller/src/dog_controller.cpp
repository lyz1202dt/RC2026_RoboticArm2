#include "dog_controller/dog_controller.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace dog_controller {

namespace {

constexpr char kTargetTopicParameter[] = "target_topic";
constexpr char kJointsParameter[] = "joints";
constexpr char kJointTorqueFilterGateParameter[] = "joint_torque_filter_gate";
constexpr char kJointOmegaFilterGateParameter[] = "joint_omega_filter_gate";
constexpr char kCommandEffortLimitParameter[] = "command_effort_limit";
constexpr std::array<const char*, 7> kDefaultJointNames{
    "yuntai",
    "left1",
    "left2",
    "left3",
    "right1",
    "right2",
    "right3",
};

std::vector<std::string> default_joint_names()
{
    return {kDefaultJointNames.begin(), kDefaultJointNames.end()};
}

bool get_named_value(
    const sensor_msgs::msg::JointState& msg,
    const std::vector<double>& values,
    const std::string& joint_name,
    double& value)
{
    const auto iter = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (iter == msg.name.end()) {
        return false;
    }

    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), iter));
    if (index >= values.size()) {
        return false;
    }

    value = values[index];
    return true;
}

bool get_joint_value(
    const sensor_msgs::msg::JointState& msg,
    const std::vector<double>& values,
    const std::string& joint_name,
    std::size_t fallback_index,
    double& value)
{
    if (get_named_value(msg, values, joint_name, value)) {
        return true;
    }

    if (fallback_index < values.size()) {
        value = values[fallback_index];
        return true;
    }

    return false;
}

}  // namespace

DogController::DogController() = default;

controller_interface::CallbackReturn DogController::on_init()
{
    auto node = get_node();

    joints_name_ = default_joint_names();
    joint_kp_ = std::vector<double>(joints_name_.size(), 50.0);
    joint_kd_ = std::vector<double>(joints_name_.size(), 3.0);

    if (!node->has_parameter(kTargetTopicParameter)) {
        node->declare_parameter<std::string>(kTargetTopicParameter, "joint_states");
    }
    if (!node->has_parameter(kJointsParameter)) {
        node->declare_parameter<std::vector<std::string>>(kJointsParameter, joints_name_);
    }

    for (std::size_t i = 0; i < joints_name_.size(); ++i) {
        const std::string kp_name = "joint" + std::to_string(i + 1) + "_kp";
        const std::string kd_name = "joint" + std::to_string(i + 1) + "_kd";
        if (!node->has_parameter(kp_name)) {
            node->declare_parameter(kp_name, joint_kp_[i]);
        }
        if (!node->has_parameter(kd_name)) {
            node->declare_parameter(kd_name, joint_kd_[i]);
        }
    }
    if (!node->has_parameter(kJointTorqueFilterGateParameter)) {
        node->declare_parameter(kJointTorqueFilterGateParameter, joint_torque_filter_gate_);
    }
    if (!node->has_parameter(kJointOmegaFilterGateParameter)) {
        node->declare_parameter(kJointOmegaFilterGateParameter, joint_omega_filter_gate_);
    }
    if (!node->has_parameter(kCommandEffortLimitParameter)) {
        node->declare_parameter(kCommandEffortLimitParameter, command_effort_limit_);
    }

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            bool handled_gain = false;
            for (std::size_t i = 0; i < joint_kp_.size(); ++i) {
                if (param.get_name() == "joint" + std::to_string(i + 1) + "_kp") {
                    joint_kp_[i] = param.as_double();
                    handled_gain = true;
                    break;
                }
                if (param.get_name() == "joint" + std::to_string(i + 1) + "_kd") {
                    joint_kd_[i] = param.as_double();
                    handled_gain = true;
                    break;
                }
            }

            if (handled_gain) {
                continue;
            }
            if (param.get_name() == kJointTorqueFilterGateParameter) {
                joint_torque_filter_gate_ = param.as_double();
            } else if (param.get_name() == kJointOmegaFilterGateParameter) {
                joint_omega_filter_gate_ = param.as_double();
            } else if (param.get_name() == kCommandEffortLimitParameter) {
                command_effort_limit_ = std::max(param.as_double(), 0.0);
            }
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;

    if (!configure_joints()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    configure_gains();

    auto node = get_node();
    target_topic_ = node->get_parameter(kTargetTopicParameter).as_string();
    target_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
        target_topic_, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState& msg) { on_target_joint_state(msg); });

    RCLCPP_INFO(
        node->get_logger(), "dog_controller target interface: sensor_msgs/JointState on %s",
        target_topic_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DogController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    (void)time;
    (void)period;

    const auto joint_count = joints_name_.size();
    for (std::size_t i = 0; i < joint_count; ++i) {
        joint_position_[i] = state_interfaces_[i * 3 + 0].get_value();
        joint_velocity_[i] =
            joint_omega_filter_gate_ * joint_velocity_[i] +
            (1.0 - joint_omega_filter_gate_) * state_interfaces_[i * 3 + 1].get_value();
        joint_effort_[i] =
            joint_torque_filter_gate_ * joint_effort_[i] +
            (1.0 - joint_torque_filter_gate_) * state_interfaces_[i * 3 + 2].get_value();
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    for (std::size_t i = 0; i < joint_count; ++i) {
        const double desired_position = has_target_ ? target_position_[i] : joint_position_[i];
        const double desired_velocity = has_target_ ? target_velocity_[i] : 0.0;
        const double desired_effort = has_target_ ? target_effort_[i] : 0.0;

        double effort =
            joint_kp_[i] * (desired_position - joint_position_[i]) +
            joint_kd_[i] * (desired_velocity - joint_velocity_[i]) +
            desired_effort;
        effort = std::clamp(effort, -command_effort_limit_, command_effort_limit_);
        command_interfaces_[i].set_value(effort);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration DogController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration DogController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

void DogController::on_target_joint_state(const sensor_msgs::msg::JointState& msg)
{
    const auto joint_count = joints_name_.size();
    std::vector<double> position(joint_count, 0.0);
    std::vector<double> velocity(joint_count, 0.0);
    std::vector<double> effort(joint_count, 0.0);

    for (std::size_t i = 0; i < joint_count; ++i) {
        if (!get_joint_value(msg, msg.position, joints_name_[i], i, position[i])) {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "JointState target missing position for joint %s", joints_name_[i].c_str());
            return;
        }

        double optional_value = 0.0;
        if (get_joint_value(msg, msg.velocity, joints_name_[i], i, optional_value)) {
            velocity[i] = optional_value;
        }
        if (get_joint_value(msg, msg.effort, joints_name_[i], i, optional_value)) {
            effort[i] = optional_value;
        }
    }

    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_position_ = std::move(position);
        target_velocity_ = std::move(velocity);
        target_effort_ = std::move(effort);
        has_target_ = true;
    }
}

bool DogController::configure_joints()
{
    auto node = get_node();
    joints_name_ = node->get_parameter(kJointsParameter).as_string_array();
    if (joints_name_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "dog_controller requires at least one joint");
        return false;
    }

    const auto joint_count = joints_name_.size();
    joint_kp_.assign(joint_count, 50.0);
    joint_kd_.assign(joint_count, 3.0);
    target_position_.assign(joint_count, 0.0);
    target_velocity_.assign(joint_count, 0.0);
    target_effort_.assign(joint_count, 0.0);
    joint_position_.assign(joint_count, 0.0);
    joint_velocity_.assign(joint_count, 0.0);
    joint_effort_.assign(joint_count, 0.0);
    has_target_ = false;
    return true;
}

void DogController::configure_gains()
{
    auto node = get_node();
    for (std::size_t i = 0; i < joints_name_.size(); ++i) {
        const std::string kp_name = "joint" + std::to_string(i + 1) + "_kp";
        const std::string kd_name = "joint" + std::to_string(i + 1) + "_kd";
        if (!node->has_parameter(kp_name)) {
            node->declare_parameter(kp_name, joint_kp_[i]);
        }
        if (!node->has_parameter(kd_name)) {
            node->declare_parameter(kd_name, joint_kd_[i]);
        }
        joint_kp_[i] = node->get_parameter(kp_name).as_double();
        joint_kd_[i] = node->get_parameter(kd_name).as_double();
    }

    joint_torque_filter_gate_ = node->get_parameter(kJointTorqueFilterGateParameter).as_double();
    joint_omega_filter_gate_ = node->get_parameter(kJointOmegaFilterGateParameter).as_double();
    command_effort_limit_ = std::max(node->get_parameter(kCommandEffortLimitParameter).as_double(), 0.0);
}

}  // namespace dog_controller

PLUGINLIB_EXPORT_CLASS(dog_controller::DogController, controller_interface::ControllerInterface)
