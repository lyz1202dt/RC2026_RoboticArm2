#include "dog_controller/dog_controller.hpp"

#include <algorithm>
#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace dog_controller {

namespace {

constexpr char kStateTopic[] = "myjoints_state";
constexpr char kTargetTopic[] = "myjoints_target";
constexpr double kTargetLogPeriodSec = 0.1;

}  // namespace

DogController::DogController() = default;

controller_interface::CallbackReturn DogController::on_init() {
    auto node = get_node();

    state_publisher_ = node->create_publisher<robot_interfaces::msg::Arm>(kStateTopic, rclcpp::SensorDataQoS());
    target_subscriber_ = node->create_subscription<robot_interfaces::msg::Arm>(
        kTargetTopic, rclcpp::SensorDataQoS(),
        [this](const robot_interfaces::msg::Arm& msg) { joints_target_ = msg; });

    joints_name_ = {"joint1", "joint2", "joint3", "joint4"};
    joint_kp_ = std::vector<double>(kJointCount, 5.0);
    joint_ki_ = std::vector<double>(kJointCount, 0.0);
    joint_kd_ = std::vector<double>(kJointCount, 0.0);
    joint_integral_error_ = std::vector<double>(kJointCount, 0.0);

    for (std::size_t i = 0; i < kJointCount; ++i) {
        const std::string kp_name = "joint" + std::to_string(i + 1) + "_kp";
        const std::string ki_name = "joint" + std::to_string(i + 1) + "_ki";
        const std::string kd_name = "joint" + std::to_string(i + 1) + "_kd";
        if (!node->has_parameter(kp_name)) {
            node->declare_parameter(kp_name, 0.0);
        }
        if (!node->has_parameter(ki_name)) {
            node->declare_parameter(ki_name, 0.0);
        }
        if (!node->has_parameter(kd_name)) {
            node->declare_parameter(kd_name, 0.0);
        }
    }
    if (!node->has_parameter("joint_torque_filter_gate")) {
        node->declare_parameter("joint_torque_filter_gate", 0.8);
    }
    if (!node->has_parameter("joint_omega_filter_gate")) {
        node->declare_parameter("joint_omega_filter_gate", 0.8);
    }
    if (!node->has_parameter("command_effort_limit")) {
        node->declare_parameter("command_effort_limit", 20.0);
    }
    if (!node->has_parameter("integral_error_limit")) {
        node->declare_parameter("integral_error_limit", 0.5);
    }

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            bool handled_gain = false;
            for (std::size_t i = 0; i < kJointCount; ++i) {
                if (param.get_name() == "joint" + std::to_string(i + 1) + "_kp") {
                    joint_kp_[i] = param.as_double();
                    handled_gain = true;
                    break;
                }
                if (param.get_name() == "joint" + std::to_string(i + 1) + "_ki") {
                    joint_ki_[i] = param.as_double();
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
            } else if (param.get_name() == "joint_torque_filter_gate") {
                joint_torque_filter_gate_ = param.as_double();
            } else if (param.get_name() == "joint_omega_filter_gate") {
                joint_omega_filter_gate_ = param.as_double();
            } else if (param.get_name() == "command_effort_limit") {
                command_effort_limit_ = std::max(param.as_double(), 0.0);
            } else if (param.get_name() == "integral_error_limit") {
                integral_error_limit_ = std::max(param.as_double(), 0.0);
            }
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    auto node = get_node();
    for (std::size_t i = 0; i < kJointCount; ++i) {
        joint_kp_[i] = node->get_parameter("joint" + std::to_string(i + 1) + "_kp").as_double();
        joint_ki_[i] = node->get_parameter("joint" + std::to_string(i + 1) + "_ki").as_double();
        joint_kd_[i] = node->get_parameter("joint" + std::to_string(i + 1) + "_kd").as_double();
        RCLCPP_INFO(node->get_logger(), "Got gains for joint %zu: kp: %f, ki: %f, kd: %f", i, joint_kp_[i], joint_ki_[i], joint_kd_[i]);
    }
    joint_torque_filter_gate_ = node->get_parameter("joint_torque_filter_gate").as_double();
    joint_omega_filter_gate_ = node->get_parameter("joint_omega_filter_gate").as_double();
    command_effort_limit_ = std::max(node->get_parameter("command_effort_limit").as_double(), 0.0);
    integral_error_limit_ = std::max(node->get_parameter("integral_error_limit").as_double(), 0.0);
    std::fill(joint_integral_error_.begin(), joint_integral_error_.end(), 0.0);
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    last_target_log_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
    std::fill(joint_integral_error_.begin(), joint_integral_error_.end(), 0.0);
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DogController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    std::fill(joint_integral_error_.begin(), joint_integral_error_.end(), 0.0);
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DogController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;

    for (std::size_t i = 0; i < kJointCount; ++i) {
        joints_state_.motor[i].rad = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
        joints_state_.motor[i].omega = static_cast<float>(
            joint_omega_filter_gate_ * joints_state_.motor[i].omega +
            (1.0 - joint_omega_filter_gate_) * state_interfaces_[i * 3 + 1].get_value());
        joints_state_.motor[i].torque = static_cast<float>(
            joint_torque_filter_gate_ * joints_state_.motor[i].torque +
            (1.0 - joint_torque_filter_gate_) * state_interfaces_[i * 3 + 2].get_value());
    }

    state_publisher_->publish(joints_state_);

    if (last_target_log_time_.nanoseconds() == 0 ||
        (time - last_target_log_time_).seconds() >= kTargetLogPeriodSec) {
        // RCLCPP_INFO(
        //     get_node()->get_logger(),
        //     "Joint targets joint1=%.6f joint2=%.6f joint3=%.6f joint4=%.6f",
        //     static_cast<double>(joints_target_.motor[0].rad),
        //     static_cast<double>(joints_target_.motor[1].rad),
        //     static_cast<double>(joints_target_.motor[2].rad),
        //     static_cast<double>(joints_target_.motor[3].rad));
        last_target_log_time_ = time;
    }

    const double dt = std::max(period.seconds(), 1e-6);
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const double position_error =
            static_cast<double>(joints_target_.motor[i].rad) - static_cast<double>(joints_state_.motor[i].rad);
        const double velocity_error =
            static_cast<double>(joints_target_.motor[i].omega) - static_cast<double>(joints_state_.motor[i].omega);

        joint_integral_error_[i] += position_error * dt;
        joint_integral_error_[i] = std::clamp(joint_integral_error_[i], -integral_error_limit_, integral_error_limit_);

        double effort = joint_kp_[i] * position_error +
                        joint_ki_[i] * joint_integral_error_[i] +
                        joint_kd_[i] * velocity_error +
                        static_cast<double>(joints_target_.motor[i].torque);

        if (std::abs(effort) > command_effort_limit_) {
            joint_integral_error_[i] -= position_error * dt;
            effort = joint_kp_[i] * position_error +
                     joint_ki_[i] * joint_integral_error_[i] +
                     joint_kd_[i] * velocity_error +
                     static_cast<double>(joints_target_.motor[i].torque);
        }

        effort = std::clamp(effort, -command_effort_limit_, command_effort_limit_);
        command_interfaces_[i].set_value(effort);
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration DogController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration DogController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

}  // namespace dog_controller

PLUGINLIB_EXPORT_CLASS(dog_controller::DogController, controller_interface::ControllerInterface)
