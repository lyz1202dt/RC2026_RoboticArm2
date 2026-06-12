#include "serialnode.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace {

constexpr uint16_t kUsbVid = 0x0483;
constexpr uint16_t kUsbPid = 0x5740;
constexpr int kTargetPackType = 0x01;
constexpr char kJointStateTopic[] = "joint_states";
constexpr char kAirPumpParameter[] = "enable_air_pump";
constexpr std::array<const char*, 7> kJointNames{
    "yuntai",
    "left1",
    "left2",
    "left3",
    "right1",
    "right2",
    "right3",
};

bool getPositionByName(const sensor_msgs::msg::JointState& msg, const char* name, double& position)
{
    const auto iter = std::find(msg.name.begin(), msg.name.end(), name);
    if (iter == msg.name.end()) {
        return false;
    }

    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), iter));
    if (index >= msg.position.size()) {
        return false;
    }

    position = msg.position[index];
    return true;
}

bool getJointPosition(
    const sensor_msgs::msg::JointState& msg,
    const char* name,
    std::size_t fallback_index,
    double& position)
{
    if (getPositionByName(msg, name, position)) {
        return true;
    }

    if (fallback_index < msg.position.size()) {
        position = msg.position[fallback_index];
        return true;
    }

    return false;
}

} // namespace

ArmNode::ArmNode()
    : Node("arm_node")
{
    declare_parameter<bool>(kAirPumpParameter, false);
    get_parameter(kAirPumpParameter, enable_air_pump);
    updateAirPumpTarget();

    param_server = add_on_set_parameters_callback(
        std::bind(&ArmNode::onParametersChanged, this, std::placeholders::_1));

    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
        kJointStateTopic, 10, std::bind(&ArmNode::jointStateCallback, this, std::placeholders::_1));

    cdc_trans = std::make_unique<CDCTrans>();
    if (!cdc_trans->open(kUsbVid, kUsbPid)) {
        exit_thread = true;
    }

    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        while (!exit_thread) {
            cdc_trans->process_once();
        }
    });
}

ArmNode::~ArmNode()
{
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void ArmNode::jointStateCallback(const sensor_msgs::msg::JointState& msg)
{
    double yuntai = 0.0;
    double left1 = 0.0;
    double left2 = 0.0;
    double left3 = 0.0;
    double right1 = 0.0;
    double right2 = 0.0;
    double right3 = 0.0;

    const bool has_all_joints =
        getJointPosition(msg, kJointNames[0], 0, yuntai) &&
        getJointPosition(msg, kJointNames[1], 1, left1) &&
        getJointPosition(msg, kJointNames[2], 2, left2) &&
        getJointPosition(msg, kJointNames[3], 3, left3) &&
        getJointPosition(msg, kJointNames[4], 4, right1) &&
        getJointPosition(msg, kJointNames[5], 5, right2) &&
        getJointPosition(msg, kJointNames[6], 6, right3);

    if (!has_all_joints) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "joint_states requires yuntai,left1,left2,left3,right1,right2,right3 or at least 7 positions");
        return;
    }

    arm_target.servo1.left_up = static_cast<float>(left3);
    arm_target.servo1.left_low = static_cast<float>(left2);
    arm_target.servo1.left_down = static_cast<float>(left1);
    arm_target.servo1.right_up = static_cast<float>(right3);
    arm_target.servo1.right_low = static_cast<float>(right2);
    arm_target.servo1.right_down = static_cast<float>(right1);
    arm_target.rob01.except_pos = static_cast<float>(yuntai);

    has_joint_target = true;
    sendTarget();
}

rcl_interfaces::msg::SetParametersResult ArmNode::onParametersChanged(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() != kAirPumpParameter) {
            continue;
        }

        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = std::string(kAirPumpParameter) + " must be bool";
            return result;
        }

        enable_air_pump = param.as_bool();
        updateAirPumpTarget();
        if (has_joint_target) {
            sendTarget();
        } else {
            RCLCPP_INFO(
                get_logger(), "enable_air_pump will be sent after the first joint_states target is received");
        }
        RCLCPP_INFO(get_logger(), "enable_air_pump=%s", enable_air_pump ? "true" : "false");
    }

    return result;
}

void ArmNode::updateAirPumpTarget()
{
    const int pump_state = enable_air_pump ? 1 : 0;
    arm_target.arm_pump_left = pump_state;
    arm_target.arm_pump_right = pump_state;
}

void ArmNode::sendTarget()
{
    arm_target.pack_type = kTargetPackType;
    updateAirPumpTarget();
    if (!cdc_trans) {
        return;
    }
    cdc_trans->send_struct(arm_target);
}
