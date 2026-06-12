#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/arm_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace arm_calc {

namespace {

constexpr std::size_t kCommandValueCount = 4;
constexpr char kDefaultArmCmdTopic[] = "arm_cmd";
constexpr char kDefaultJointStateTopic[] = "joint_states";
constexpr int32_t kLeftArmId = 0;
constexpr int32_t kIdleMode = 0;
constexpr int32_t kJointSpaceMode = 1;
constexpr int32_t kCartesianSpaceMode = 2;
constexpr std::array<const char*, kCommandValueCount> kJointParameterNames = {
    "joint1", "joint2", "joint3", "joint4"};
constexpr std::array<const char*, kCommandValueCount> kPoseParameterNames = {
    "pose_x", "pose_y", "pose_z", "pose_pitch"};

bool true_bool_parameter(const rclcpp::Parameter& param) {
    return param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && param.as_bool();
}

}  // namespace

class ArmTestNode : public rclcpp::Node {
public:
    ArmTestNode()
        : rclcpp::Node("arm_test_node") {
        declare_parameters();

        arm_cmd_topic_ = get_parameter("arm_cmd_topic").as_string();
        joint_state_topic_ = get_parameter("joint_state_topic").as_string();

        arm_cmd_pub_ = create_publisher<robot_interfaces::msg::ArmCmd>(arm_cmd_topic_, 10);
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, 10, std::bind(&ArmTestNode::on_joint_state, this, std::placeholders::_1));

        param_callback_ = add_on_set_parameters_callback(
            std::bind(&ArmTestNode::on_parameters_changed, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Arm test node ready: publishing ArmCmd on '%s'",
            arm_cmd_topic_.c_str());
    }

private:
    void declare_parameters() {
        declare_parameter<std::string>("arm_cmd_topic", kDefaultArmCmdTopic);
        declare_parameter<std::string>("joint_state_topic", kDefaultJointStateTopic);
        declare_parameter<int>("arm_id", kLeftArmId);
        declare_parameter<int>("mode", kJointSpaceMode);
        declare_parameter<double>("duration", 3.0);
        declare_parameter<std::vector<double>>("position", std::vector<double>(kCommandValueCount, 0.0));

        for (const auto* name : kJointParameterNames) {
            declare_parameter<double>(name, 0.0);
        }

        declare_parameter<double>("pose_x", 1.2);
        declare_parameter<double>("pose_y", 0.0);
        declare_parameter<double>("pose_z", -0.3);
        declare_parameter<double>("pose_pitch", 0.0);

        declare_parameter<bool>("publish_arm_cmd", false);
        declare_parameter<bool>("publish_joint_target", false);
        declare_parameter<bool>("publish_pose_target", false);
        declare_parameter<bool>("publish_stop", false);
        declare_parameter<bool>("echo_joint_states", false);
    }

    rcl_interfaces::msg::SetParametersResult on_parameters_changed(
        const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            const auto& name = param.get_name();
            if (name == "publish_arm_cmd" && true_bool_parameter(param)) {
                publish_configured_command();
            } else if (name == "publish_joint_target" && true_bool_parameter(param)) {
                publish_joint_target();
            } else if (name == "publish_pose_target" && true_bool_parameter(param)) {
                publish_pose_target();
            } else if (name == "publish_stop" && true_bool_parameter(param)) {
                publish_stop();
            }
        }

        return result;
    }

    void publish_configured_command() {
        std::vector<double> position = get_parameter("position").as_double_array();
        const auto mode = static_cast<int32_t>(get_parameter("mode").as_int());
        publish_command(mode, position, "configured");
    }

    void publish_joint_target() {
        std::vector<double> position;
        position.reserve(kCommandValueCount);
        for (const auto* name : kJointParameterNames) {
            position.push_back(get_parameter(name).as_double());
        }

        publish_command(kJointSpaceMode, position, "joint-space");
    }

    void publish_pose_target() {
        std::vector<double> position;
        position.reserve(kCommandValueCount);
        for (const auto* name : kPoseParameterNames) {
            position.push_back(get_parameter(name).as_double());
        }

        publish_command(kCartesianSpaceMode, position, "cartesian-space");
    }

    void publish_stop() {
        publish_command(kIdleMode, std::vector<double>(kCommandValueCount, 0.0), "stop");
    }

    void publish_command(int32_t mode, const std::vector<double>& position, const char* label) {
        if (mode != kIdleMode && position.size() < kCommandValueCount) {
            RCLCPP_WARN(
                get_logger(),
                "Not publishing %s ArmCmd: position requires at least %zu values",
                label,
                kCommandValueCount);
            return;
        }

        robot_interfaces::msg::ArmCmd msg;
        msg.stamp = now();
        msg.arm_id = static_cast<int32_t>(get_parameter("arm_id").as_int());
        msg.mode = mode;
        msg.duration = static_cast<float>(std::max(get_parameter("duration").as_double(), 1e-3));
        msg.position.data = position;

        arm_cmd_pub_->publish(msg);

        RCLCPP_INFO(
            get_logger(),
            "Published %s ArmCmd: arm_id=%d mode=%d duration=%.3f position=%s",
            label,
            msg.arm_id,
            msg.mode,
            static_cast<double>(msg.duration),
            format_values(msg.position.data).c_str());
    }

    void on_joint_state(const sensor_msgs::msg::JointState& msg) {
        if (!get_parameter("echo_joint_states").as_bool()) {
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "joint_states: %s",
            format_joint_state(msg).c_str());
    }

    static std::string format_values(const std::vector<double>& values) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3) << "[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) {
                stream << ", ";
            }
            stream << values[i];
        }
        stream << "]";
        return stream.str();
    }

    static std::string format_joint_state(const sensor_msgs::msg::JointState& msg) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3);
        const std::size_t count = std::min(msg.name.size(), msg.position.size());
        for (std::size_t i = 0; i < count; ++i) {
            if (i > 0) {
                stream << ", ";
            }
            stream << msg.name[i] << "=" << msg.position[i];
        }
        return stream.str();
    }

    std::string arm_cmd_topic_;
    std::string joint_state_topic_;
    rclcpp::Publisher<robot_interfaces::msg::ArmCmd>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace arm_calc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_calc::ArmTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
