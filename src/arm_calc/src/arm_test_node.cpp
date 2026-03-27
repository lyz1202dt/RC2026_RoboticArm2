#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>

namespace arm_calc {

namespace {

constexpr std::size_t kJointCount = 6;
constexpr char kJointSpaceTargetTopic[] = "joint_space_target";
constexpr char kVisualTargetTopic[] = "visual_target_pose";

class ArmTestNode : public rclcpp::Node {
public:
    ArmTestNode()
        : rclcpp::Node("arm_test_node") {
        declare_parameters();
        joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(kJointSpaceTargetTopic, 10);
        pose_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kVisualTargetTopic, 10);

        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ArmTestNode::on_parameters_changed, this, std::placeholders::_1));
    }

private:
    void declare_parameters() {
        this->declare_parameter<std::vector<double>>("joint_target", std::vector<double>(kJointCount, 0.0));
        this->declare_parameter<std::vector<double>>("pose_position", std::vector<double>{0.7, 0.0, 0.15});
        this->declare_parameter<std::vector<double>>("pose_quaternion", std::vector<double>{1.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::string>("pose_frame_id", "world");
        this->declare_parameter<bool>("publish_joint_target", false);
        this->declare_parameter<bool>("publish_pose_target", false);
    }

    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "joint_target") {
                if (param.as_double_array().size() != kJointCount) {
                    result.successful = false;
                    result.reason = "joint_target must contain 6 elements";
                    return result;
                }
            } else if (param.get_name() == "pose_position") {
                if (param.as_double_array().size() != 3) {
                    result.successful = false;
                    result.reason = "pose_position must contain 3 elements";
                    return result;
                }
            } else if (param.get_name() == "pose_quaternion") {
                if (param.as_double_array().size() != 4) {
                    result.successful = false;
                    result.reason = "pose_quaternion must contain 4 elements";
                    return result;
                }
            } else if (param.get_name() == "publish_joint_target" && !updating_trigger_params_ && param.as_bool()) {
                publish_joint_target();
                reset_trigger_parameter("publish_joint_target");
            } else if (param.get_name() == "publish_pose_target" && !updating_trigger_params_ && param.as_bool()) {
                publish_pose_target();
                reset_trigger_parameter("publish_pose_target");
            }
        }

        return result;
    }

    void publish_joint_target() {
        const auto values = this->get_parameter("joint_target").as_double_array();
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(values.begin(), values.end());
        joint_target_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published joint target with %zu joints", msg.data.size());
    }

    void publish_pose_target() {
        const auto position = this->get_parameter("pose_position").as_double_array();
        const auto quaternion = this->get_parameter("pose_quaternion").as_double_array();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = this->get_parameter("pose_frame_id").as_string();
        msg.pose.position.x = position[0];
        msg.pose.position.y = position[1];
        msg.pose.position.z = position[2];
        msg.pose.orientation.w = quaternion[0];
        msg.pose.orientation.x = quaternion[1];
        msg.pose.orientation.y = quaternion[2];
        msg.pose.orientation.z = quaternion[3];
        pose_target_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published pose target in frame %s", msg.header.frame_id.c_str());
    }

    void reset_trigger_parameter(const std::string& name) {
        updating_trigger_params_ = true;
        this->set_parameter(rclcpp::Parameter(name, false));
        updating_trigger_params_ = false;
    }

    bool updating_trigger_params_{false};
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace

}  // namespace arm_calc

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_calc::ArmTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
