#pragma once

#include "arm_action/cartesian_space_move.hpp"
#include "arm_action/joint_space_move.hpp"
#include "arm_calc/arm_calc.hpp"

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/arm_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace arm_calc {

class ArmCtrlNode : public rclcpp::Node {
public:
    explicit ArmCtrlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using RobotJointPosition = Eigen::Matrix<double, 7, 1>;

    enum class MotionMode { kIdle = 0, kJointSpace = 1, kCartesianSpace = 2 };

    void declare_parameters();
    void load_kinematics();
    void create_interfaces();

    std::string fetch_robot_description() const;
    std::string load_local_urdf() const;

    void on_arm_cmd(const robot_interfaces::msg::ArmCmd& msg);
    void publish_control_loop();
    void stop_motion();
    void publish_desired_joint_state();

    JointPosition desired_arm_position(ArmSide side) const;
    void write_desired_arm_position(ArmSide side, const JointPosition& joints);

    static bool parse_side(int32_t arm_id, ArmSide& side);
    static bool parse_mode(int32_t mode_value, MotionMode& mode);
    static JointPosition to_joint_position(const std::vector<double>& values);
    static CartesianTarget to_cartesian_target(const std::vector<double>& values);
    static const char* side_name(ArmSide side);

    std::string arm_cmd_topic_{"arm_cmd"};
    std::string joint_state_topic_{"joint_states"};
    std::string base_link_{"base_link"};
    std::string left_tip_link_{"left4"};
    std::string right_tip_link_{"right4"};
    double control_period_sec_{0.02};

    RobotJointPosition desired_joint_positions_;

    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    std::shared_ptr<ArmCalc> arm_calc_;
    arm_action::JointSpaceMove joint_space_move_;
    std::unique_ptr<arm_action::CartesianSpaceMove> cartesian_space_move_;

    bool active_{false};
    ArmSide active_side_{ArmSide::kLeft};
    MotionMode active_mode_{MotionMode::kIdle};

    rclcpp::Subscription<robot_interfaces::msg::ArmCmd>::SharedPtr arm_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace arm_calc
