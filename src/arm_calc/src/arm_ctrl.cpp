#include "arm_ctrl.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace arm_calc {

namespace {

constexpr std::array<const char*, 7> kJointNames{
    "yuntai",
    "left1",
    "left2",
    "left3",
    "right1",
    "right2",
    "right3",
};

constexpr int32_t kLeftArmId = 0;
constexpr int32_t kRightArmId = 1;
constexpr int32_t kIdleMode = 0;
constexpr int32_t kJointSpaceMode = 1;
constexpr int32_t kCartesianSpaceMode = 2;

}  // namespace

ArmCtrlNode::ArmCtrlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("arm_calc_node", options) {
    desired_joint_positions_.setZero();
    declare_parameters();
    load_kinematics();
    create_interfaces();
    publish_desired_joint_state();
}

void ArmCtrlNode::declare_parameters() {
    declare_parameter<std::string>("arm_cmd_topic", arm_cmd_topic_);
    declare_parameter<std::string>("joint_state_topic", joint_state_topic_);
    declare_parameter<double>("control_period", control_period_sec_);
    declare_parameter<std::string>("base_link", base_link_);
    declare_parameter<std::string>("left_tip_link", left_tip_link_);
    declare_parameter<std::string>("right_tip_link", right_tip_link_);
    declare_parameter<std::vector<double>>("initial_joint_positions", std::vector<double>(kJointNames.size(), 0.0));
}

void ArmCtrlNode::load_kinematics() {
    arm_cmd_topic_ = get_parameter("arm_cmd_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    control_period_sec_ = std::max(get_parameter("control_period").as_double(), 0.005);
    base_link_ = get_parameter("base_link").as_string();
    left_tip_link_ = get_parameter("left_tip_link").as_string();
    right_tip_link_ = get_parameter("right_tip_link").as_string();

    const auto initial_positions = get_parameter("initial_joint_positions").as_double_array();
    if (initial_positions.size() != kJointNames.size()) {
        throw std::runtime_error("initial_joint_positions must contain 7 values");
    }
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        desired_joint_positions_[static_cast<int>(i)] = initial_positions[i];
    }

    KDL::Tree tree;
    const std::string urdf_xml = fetch_robot_description();
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        throw std::runtime_error("failed to parse arm URDF into KDL tree");
    }
    if (!tree.getChain(base_link_, left_tip_link_, left_chain_)) {
        throw std::runtime_error("failed to build left arm chain: " + base_link_ + " -> " + left_tip_link_);
    }
    if (!tree.getChain(base_link_, right_tip_link_, right_chain_)) {
        throw std::runtime_error("failed to build right arm chain: " + base_link_ + " -> " + right_tip_link_);
    }
    if (left_chain_.getNrOfJoints() != kArmJointDof || right_chain_.getNrOfJoints() != kArmJointDof) {
        throw std::runtime_error("left/right arm chains must each contain 4 movable joints");
    }

    arm_calc_ = std::make_shared<ArmCalc>(left_chain_, right_chain_);
    arm_calc_->set_last_joint_pos(ArmSide::kLeft, desired_arm_position(ArmSide::kLeft));
    arm_calc_->set_last_joint_pos(ArmSide::kRight, desired_arm_position(ArmSide::kRight));
    cartesian_space_move_ = std::make_unique<arm_action::CartesianSpaceMove>(arm_calc_);
}

void ArmCtrlNode::create_interfaces() {
    arm_cmd_sub_ = create_subscription<robot_interfaces::msg::ArmCmd>(
        arm_cmd_topic_, 10, std::bind(&ArmCtrlNode::on_arm_cmd, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(control_period_sec_),
        std::bind(&ArmCtrlNode::publish_control_loop, this));
}

std::string ArmCtrlNode::fetch_robot_description() const {
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        const_cast<ArmCtrlNode*>(this), "/robot_state_publisher");

    if (client->wait_for_service(std::chrono::seconds(1))) {
        try {
            const auto params = client->get_parameters({"robot_description"});
            if (!params.empty()) {
                const std::string urdf_xml = params.front().as_string();
                if (!urdf_xml.empty()) {
                    RCLCPP_INFO(get_logger(), "Loaded robot_description from /robot_state_publisher");
                    return urdf_xml;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to fetch robot_description: %s", e.what());
        }
    }

    RCLCPP_WARN(get_logger(), "Falling back to local arm URDF");
    return load_local_urdf();
}

std::string ArmCtrlNode::load_local_urdf() const {
    const std::string arm_share = ament_index_cpp::get_package_share_directory("arm");
    const std::string urdf_path = arm_share + "/model/arm4.urdf";

    std::ifstream input(urdf_path);
    if (!input.is_open()) {
        throw std::runtime_error("unable to open local URDF: " + urdf_path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void ArmCtrlNode::on_arm_cmd(const robot_interfaces::msg::ArmCmd& msg) {
    MotionMode mode = MotionMode::kIdle;
    if (!parse_mode(msg.mode, mode)) {
        RCLCPP_WARN(get_logger(), "Unsupported ArmCmd mode: %d", msg.mode);
        return;
    }

    if (mode == MotionMode::kIdle) {
        stop_motion();
        publish_desired_joint_state();
        return;
    }

    ArmSide side = ArmSide::kLeft;
    if (!parse_side(msg.arm_id, side)) {
        RCLCPP_WARN(get_logger(), "Unsupported ArmCmd arm_id: %d", msg.arm_id);
        return;
    }

    if (msg.position.data.size() < kArmJointDof) {
        RCLCPP_WARN(get_logger(), "ArmCmd position requires at least 4 values");
        return;
    }

    const double now_sec = get_clock()->now().seconds();
    const double duration = std::max(static_cast<double>(msg.duration), 1e-3);
    const JointPosition start_position = desired_arm_position(side);

    if (mode == MotionMode::kJointSpace) {
        joint_space_move_.start(start_position, to_joint_position(msg.position.data), duration, now_sec);
        if (cartesian_space_move_) {
            cartesian_space_move_->stop();
        }
    } else {
        if (!cartesian_space_move_) {
            RCLCPP_WARN(get_logger(), "Cartesian planner is not initialized");
            return;
        }
        cartesian_space_move_->start(side, start_position, to_cartesian_target(msg.position.data), duration, now_sec);
        joint_space_move_.stop();
    }

    active_side_ = side;
    active_mode_ = mode;
    active_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Accepted %s ArmCmd for %s arm, duration=%.3f",
        mode == MotionMode::kJointSpace ? "joint-space" : "cartesian-space",
        side_name(side),
        duration);
}

void ArmCtrlNode::publish_control_loop() {
    if (!active_) {
        publish_desired_joint_state();
        return;
    }

    const double now_sec = get_clock()->now().seconds();
    JointPosition target_position = desired_arm_position(active_side_);
    bool still_active = false;

    if (active_mode_ == MotionMode::kJointSpace) {
        target_position = joint_space_move_.sample(now_sec);
        still_active = joint_space_move_.active(now_sec);
    } else if (active_mode_ == MotionMode::kCartesianSpace && cartesian_space_move_) {
        bool ik_ok = true;
        const JointPosition seed_position = desired_arm_position(active_side_);
        target_position = cartesian_space_move_->sample(now_sec, seed_position, &ik_ok);
        still_active = cartesian_space_move_->active(now_sec);
        if (!ik_ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK failed while sampling cartesian trajectory");
        }
    }

    write_desired_arm_position(active_side_, target_position);
    publish_desired_joint_state();

    if (!still_active) {
        active_ = false;
        active_mode_ = MotionMode::kIdle;
        joint_space_move_.stop();
        if (cartesian_space_move_) {
            cartesian_space_move_->stop();
        }
        RCLCPP_INFO(get_logger(), "ArmCmd trajectory finished");
    }
}

void ArmCtrlNode::stop_motion() {
    active_ = false;
    active_mode_ = MotionMode::kIdle;
    joint_space_move_.stop();
    if (cartesian_space_move_) {
        cartesian_space_move_->stop();
    }
}

void ArmCtrlNode::publish_desired_joint_state() {
    if (!joint_state_pub_) {
        return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(kJointNames.size());
    msg.position.resize(kJointNames.size(), 0.0);

    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        msg.name.emplace_back(kJointNames[i]);
        msg.position[i] = desired_joint_positions_[static_cast<int>(i)];
    }

    joint_state_pub_->publish(msg);
}

JointPosition ArmCtrlNode::desired_arm_position(ArmSide side) const {
    JointPosition position = JointPosition::Zero();
    position[0] = desired_joint_positions_[0];
    if (side == ArmSide::kLeft) {
        position[1] = desired_joint_positions_[1];
        position[2] = desired_joint_positions_[2];
        position[3] = desired_joint_positions_[3];
    } else {
        position[1] = desired_joint_positions_[4];
        position[2] = desired_joint_positions_[5];
        position[3] = desired_joint_positions_[6];
    }
    return position;
}

void ArmCtrlNode::write_desired_arm_position(ArmSide side, const JointPosition& joints) {
    desired_joint_positions_[0] = joints[0];
    if (side == ArmSide::kLeft) {
        desired_joint_positions_[1] = joints[1];
        desired_joint_positions_[2] = joints[2];
        desired_joint_positions_[3] = joints[3];
    } else {
        desired_joint_positions_[4] = joints[1];
        desired_joint_positions_[5] = joints[2];
        desired_joint_positions_[6] = joints[3];
    }

    if (arm_calc_) {
        arm_calc_->set_last_joint_pos(side, joints);
    }
}

bool ArmCtrlNode::parse_side(int32_t arm_id, ArmSide& side) {
    if (arm_id == kLeftArmId) {
        side = ArmSide::kLeft;
        return true;
    }
    if (arm_id == kRightArmId) {
        side = ArmSide::kRight;
        return true;
    }
    return false;
}

bool ArmCtrlNode::parse_mode(int32_t mode_value, MotionMode& mode) {
    if (mode_value == kIdleMode) {
        mode = MotionMode::kIdle;
        return true;
    }
    if (mode_value == kJointSpaceMode) {
        mode = MotionMode::kJointSpace;
        return true;
    }
    if (mode_value == kCartesianSpaceMode) {
        mode = MotionMode::kCartesianSpace;
        return true;
    }
    return false;
}

JointPosition ArmCtrlNode::to_joint_position(const std::vector<double>& values) {
    JointPosition position = JointPosition::Zero();
    for (std::size_t i = 0; i < kArmJointDof; ++i) {
        position[static_cast<int>(i)] = values[i];
    }
    return position;
}

CartesianTarget ArmCtrlNode::to_cartesian_target(const std::vector<double>& values) {
    CartesianTarget target;
    target.position = Eigen::Vector3d(values[0], values[1], values[2]);
    target.pitch = values[3];
    return target;
}

const char* ArmCtrlNode::side_name(ArmSide side) {
    return side == ArmSide::kLeft ? "left" : "right";
}

}  // namespace arm_calc
