#include "arm_calc/arm_ctrl.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace arm_calc {

namespace {

constexpr char kJointStateTopic[] = "myjoints_state";
constexpr char kJointTargetTopic[] = "myjoints_target";
constexpr char kVisualTargetTopic[] = "visual_target_pose";
constexpr char kRvizJointTopic[] = "joint_states";
constexpr char kMarkerTopic[] = "visualization_marker_array";

geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& value) {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

geometry_msgs::msg::Quaternion ToMsgQuaternion(const Eigen::Quaterniond& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

}  // namespace

ArmCtrlNode::ArmCtrlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("arm_calc_node", options) {
    declare_parameters();
    load_robot_description_and_build_solver();
    create_interfaces();
    param_callback_ = this->add_on_set_parameters_callback(
        std::bind(&ArmCtrlNode::on_parameters_changed, this, std::placeholders::_1));
}

void ArmCtrlNode::declare_parameters() {
    this->declare_parameter<std::string>("motion_mode", "joint_space");
    this->declare_parameter<double>("trajectory_duration", 3.0);
    this->declare_parameter<double>("control_period", 0.02);
    this->declare_parameter<bool>("auto_start", true);
    this->declare_parameter<double>("visual_servo_kp", 2.0);
    this->declare_parameter<double>("visual_servo_max_linear_acceleration", 0.5);
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tip_link", "link6");
    this->declare_parameter<std::vector<double>>("joint_target", std::vector<double>(kJointDoF, 0.0));
    this->declare_parameter<std::vector<double>>("cartesian_target_position", std::vector<double>{0.7, 0.0, 0.15});
    this->declare_parameter<std::vector<double>>("cartesian_target_quaternion", std::vector<double>{1.0, 0.0, 0.0, 0.0});
}

void ArmCtrlNode::create_interfaces() {
    joint_state_sub_ = this->create_subscription<robot_interfaces::msg::Arm>(
        kJointStateTopic, rclcpp::SensorDataQoS(),
        std::bind(&ArmCtrlNode::on_joint_state, this, std::placeholders::_1));

    visual_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        kVisualTargetTopic, 10, std::bind(&ArmCtrlNode::on_visual_target, this, std::placeholders::_1));

    joint_target_pub_ = this->create_publisher<robot_interfaces::msg::Arm>(kJointTargetTopic, 10);
    rviz_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kRvizJointTopic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kMarkerTopic, 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period_sec_), std::bind(&ArmCtrlNode::publish_control_loop, this));
}

void ArmCtrlNode::load_robot_description_and_build_solver() {
    base_link_ = this->get_parameter("base_link").as_string();
    tip_link_ = this->get_parameter("tip_link").as_string();
    trajectory_duration_sec_ = this->get_parameter("trajectory_duration").as_double();
    control_period_sec_ = this->get_parameter("control_period").as_double();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    visual_servo_kp_ = std::max(this->get_parameter("visual_servo_kp").as_double(), 0.0);
    visual_servo_max_linear_acceleration_ =
        std::max(this->get_parameter("visual_servo_max_linear_acceleration").as_double(), 0.0);
    motion_mode_ = parse_motion_mode(this->get_parameter("motion_mode").as_string());

    const std::vector<double> joint_target = get_double_array_param(*this, "joint_target", kJointDoF);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        joint_target_state_.position[static_cast<int>(i)] = joint_target[i];
    }

    const std::vector<double> cartesian_position = get_double_array_param(*this, "cartesian_target_position", 3);
    const std::vector<double> cartesian_quaternion = get_double_array_param(*this, "cartesian_target_quaternion", 4);
    cartesian_target_.position = Eigen::Vector3d(cartesian_position[0], cartesian_position[1], cartesian_position[2]);
    cartesian_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(
        cartesian_quaternion[0], cartesian_quaternion[1], cartesian_quaternion[2], cartesian_quaternion[3]));
    visual_target_ = cartesian_target_;

    KDL::Tree tree;
    const std::string urdf_xml = fetch_robot_description();
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        throw std::runtime_error("failed to parse arm URDF into KDL tree");
    }
    if (!tree.getChain(base_link_, tip_link_, arm_chain_)) {
        throw std::runtime_error("failed to build KDL chain from " + base_link_ + " to " + tip_link_);
    }

    arm_calc_ = std::make_shared<ArmCalc>(arm_chain_);
    joint_space_move_ = std::make_shared<arm_action::JointSpaceMove>();
    cartesian_space_move_ = std::make_shared<arm_action::JCartesianSpaceMove>(arm_calc_);
    visual_servo_move_ = std::make_shared<arm_action::VisualServoMove>(arm_calc_);
    visual_servo_move_->set_kp(visual_servo_kp_);
    visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
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
                    RCLCPP_INFO(this->get_logger(), "Loaded robot_description from /robot_state_publisher");
                    return urdf_xml;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to fetch robot_description from parameter service: %s", e.what());
        }
    }

    RCLCPP_WARN(this->get_logger(), "Falling back to local URDF file");
    return load_local_urdf();
}

std::string ArmCtrlNode::load_local_urdf() const {
    const std::string arm_share = ament_index_cpp::get_package_share_directory("arm");
    const std::string urdf_path = arm_share + "/model/robotic_arm.urdf";
    std::ifstream input(urdf_path);
    if (!input.is_open()) {
        throw std::runtime_error("unable to open local URDF: " + urdf_path);
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void ArmCtrlNode::refresh_plan(double now_sec) {
    if (!arm_calc_ || !has_joint_state_) {
        return;
    }

    switch (motion_mode_) {
        case MotionMode::kJointSpace:
            joint_space_move_->set_start_state(current_joint_state_);
            joint_space_move_->set_goal_state(joint_target_state_, trajectory_duration_sec_);
            if (auto_start_) {
                joint_space_move_->start(now_sec);
            }
            break;
        case MotionMode::kCartesianSpace:
            cartesian_space_move_->set_start_state(current_joint_state_);
            cartesian_space_move_->set_goal_state(cartesian_target_, trajectory_duration_sec_);
            if (auto_start_) {
                cartesian_space_move_->start(now_sec);
            }
            break;
        case MotionMode::kVisualServo:
            visual_servo_move_->set_kp(visual_servo_kp_);
            visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
            visual_servo_move_->set_current_joint_state(current_joint_state_);
            visual_servo_move_->set_target_pose(visual_target_);
            break;
    }
    planners_ready_ = true;
}

void ArmCtrlNode::publish_control_loop() {
    if (!has_joint_state_ || !planners_ready_) {
        return;
    }

    const double now_sec = this->get_clock()->now().seconds();
    JointTrajectoryPoint target_point;

    switch (motion_mode_) {
        case MotionMode::kJointSpace:
            target_point = joint_space_move_->sample(now_sec);
            break;
        case MotionMode::kCartesianSpace:
            target_point = cartesian_space_move_->sample(now_sec);
            break;
        case MotionMode::kVisualServo:
            visual_servo_move_->set_current_joint_state(current_joint_state_);
            target_point = visual_servo_move_->sample(now_sec);
            break;
    }

    publish_joint_target(target_point);
    publish_visualization(target_point);
}

void ArmCtrlNode::publish_joint_target(const JointTrajectoryPoint& point) {
    const rclcpp::Time stamp = this->get_clock()->now();
    joint_target_pub_->publish(to_arm_message(point));
    rviz_joint_pub_->publish(to_joint_state_msg(point, stamp));
}

void ArmCtrlNode::publish_visualization(const JointTrajectoryPoint& target_point) {
    if (!arm_calc_) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = this->now();

    visualization_msgs::msg::Marker current_marker;
    current_marker.header.frame_id = base_link_;
    current_marker.header.stamp = stamp;
    current_marker.ns = "arm_ctrl";
    current_marker.id = 0;
    current_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_marker.action = visualization_msgs::msg::Marker::ADD;
    current_marker.scale.x = 0.04;
    current_marker.scale.y = 0.04;
    current_marker.scale.z = 0.04;
    current_marker.color.r = 0.1F;
    current_marker.color.g = 0.8F;
    current_marker.color.b = 0.2F;
    current_marker.color.a = 1.0F;
    current_marker.pose.position = ToPoint(arm_calc_->end_pose(current_joint_state_.position).position);
    current_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker target_marker = current_marker;
    target_marker.id = 1;
    target_marker.color.r = 0.95F;
    target_marker.color.g = 0.25F;
    target_marker.color.b = 0.15F;
    target_marker.pose.position = ToPoint(arm_calc_->end_pose(target_point.position).position);
    target_marker.pose.orientation = ToMsgQuaternion(arm_calc_->end_pose(target_point.position).orientation);

    visualization_msgs::msg::Marker line_marker = current_marker;
    line_marker.id = 2;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.scale.x = 0.01;
    line_marker.color.r = 0.1F;
    line_marker.color.g = 0.4F;
    line_marker.color.b = 0.95F;
    line_marker.points.push_back(current_marker.pose.position);
    line_marker.points.push_back(target_marker.pose.position);

    markers.markers.push_back(current_marker);
    markers.markers.push_back(target_marker);
    markers.markers.push_back(line_marker);
    marker_pub_->publish(markers);
}

void ArmCtrlNode::on_joint_state(const robot_interfaces::msg::Arm& msg) {
    current_joint_state_ = from_arm_message(msg);
    has_joint_state_ = true;

    if (!planners_ready_) {
        refresh_plan(this->get_clock()->now().seconds());
    }
}

void ArmCtrlNode::on_visual_target(const geometry_msgs::msg::PoseStamped& msg) {
    visual_target_.position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    visual_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z));

    if (visual_servo_move_) {
        visual_servo_move_->set_target_pose(visual_target_);
    }

    if (motion_mode_ == MotionMode::kVisualServo && has_joint_state_) {
        planners_ready_ = true;
    }
}

rcl_interfaces::msg::SetParametersResult ArmCtrlNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "motion_mode") {
            motion_mode_ = parse_motion_mode(param.as_string());
        } else if (param.get_name() == "trajectory_duration") {
            trajectory_duration_sec_ = std::max(param.as_double(), 0.1);
        } else if (param.get_name() == "control_period") {
            control_period_sec_ = std::max(param.as_double(), 0.005);
        } else if (param.get_name() == "auto_start") {
            auto_start_ = param.as_bool();
        } else if (param.get_name() == "visual_servo_kp") {
            visual_servo_kp_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                visual_servo_move_->set_kp(visual_servo_kp_);
            }
        } else if (param.get_name() == "visual_servo_max_linear_acceleration") {
            visual_servo_max_linear_acceleration_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
            }
        } else if (param.get_name() == "joint_target") {
            const auto values = param.as_double_array();
            if (values.size() != kJointDoF) {
                result.successful = false;
                result.reason = "joint_target must contain 6 values";
                return result;
            }
            for (std::size_t i = 0; i < kJointDoF; ++i) {
                joint_target_state_.position[static_cast<int>(i)] = values[i];
            }
        } else if (param.get_name() == "cartesian_target_position") {
            const auto values = param.as_double_array();
            if (values.size() != 3) {
                result.successful = false;
                result.reason = "cartesian_target_position must contain 3 values";
                return result;
            }
            cartesian_target_.position = Eigen::Vector3d(values[0], values[1], values[2]);
        } else if (param.get_name() == "cartesian_target_quaternion") {
            const auto values = param.as_double_array();
            if (values.size() != 4) {
                result.successful = false;
                result.reason = "cartesian_target_quaternion must contain 4 values";
                return result;
            }
            cartesian_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(values[0], values[1], values[2], values[3]));
        }
    }

    if (control_timer_) {
        control_timer_->cancel();
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_sec_), std::bind(&ArmCtrlNode::publish_control_loop, this));
    }

    planners_ready_ = false;
    if (has_joint_state_) {
        refresh_plan(this->get_clock()->now().seconds());
    }
    return result;
}

ArmCtrlNode::MotionMode ArmCtrlNode::parse_motion_mode(const std::string& mode_name) {
    if (mode_name == "joint_space") {
        return MotionMode::kJointSpace;
    }
    if (mode_name == "cartesian_space") {
        return MotionMode::kCartesianSpace;
    }
    if (mode_name == "visual_servo") {
        return MotionMode::kVisualServo;
    }
    throw std::invalid_argument("unsupported motion_mode: " + mode_name);
}

JointState ArmCtrlNode::from_arm_message(const robot_interfaces::msg::Arm& msg) {
    JointState state;
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        state.position[static_cast<int>(i)] = msg.motor[i].rad;
        state.velocity[static_cast<int>(i)] = msg.motor[i].omega;
        state.torque[static_cast<int>(i)] = msg.motor[i].torque;
    }
    return state;
}

robot_interfaces::msg::Arm ArmCtrlNode::to_arm_message(const JointTrajectoryPoint& point) {
    robot_interfaces::msg::Arm msg;
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        msg.motor[i].rad = static_cast<float>(point.position[static_cast<int>(i)]);
        msg.motor[i].omega = static_cast<float>(point.velocity[static_cast<int>(i)]);
        msg.motor[i].torque = static_cast<float>(point.torque[static_cast<int>(i)]);
    }
    return msg;
}

sensor_msgs::msg::JointState ArmCtrlNode::to_joint_state_msg(const JointTrajectoryPoint& point, const rclcpp::Time& stamp) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    msg.position.resize(kJointDoF);
    msg.velocity.resize(kJointDoF);
    msg.effort.resize(kJointDoF);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        msg.position[i] = point.position[static_cast<int>(i)];
        msg.velocity[i] = point.velocity[static_cast<int>(i)];
        msg.effort[i] = point.torque[static_cast<int>(i)];
    }
    return msg;
}

std::vector<double> ArmCtrlNode::get_double_array_param(const rclcpp::Node& node,
                                                        const std::string& name,
                                                        std::size_t expected_size) {
    const auto values = node.get_parameter(name).as_double_array();
    if (values.size() != expected_size) {
        throw std::runtime_error("parameter " + name + " expected size " + std::to_string(expected_size));
    }
    return values;
}

}  // namespace arm_calc
