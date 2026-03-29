#include "arm_task/task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace arm_task {

ArmTaskNode::ArmTaskNode(const rclcpp::NodeOptions& options)
    : Node("arm_task", options) {

    RCLCPP_INFO(this->get_logger(), "Initializing ArmTaskNode...");

    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    this->declare_parameter<int32_t>("arm_task", 0);
    this->declare_parameter<bool>("stop_visual_servo", false);
    this->declare_parameter<double>("trajectory_duration", 3.0);
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("visual_servo_kp", 2.0);
    this->declare_parameter<double>("visual_servo_max_linear_acc", 0.5);
    this->declare_parameter<int>("air_pump_pin", 0);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("object_frame", "target_object");
    this->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");

    // Get parameters
    this->get_parameter("trajectory_duration", trajectory_duration_);
    this->get_parameter("approach_distance", approach_distance_);
    this->get_parameter("visual_servo_kp", visual_servo_kp_);
    this->get_parameter("visual_servo_max_linear_acc", visual_servo_max_linear_acc_);
    this->get_parameter("air_pump_pin", air_pump_pin_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("object_frame", object_frame_);
    this->get_parameter("arm_calc_node_name", arm_calc_node_name_);

    // Load arm positions from YAML
    load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);

    // Create subscribers
    place_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "place_target_pose", 10, std::bind(&ArmTaskNode::on_place_target_pose, this, std::placeholders::_1));

    // Setup parameter callback
    param_callback_ = this->add_on_set_parameters_callback(std::bind(&ArmTaskNode::on_parameters_changed, this, std::placeholders::_1));

    // Create parameter client for arm_calc node
    arm_calc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, arm_calc_node_name_);
    RCLCPP_INFO(this->get_logger(), "Using arm_calc parameter client target: %s", arm_calc_node_name_.c_str());

    // Start task execution thread
    task_thread_ = std::thread(&ArmTaskNode::task_execution_thread, this);

    RCLCPP_INFO(this->get_logger(), "ArmTaskNode initialized successfully");
}

ArmTaskNode::~ArmTaskNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down ArmTaskNode...");
    shutdown_requested_  = true;
    visual_servo_active_ = false;

    if (task_thread_.joinable()) {
        task_thread_.join();
    }

    if (visual_servo_thread_.joinable()) {
        visual_servo_thread_.join();
    }
}

void ArmTaskNode::load_arm_positions_from_yaml() {
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("arm_task");
        std::string yaml_path     = package_share + "/config/arm_position.yaml";

        YAML::Node config = YAML::LoadFile(yaml_path);

        if (config["arm_positions"]) {
            for (const auto& pos : config["arm_positions"]) {
                int index                  = pos["index"].as<int>();
                std::vector<double> joints = pos["joints"].as<std::vector<double>>();
                arm_positions_[index]      = joints;
                RCLCPP_INFO(this->get_logger(), "Loaded position %d with %zu joints", index, joints.size());
            }
        }

        if (config["ready_position"]) {
            ready_position_ = config["ready_position"].as<std::vector<double>>();
            RCLCPP_INFO(this->get_logger(), "Loaded ready position with %zu joints", ready_position_.size());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load arm positions: %s", e.what());
    }
}

void ArmTaskNode::on_place_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    place_target_pose_ = *msg;
    has_place_target_  = true;
    RCLCPP_INFO(this->get_logger(), "Received place target pose");
}

rcl_interfaces::msg::SetParametersResult ArmTaskNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "arm_task") {
            int32_t new_mode = param.as_int();
            arm_task_mode_   = new_mode;
            RCLCPP_INFO(this->get_logger(), "Task mode changed to: %d", new_mode);
        } else if (param.get_name() == "stop_visual_servo") {
            bool stop = param.as_bool();
            if (stop) {
                visual_servo_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Visual servo stopped via parameter");
                // Reset parameter back to false
                this->set_parameter(rclcpp::Parameter("stop_visual_servo", false));
            }
        }
    }

    return result;
}

void ArmTaskNode::task_execution_thread() {
    RCLCPP_INFO(this->get_logger(), "Task execution thread started");

    while (!shutdown_requested_) {
        execute_task_state_machine();
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Task execution thread stopped");
}

void ArmTaskNode::execute_task_state_machine() {
    int32_t current_mode = arm_task_mode_.load();

    if (current_mode == 0) {
       //CPP_INFO(this->get_logger(), "空闲任务");
        return;
    }

    if (task_running_) {
        // Task already running, skip
        return;
    }

    // Lock and execute task
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_running_ = true;

    try {
        if (current_mode == 1) {
            // Grasp flow
            RCLCPP_INFO(this->get_logger(), "Starting grasp flow");
            execute_grasp_flow();
        } else if (current_mode == 2) {
            // Place flow
            RCLCPP_INFO(this->get_logger(), "Starting place flow");
            execute_place_flow();
        } else if (current_mode >= 10 && current_mode < 20) {
            // Move to position x
            int position_index = current_mode - 10;
            RCLCPP_INFO(this->get_logger(), "Moving to position %d", position_index);
            execute_move_to_position(position_index);
        }

        // Reset mode to standby after completion
        arm_task_mode_ = 0;
        this->set_parameter(rclcpp::Parameter("arm_task", 0));

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed: %s", e.what());
        arm_task_mode_ = 0;
        this->set_parameter(rclcpp::Parameter("arm_task", 0));
    }

    task_running_ = false;
}

void ArmTaskNode::execute_grasp_flow() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 2. Wait for object pose from camera
    RCLCPP_INFO(this->get_logger(), "等待相机提供物体位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    int retry_count = 0;
    while (!get_object_pose_in_base_frame(object_pose) && retry_count < 50) {
        std::this_thread::sleep_for(100ms);
        retry_count++;
    }

    if (retry_count >= 50) {
        RCLCPP_ERROR(this->get_logger(), "从相机获取目标位姿失败");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), "物体在坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    // 强制规定姿态
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    // 3. Move to approach position (distance above target)
    RCLCPP_INFO(this->get_logger(), "移动到接近位置");
    auto approach_pose = create_approach_pose(object_pose, -0.1);
    execute_cartesian_space_trajectory(approach_pose, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 4. Execute visual servo to grasp object
    RCLCPP_INFO(this->get_logger(), "开始视觉伺服抓取");
    execute_visual_servo(object_pose);
    std::this_thread::sleep_for(3000ms); // Wait for visual servo to complete


    visual_servo_active_ = false;

    // 5. Activate air pump to grasp object
    RCLCPP_INFO(this->get_logger(), "启动气泵");
    set_parameter_on_remote_node("air_pump_controller", "pump_state", rclcpp::Parameter("pump_state", true));
    std::this_thread::sleep_for(500ms);

    // 6. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_place_flow() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "Moving to ready position");
    execute_joint_space_trajectory(ready_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 2. Check for place target pose
    geometry_msgs::msg::PoseStamped place_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_place_target_) {
            RCLCPP_ERROR(this->get_logger(), "No place target pose available");
            return;
        }
        place_pose = place_target_pose_;
    }

    RCLCPP_INFO(
        this->get_logger(), "Place target: [%.3f, %.3f, %.3f]", place_pose.pose.position.x, place_pose.pose.position.y,
        place_pose.pose.position.z);

    // 3. Execute Cartesian trajectory to place position
    RCLCPP_INFO(this->get_logger(), "Moving to place position");
    execute_cartesian_space_trajectory(place_pose, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 4. Deactivate air pump to release object
    RCLCPP_INFO(this->get_logger(), "Deactivating air pump");
    set_parameter_on_remote_node("air_pump_controller", "pump_state", rclcpp::Parameter("pump_state", false));
    std::this_thread::sleep_for(500ms);

    // 5. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "Moving back to ready position");
    execute_joint_space_trajectory(ready_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "Place flow completed");
}

void ArmTaskNode::execute_move_to_position(int position_index) {
    if (arm_positions_.find(position_index) == arm_positions_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Position %d not found in configuration", position_index);
        return;
    }

    const auto& joint_angles = arm_positions_[position_index];
    RCLCPP_INFO(this->get_logger(), "Moving to position %d", position_index);
    execute_joint_space_trajectory(joint_angles, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "Move to position %d completed", position_index);
}

void ArmTaskNode::stop_arm_motion() {
    RCLCPP_INFO(this->get_logger(), "Stopping arm motion");
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", false)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration) {
    RCLCPP_INFO(this->get_logger(), "Executing joint space trajectory");

    // Publish joint target
    std_msgs::msg::Float64MultiArray msg;
    msg.data = joint_angles;
    joint_space_target_pub_->publish(msg);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 1)});

        std::this_thread::sleep_for(100ms);

        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration) {

    RCLCPP_INFO(this->get_logger(), "Executing Cartesian space trajectory");

    // Publish visual target
    visual_target_pub_->publish(target_pose);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 2)});

        std::this_thread::sleep_for(100ms);

        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::execute_visual_servo(const geometry_msgs::msg::PoseStamped& target_pose) {
    RCLCPP_INFO(this->get_logger(), "Executing visual servo");

    // Store target pose for publishing thread
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target_object_pose_ = target_pose;
        has_object_pose_    = true;
    }

    // Set visual servo parameters
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters(
            {rclcpp::Parameter("visual_servo_kp", visual_servo_kp_),
             rclcpp::Parameter("visual_servo_max_linear_acceleration", visual_servo_max_linear_acc_), rclcpp::Parameter("motion_mode", 3)});

        std::this_thread::sleep_for(100ms);

        // Start visual servo publishing thread
        visual_servo_active_ = true;
        if (visual_servo_thread_.joinable()) {
            visual_servo_thread_.join();
        }
        visual_servo_thread_ = std::thread(&ArmTaskNode::visual_servo_publish_thread, this);

        // Start execution
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::visual_servo_publish_thread() {
    RCLCPP_INFO(this->get_logger(), "视觉伺服线程开始执行");

    rclcpp::Rate rate(100); // 100 Hz

    while (visual_servo_active_ && !shutdown_requested_) {
        geometry_msgs::msg::PoseStamped pose_to_publish;
        bool has_pose = false;

        // Try to get current object pose from TF
        if (get_object_pose_in_base_frame(pose_to_publish)) {
            has_pose = true;
            RCLCPP_INFO(get_logger(), "得到目标");
        } else {
            // Fall back to stored pose
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (has_object_pose_) {
                pose_to_publish = target_object_pose_;
                has_pose        = true;
            }
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 1.57, 0.0);
        pose_to_publish.pose.orientation.w = q.w();
        pose_to_publish.pose.orientation.x = q.x();
        pose_to_publish.pose.orientation.y = q.y();
        pose_to_publish.pose.orientation.x = q.z();
        RCLCPP_INFO(
            get_logger(), "期望坐标:(%lf,%lf,%lf", pose_to_publish.pose.position.x, pose_to_publish.pose.position.y,
            pose_to_publish.pose.position.z);

        if (has_pose) {
            visual_target_pub_->publish(pose_to_publish);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No object pose available for visual servo");
            visual_servo_active_ = false;
            break;
        }

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "视觉伺服线程结束执行");
}

bool ArmTaskNode::get_object_pose_in_base_frame(geometry_msgs::msg::PoseStamped& pose_out) {
    try {
        // Look up transform from base_link to camera_link
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(base_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));

        // The object is assumed to be at the camera frame origin
        // (in real scenario, you'd get object pose relative to camera)
        pose_out.header.frame_id  = base_frame_;
        pose_out.header.stamp     = this->now();
        pose_out.pose.position.x  = transform.transform.translation.x;
        pose_out.pose.position.y  = transform.transform.translation.y;
        pose_out.pose.position.z  = transform.transform.translation.z;
        pose_out.pose.orientation = transform.transform.rotation;
        return true;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "从 %s 到 %s变换失败: %s", object_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

void ArmTaskNode::set_parameter_on_remote_node(
    const std::string& node_name, const std::string& /* param_name */, const rclcpp::Parameter& param) {
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);

    if (param_client->wait_for_service(1s)) {
        param_client->set_parameters({param});
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter service for %s not available", node_name.c_str());
    }
}

geometry_msgs::msg::PoseStamped ArmTaskNode::create_approach_pose(const geometry_msgs::msg::PoseStamped& target_pose, double distance) {

    geometry_msgs::msg::PoseStamped approach_pose = target_pose;
    approach_pose.pose.position.x += distance;

    return approach_pose;
}

} // namespace arm_task
