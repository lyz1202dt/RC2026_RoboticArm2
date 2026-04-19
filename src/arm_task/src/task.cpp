#include "arm_task/task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/armmode.hpp>
#include <robot_interfaces/msg/vis.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace arm_task {

namespace {

constexpr double kVisualServoExitPositionToleranceMeters = 0.015;
constexpr double kVisualServoConvergenceTimeoutSec       = 20.0;

} // namespace

ArmTaskNode::ArmTaskNode(const rclcpp::NodeOptions& options)
    : Node("arm_task", options) {

    RCLCPP_INFO(this->get_logger(), "Initializing ArmTaskNode...");

    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declare parameters
    this->declare_parameter<int32_t>("arm_task", 0);
    this->declare_parameter<bool>("stop_visual_servo", false);
    this->declare_parameter<double>("trajectory_duration", 3.0);
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("visual_servo_kp", 1.6);
    this->declare_parameter<double>("visual_servo_max_linear_acc", 0.5);
    this->declare_parameter<int>("air_pump_pin", 0);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("object_frame", "target_object");
    this->declare_parameter<std::string>("tip_frame", "link5");
    this->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");

    // Get parameters
    this->get_parameter("trajectory_duration", trajectory_duration_);
    this->get_parameter("approach_distance", approach_distance_);
    // this->get_parameter("visual_servo_kp", visual_servo_kp_);
    this->get_parameter("visual_servo_max_linear_acc", visual_servo_max_linear_acc_);
    this->get_parameter("air_pump_pin", air_pump_pin_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("object_frame", object_frame_);
    this->get_parameter("tip_frame", tip_frame_);
    this->get_parameter("arm_calc_node_name", arm_calc_node_name_);

    // Load arm positions from YAML
    load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);

    vision_sub_ = this->create_subscription<robot_interfaces::msg::Vis>(
        "pnp_move", 10, std::bind(&ArmTaskNode::vision_callback, this, std::placeholders::_1));


    air_pub_ = this->create_publisher<robot_interfaces::msg::Armmode>("air_pump_target", 10);

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

        if (config["place_position"]) {
            place_position_2 = config["place_position"].as<std::vector<double>>();
            RCLCPP_INFO(this->get_logger(), "Loaded place position with %zu joints", place_position_2.size());
        }

         if (config["home_position"]) {
            home_position_ = config["home_position"].as<std::vector<double>>();
            RCLCPP_INFO(this->get_logger(), "Loaded place position with %zu joints", home_position_.size());
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
        // CPP_INFO(this->get_logger(), "空闲任务");
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
            RCLCPP_INFO(this->get_logger(), "开始抓取任务");
            execute_grasp_flow();
        } else if (current_mode == 2) {
            // Place flow
            RCLCPP_INFO(this->get_logger(), "开始放置任务");
            execute_place_flow();
        } else if (current_mode == 3) {
            // Place flow
            RCLCPP_INFO(this->get_logger(), "开始纯关节抓取任务");
            execute_place_flow_rad();
        } else if (current_mode == 4) {         
            RCLCPP_INFO(this->get_logger(), "开始纯关节放置任务");
            execute_place_place_rad();
        } else if (current_mode == 5) {
            // Place flow
            RCLCPP_INFO(this->get_logger(), "开始抓取任务");
            execute_place_place_rad();
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
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 300));

    std::this_thread::sleep_for(std::chrono::seconds(3));


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

    // // 4. Execute visual servo to grasp object
    // RCLCPP_INFO(this->get_logger(), "开始视觉伺服抓取");
    // execute_visual_servo(object_pose);
    // wait_for_visual_servo_convergence(kVisualServoExitPositionToleranceMeters, kVisualServoConvergenceTimeoutSec);
    // visual_servo_active_ = false;

    // stop_arm_motion();  // 必须先停止上一次视觉伺服，否则 mode 切换会失效

     RCLCPP_INFO(this->get_logger(), "kaISHI启动气泵");
    robot_interfaces::msg::Armmode msg;
    msg.mode = 1;
    air_pub_->publish(msg);
    std::this_thread::sleep_for(500ms);


    // 6. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_place_flow() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 2. Check for place target pose
    geometry_msgs::msg::PoseStamped place_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_place_target_) {
            RCLCPP_ERROR(this->get_logger(), "没有检测到期望位置");
            return;
        }
        place_pose = place_target_pose_;
    }

    RCLCPP_INFO(
        this->get_logger(), "放置位置: [%.3f, %.3f, %.3f]", place_pose.pose.position.x, place_pose.pose.position.y,
        place_pose.pose.position.z);

    // 3. Execute Cartesian trajectory to place position
    RCLCPP_INFO(this->get_logger(), "移动到目标放置位置");
    execute_cartesian_space_trajectory(place_pose, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 4. Deactivate air pump to release object
    RCLCPP_INFO(this->get_logger(), "关掉气泵");
    set_parameter_on_remote_node("air_pump_controller", "pump_state", rclcpp::Parameter("pump_state", false));
    std::this_thread::sleep_for(500ms);

    // 5. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "返回准备位置");
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "放块任务结束");
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
    RCLCPP_INFO(this->get_logger(), "停止机械臂运动");
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", false)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration) {
    RCLCPP_INFO(this->get_logger(), "执行关节轨迹规划");

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

    RCLCPP_INFO(this->get_logger(), "执行笛卡尔空间轨迹规划");

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
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("motion_mode", 3)});

        std::this_thread::sleep_for(100ms);

        {
            std::lock_guard<std::mutex> lock(visual_servo_state_mutex_);
            visual_servo_result_ready_ = false;
            visual_servo_succeeded_    = false;
        }

        // Start visual servo publishing thread
        visual_servo_active_ = false;
        if (visual_servo_thread_.joinable()) {
            visual_servo_thread_.join();
        }
        visual_servo_active_ = true;
        visual_servo_thread_ = std::thread(&ArmTaskNode::visual_servo_publish_thread, this);

        // Start execution
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}


bool ArmTaskNode::wait_for_visual_servo_convergence(double position_tolerance_m, double timeout_sec) {
    std::unique_lock<std::mutex> lock(visual_servo_state_mutex_);
    const bool completed = visual_servo_state_cv_.wait_for(lock, std::chrono::duration<double>(timeout_sec), [this]() {
        return visual_servo_result_ready_ || shutdown_requested_ || !visual_servo_active_;
    });

    if (visual_servo_result_ready_) {
        return visual_servo_succeeded_;
    }

    if (!completed) {
        RCLCPP_WARN(this->get_logger(), "视觉伺服在 %.1f s 内未收敛到 %.3f m，主动停止视觉伺服", timeout_sec, position_tolerance_m);
    } else {
        RCLCPP_WARN(this->get_logger(), "视觉伺服在线程结束前未报告收敛结果，主动停止视觉伺服");
    }

    lock.unlock();
    visual_servo_active_ = false;
    visual_servo_state_cv_.notify_all();
    if (visual_servo_thread_.joinable()) {
        visual_servo_thread_.join();
    }

    return false;
}

void ArmTaskNode::visual_servo_publish_thread() {
    RCLCPP_INFO(this->get_logger(), "视觉伺服线程开始执行");

    rclcpp::Rate rate(100); // 100 Hz
    constexpr double kCameraDataLockDistanceMeters                  = 0.35;
    constexpr double kVisualServoConvergencePositionToleranceMeters = kVisualServoExitPositionToleranceMeters;
    bool camera_data_locked                                         = false;
    geometry_msgs::msg::PoseStamped last_trusted_pose;
    bool has_last_trusted_pose = false;

    auto publish_visual_servo_result = [this](bool succeeded) {
        {
            std::lock_guard<std::mutex> lock(visual_servo_state_mutex_);
            if (visual_servo_result_ready_) {
                return;
            }
            visual_servo_result_ready_ = true;
            visual_servo_succeeded_    = succeeded;
        }
        visual_servo_state_cv_.notify_all();
    };

    while (visual_servo_active_ && !shutdown_requested_) {
        geometry_msgs::msg::PoseStamped pose_to_publish;
        bool has_pose = false;

        if (!camera_data_locked) {
            // Try to get current object pose from TF
            try {
                auto target_in_base =
                    tf_buffer_->lookupTransform(base_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
                pose_to_publish.pose.position.x = target_in_base.transform.translation.x;
                pose_to_publish.pose.position.y = target_in_base.transform.translation.y;
                pose_to_publish.pose.position.z = target_in_base.transform.translation.z;
                pose_to_publish.header.frame_id = base_frame_;
                pose_to_publish.header.stamp    = this->now();
                has_pose                        = true;
                last_trusted_pose               = pose_to_publish;
                has_last_trusted_pose           = true;

                auto target_in_camera =
                    tf_buffer_->lookupTransform(camera_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
                const auto& translation = target_in_camera.transform.translation;
                const double distance_to_target =
                    std::sqrt(translation.x * translation.x + translation.y * translation.y + translation.z * translation.z);

                if (distance_to_target < kCameraDataLockDistanceMeters) {
                    camera_data_locked = true;
                    RCLCPP_INFO(
                        this->get_logger(), "camera_data_locked=true, %s 到 %s 距离为 %.3f m", camera_frame_.c_str(), object_frame_.c_str(),
                        distance_to_target);
                }

                RCLCPP_INFO_THROTTLE(
                    get_logger(), *this->get_clock(), 100, "得到目标(%lf, %lf, %lf)", pose_to_publish.pose.position.x,
                    pose_to_publish.pose.position.y, pose_to_publish.pose.position.z);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "获取变换失败: %s", ex.what());
            }
        }

        if (camera_data_locked && has_last_trusted_pose) {
            pose_to_publish = last_trusted_pose;
            has_pose        = true;

            try {
                const auto ee_tf = tf_buffer_->lookupTransform(base_frame_, tip_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
                const double dx  = pose_to_publish.pose.position.x - ee_tf.transform.translation.x;
                const double dy  = pose_to_publish.pose.position.y - ee_tf.transform.translation.y;
                const double dz  = pose_to_publish.pose.position.z - ee_tf.transform.translation.z;
                const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 500,
                    "视觉伺服位置误差: %.4f m, ee=(%.3f, %.3f, %.3f), locked_target=(%.3f, %.3f, %.3f)", position_error,
                    ee_tf.transform.translation.x, ee_tf.transform.translation.y, ee_tf.transform.translation.z,
                    pose_to_publish.pose.position.x, pose_to_publish.pose.position.y, pose_to_publish.pose.position.z);

                if (position_error < kVisualServoConvergencePositionToleranceMeters) {
                    RCLCPP_INFO(
                        this->get_logger(), "视觉伺服收敛，位置误差 %.4f m 小于阈值 %.4f m", position_error,
                        kVisualServoConvergencePositionToleranceMeters);
                    publish_visual_servo_result(true);
                    visual_servo_active_ = false;
                    break;
                }
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "检查视觉伺服收敛时获取TF失败: %s", ex.what());
            }
        }

        if (!has_pose) {
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
        pose_to_publish.pose.orientation.z = q.z();

        if (has_pose) {
            visual_target_pub_->publish(pose_to_publish);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No object pose available for visual servo");
            publish_visual_servo_result(false);
            visual_servo_active_ = false;
            break;
        }

        rate.sleep();
    }

    publish_visual_servo_result(false);

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

void ArmTaskNode::vision_callback(const robot_interfaces::msg::Vis& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);

   geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = camera_frame_;
        tf_msg.child_frame_id  = object_frame_;

        tf_msg.transform.translation.x = msg.z;
        tf_msg.transform.translation.y = msg.y;
        tf_msg.transform.translation.z = -msg.x;


        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf_msg);
    has_visual_pose_                 = true;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500, "收到视觉消息 (相机坐标系): x=%.4f, y=%.4f, z=%.4f", msg.x, msg.y,
        msg.z);                                          // 请根据实际字段名调整
}

geometry_msgs::msg::PoseStamped ArmTaskNode::create_approach_pose(const geometry_msgs::msg::PoseStamped& target_pose, double distance) {

    geometry_msgs::msg::PoseStamped approach_pose = target_pose;
    approach_pose.pose.position.z += distance;

    return approach_pose;
}

void ArmTaskNode::execute_place_flow_rad() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(grasp_position, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));


    // 4. Deactivate air pump to release object
    RCLCPP_INFO(this->get_logger(), "打开气泵");
    robot_interfaces::msg::Armmode msg;
    msg.mode = 1;
    air_pub_->publish(msg);
    std::this_thread::sleep_for(500ms);


    RCLCPP_INFO(this->get_logger(), "移动到二次准备位置");
    execute_joint_space_trajectory(grasp_position_two, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000)));




    // 5. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "返回准备位置");
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "任务结束");
}

void ArmTaskNode::execute_place_place_rad() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));


    // 4. Deactivate air pump to release object
    RCLCPP_INFO(this->get_logger(), "关闭气泵");
    robot_interfaces::msg::Armmode msg;
    msg.mode = 0;
    air_pub_->publish(msg);
    std::this_thread::sleep_for(500ms);



    // 5. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "返回准备位置");
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000)));

    RCLCPP_INFO(this->get_logger(), "任务结束");
}



} // namespace arm_task
