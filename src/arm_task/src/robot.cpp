#include "robot.hpp"
#include "task/base_task.hpp"
#include "task/catch_kfs.hpp"
#include "task/idel.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

Robot::Robot(rclcpp::Node::SharedPtr node){

    node_=node;
    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    node_->declare_parameter<int32_t>("arm_task", 0);
    node_->declare_parameter<bool>("stop_visual_servo", false);
    node_->declare_parameter<double>("trajectory_duration", 3.0);
    node_->declare_parameter<double>("grasp_time", 5.0);
    node_->declare_parameter<double>("approach_distance", 0.1);
    node_->declare_parameter<double>("visual_servo_kp", 2.0);
    node_->declare_parameter<double>("visual_servo_max_linear_acc", 0.5);
    node_->declare_parameter<std::string>("base_frame", "base_link");
    node_->declare_parameter<std::string>("camera_frame", "camera_link");
    node_->declare_parameter<std::string>("object_frame", "target_object");
    node_->declare_parameter<std::string>("tip_frame", "link6");
    node_->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");

    // Get parameters
    node_->get_parameter("trajectory_duration", trajectory_duration_);
    node_->get_parameter("grasp_time", grasp_time_);
    node_->get_parameter("approach_distance", approach_distance_);
    // node_->get_parameter("visual_servo_kp", visual_servo_kp_);
    node_->get_parameter("visual_servo_max_linear_acc", visual_servo_max_linear_acc_);
    node_->get_parameter("base_frame", base_frame_);
    node_->get_parameter("camera_frame", camera_frame_);
    node_->get_parameter("object_frame", object_frame_);
    node_->get_parameter("tip_frame", tip_frame_);
    node_->get_parameter("arm_calc_node_name", arm_calc_node_name_);

    // Load arm positions from YAML
    load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = node_->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);

    // Setup parameter callback
    param_callback_ = node_->add_on_set_parameters_callback(std::bind(&Robot::on_parameters_changed, this, std::placeholders::_1));

    // Create parameter client for arm_calc node
    arm_calc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, arm_calc_node_name_);
    RCLCPP_INFO(node_->get_logger(), "Using arm_calc parameter client target: %s", arm_calc_node_name_.c_str());


    
    register_task(std::make_shared<IdelTask>(this,"idel"));
    init_task_manager("idel");

    task_thread_ = std::make_shared<std::thread>([this]() { porcess_task(); });

    RCLCPP_INFO(node_->get_logger(), "ArmTaskNode初始化完成");
}

Robot::~Robot() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down ArmTaskNode...");

}

void Robot::load_arm_positions_from_yaml() {
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("arm_task");
        std::string yaml_path     = package_share + "/config/arm_position.yaml";
        load_named_joint_positions_from_yaml(yaml_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load arm positions: %s", e.what());
    }
}

void Robot::porcess_task() {
    std::shared_ptr<BaseTask> current_task;
    std::string current_task_name;
    std::string previous_task_name;

    {
        std::unique_lock<std::mutex> lock(task_manager_mutex_);
        task_manager_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return !rclcpp::ok() || (task_manager_initialized_ && !current_task_name_.empty());
        });

        if (!rclcpp::ok()) {
            return;
        }

        auto task_it = task_table_.find(current_task_name_);
        if (task_it == task_table_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "当前任务 [%s] 未注册，任务调度暂停", current_task_name_.c_str());
            current_task_name_.clear();
            return;
        }

        current_task       = task_it->second;
        current_task_name  = current_task_name_;
        previous_task_name = last_task_name_;
    }

    std::string next_task_name;
    try {
        next_task_name = current_task->process(previous_task_name);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生异常: %s", current_task_name.c_str(), e.what());
        next_task_name.clear();
    } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "执行任务 [%s] 时发生未知异常", current_task_name.c_str());
        next_task_name.clear();
    }

    {
        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        last_task_name_ = current_task_name;

        if (next_task_name.empty()) {
            RCLCPP_INFO(node_->get_logger(), "任务 [%s] 执行完成，当前没有后续任务", current_task_name.c_str());
            current_task_name_.clear();
            return;
        }

        if (task_table_.find(next_task_name) == task_table_.end()) {
            RCLCPP_ERROR(
                node_->get_logger(), "任务 [%s] 请求切换到未注册任务 [%s]，调度保持等待", current_task_name.c_str(),
                next_task_name.c_str());
            current_task_name_.clear();
            return;
        }

        if (next_task_name != current_task_name) {
            RCLCPP_INFO(node_->get_logger(), "任务切换: [%s] -> [%s]", current_task_name.c_str(), next_task_name.c_str());
        }
        current_task_name_ = std::move(next_task_name);
    }
}

void Robot::register_task(std::shared_ptr<BaseTask> task_ptr) {
    if (!task_ptr) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: task_ptr 为空");
        return;
    }

    const std::string task_name = task_ptr->task_name;
    if (task_name.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "注册任务失败: 任务名为空");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(task_manager_mutex_);
        auto [it, inserted] = task_table_.insert_or_assign(task_name, std::move(task_ptr));
        (void)it;
        if (inserted) {
            RCLCPP_INFO(node_->get_logger(), "注册任务成功: [%s]", task_name.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "任务 [%s] 已存在，已更新为新的任务实现", task_name.c_str());
        }
    }

    task_manager_cv_.notify_all();
}

void Robot::init_task_manager(const std::string first_task_name) {
    std::lock_guard<std::mutex> lock(task_manager_mutex_);
    if (task_table_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 当前没有已注册任务");
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    std::string initial_task_name = first_task_name;
    if (initial_task_name.empty()) {
        initial_task_name = task_table_.begin()->first;
        RCLCPP_WARN(node_->get_logger(), "未指定初始任务，默认使用已注册的首个任务 [%s]", initial_task_name.c_str());
    }

    if (task_table_.find(initial_task_name) == task_table_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "初始化任务调度器失败: 初始任务 [%s] 未注册", initial_task_name.c_str());
        task_manager_initialized_ = false;
        current_task_name_.clear();
        last_task_name_.clear();
        return;
    }

    current_task_name_ = std::move(initial_task_name);
    last_task_name_.clear();
    task_manager_initialized_ = true;

    RCLCPP_INFO(node_->get_logger(), "任务调度器初始化完成，初始任务为 [%s]", current_task_name_.c_str());
    task_manager_cv_.notify_all();
}


rcl_interfaces::msg::SetParametersResult Robot::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    return result;
}



void Robot::stop_arm_motion() {
    RCLCPP_INFO(node_->get_logger(), "Stopping arm motion");
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", false)});
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

bool Robot::execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration) {
    RCLCPP_INFO(node_->get_logger(), "Executing joint space trajectory");

    // Publish joint target
    std_msgs::msg::Float64MultiArray msg;
    msg.data = joint_angles;
    joint_space_target_pub_->publish(msg);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (!arm_calc_param_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
        return false;
    }

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 1)});

    std::this_thread::sleep_for(100ms);

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});

    const auto timeout_sec = duration + 2.0;
    const auto start_time  = std::chrono::steady_clock::now();
    bool observed_running  = false;

    while (!shutdown_requested_) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(
                node_->get_logger(), "Timed out waiting for %s joint trajectory completion after %.2f s",
                arm_calc_node_name_.c_str(), timeout_sec);
            return false;
        }

        auto future = arm_calc_param_client_->get_parameters({"execute_trajectory"});
        const auto future_status = future.wait_for(200ms);
        if (future_status != std::future_status::ready) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        bool execute_trajectory = false;
        try {
            const auto result = future.get();
            if (!result.empty() && result.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                execute_trajectory = result.front().as_bool();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to query %s execute_trajectory: %s", arm_calc_node_name_.c_str(), e.what());
            std::this_thread::sleep_for(50ms);
            continue;
        }

        if (execute_trajectory) {
            observed_running = true;
        } else if (observed_running) {
            return true;
        }

        std::this_thread::sleep_for(50ms);
    }

    return false;
}

bool Robot::execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration) {

    RCLCPP_INFO(node_->get_logger(), "Executing Cartesian space trajectory");

    // Publish visual target
    visual_target_pub_->publish(target_pose);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (!arm_calc_param_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(node_->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
        return false;
    }

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 2)});

    std::this_thread::sleep_for(100ms);

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});

    const auto timeout_sec = duration + 2.0;
    const auto start_time  = std::chrono::steady_clock::now();
    bool observed_running  = false;

    while (!shutdown_requested_) {
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(
                node_->get_logger(), "Timed out waiting for %s Cartesian trajectory completion after %.2f s",
                arm_calc_node_name_.c_str(), timeout_sec);
            return false;
        }

        auto future = arm_calc_param_client_->get_parameters({"execute_trajectory"});
        const auto future_status = future.wait_for(200ms);
        if (future_status != std::future_status::ready) {
            std::this_thread::sleep_for(50ms);
            continue;
        }

        bool execute_trajectory = false;
        try {
            const auto result = future.get();
            if (!result.empty() && result.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                execute_trajectory = result.front().as_bool();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Failed to query %s execute_trajectory: %s", arm_calc_node_name_.c_str(), e.what());
            std::this_thread::sleep_for(50ms);
            continue;
        }

        if (execute_trajectory) {
            observed_running = true;
        } else if (observed_running) {
            return true;
        }

        std::this_thread::sleep_for(50ms);
    }

    return false;
}

void Robot::execute_visual_servo(const geometry_msgs::msg::Twist& velocity)
{

}

bool Robot::set_air_pump(const bool &enable) {
    if (arm_calc_param_client_->wait_for_service(std::chrono::duration<double>(0.5))) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("enable_air_pump", enable)});
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "%s 的参数服务不可用", arm_calc_node_name_.c_str());
        return false;
    }
}

bool Robot::get_named_joint_position(const std::string& name, std::vector<double>& joint_angles) const {
    const auto it = arm_positions_.find(name);
    if (it == arm_positions_.end()) {
        return false;
    }

    joint_angles = it->second;
    return true;
}

void Robot::load_named_joint_positions_from_yaml(const std::string& yaml_path) {
    YAML::Node config = YAML::LoadFile(yaml_path);

    arm_positions_.clear();
    ready_position_.clear();

    if (config["arm_positions"]) {
        const YAML::Node arm_positions_node = config["arm_positions"];

        if (arm_positions_node.IsMap()) {
            for (const auto& entry : arm_positions_node) {
                const std::string name = entry.first.as<std::string>();
                const std::vector<double> joints = entry.second.as<std::vector<double>>();
                arm_positions_[name] = joints;
                RCLCPP_INFO(node_->get_logger(), "Loaded named position [%s] with %zu joints", name.c_str(), joints.size());
            }
        } else if (arm_positions_node.IsSequence()) {
            for (const auto& pos : arm_positions_node) {
                if (!pos["name"] || !pos["joints"]) {
                    RCLCPP_WARN(node_->get_logger(), "Skip invalid arm position entry in %s", yaml_path.c_str());
                    continue;
                }

                const std::string name = pos["name"].as<std::string>();
                const std::vector<double> joints = pos["joints"].as<std::vector<double>>();
                arm_positions_[name] = joints;
                RCLCPP_INFO(node_->get_logger(), "Loaded named position [%s] with %zu joints", name.c_str(), joints.size());
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Node [arm_positions] in %s is neither a map nor a sequence", yaml_path.c_str());
        }
    }
}
