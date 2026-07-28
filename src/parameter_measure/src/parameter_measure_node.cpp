#include "parameter_measure_node.hpp"

#include "record.hpp"
#include "trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

namespace {

std::chrono::high_resolution_clock::duration SecondsToDuration(double seconds)
{
    return std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
        std::chrono::duration<double>(std::max(seconds, 0.001)));
}

constexpr char kDefaultModelPath[] = "/space2/Project/RC2026_RoboticArm2/src/arm/model/robotic_arm.urdf";

}  // namespace

ParameterMeasure::ParameterMeasure(const rclcpp::Node::SharedPtr node)
    : node_(node)
{
    node_->declare_parameter<bool>("start_measure", false);
    node_->declare_parameter<std::string>("model_path", kDefaultModelPath);
    node_->declare_parameter<std::string>("joint_state_topic", "myjoints_state");
    node_->declare_parameter<std::string>("joint_target_topic", "myjoints_target");
    node_->declare_parameter<std::string>("csv_file_path", "");
    node_->declare_parameter<std::string>("trajectory_file_path", "");
    node_->declare_parameter<int>("joint_dof", 6);
    node_->declare_parameter<double>("move_to_start_duration", 3.0);
    node_->declare_parameter<double>("control_period", 0.02);
    node_->declare_parameter<int>("discard_initial_samples", 5);

    model_path_ = node_->get_parameter("model_path").as_string();
    if (model_path_.empty()) {
        model_path_ = kDefaultModelPath;
    }
    joint_state_topic_ = node_->get_parameter("joint_state_topic").as_string();
    joint_target_topic_ = node_->get_parameter("joint_target_topic").as_string();
    csv_file_path_ = node_->get_parameter("csv_file_path").as_string();
    trajectory_file_path_ = node_->get_parameter("trajectory_file_path").as_string();
    joint_dof_ = static_cast<int>(std::clamp<int64_t>(node_->get_parameter("joint_dof").as_int(), 1, 6));
    move_to_start_duration_sec_ = std::max(node_->get_parameter("move_to_start_duration").as_double(), 0.1);
    control_period_sec_ = std::max(node_->get_parameter("control_period").as_double(), 0.005);
    discard_initial_samples_ = static_cast<int>(std::max<int64_t>(node_->get_parameter("discard_initial_samples").as_int(), 0));

    latest_joint_pos_.assign(static_cast<std::size_t>(joint_dof_), 0.0F);
    latest_joint_vel_.assign(static_cast<std::size_t>(joint_dof_), 0.0F);
    latest_joint_torque_.assign(static_cast<std::size_t>(joint_dof_), 0.0F);

    joint_state_sub_ = node_->create_subscription<robot_interfaces::msg::Arm>(
        joint_state_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ParameterMeasure::jointStateCallback, this, std::placeholders::_1));
    joint_target_pub_ = node_->create_publisher<robot_interfaces::msg::Arm>(joint_target_topic_, 10);

    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ParameterMeasure::on_parameters_changed, this, std::placeholders::_1));
}

ParameterMeasure::~ParameterMeasure()
{
    exit_requested_ = true;
    state_cv_.notify_all();

    std::lock_guard<std::mutex> lock(thread_mutex_);
    if (measure_deal_thread && measure_deal_thread->joinable()) {
        measure_deal_thread->join();
    }
}

rcl_interfaces::msg::SetParametersResult ParameterMeasure::on_parameters_changed(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "start_measure" && param.as_bool() && !resetting_start_measure_) {
            start_measure_thread();
        } else if (param.get_name() == "move_to_start_duration") {
            move_to_start_duration_sec_ = std::max(param.as_double(), 0.1);
        } else if (param.get_name() == "control_period") {
            control_period_sec_ = std::max(param.as_double(), 0.005);
        } else if (param.get_name() == "csv_file_path") {
            csv_file_path_ = param.as_string();
        } else if (param.get_name() == "trajectory_file_path") {
            trajectory_file_path_ = param.as_string();
        } else if (param.get_name() == "discard_initial_samples") {
            discard_initial_samples_ = static_cast<int>(std::max<int64_t>(param.as_int(), 0));
        }
    }
    return result;
}

void ParameterMeasure::jointStateCallback(const robot_interfaces::msg::Arm::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (int i = 0; i < joint_dof_; ++i) {
        const auto index = static_cast<std::size_t>(i);
        latest_joint_pos_[index] = msg->motor[index].rad;
        latest_joint_vel_[index] = msg->motor[index].omega;
        latest_joint_torque_[index] = msg->motor[index].torque;
    }

    has_joint_state_ = true;
    state_cv_.notify_all();
}

void ParameterMeasure::start_measure_thread()
{
    if (measure_running_.exchange(true)) {
        RCLCPP_WARN(node_->get_logger(), "Parameter measurement is already running");
        return;
    }

    std::lock_guard<std::mutex> lock(thread_mutex_);
    if (measure_deal_thread && measure_deal_thread->joinable()) {
        measure_deal_thread->join();
    }
    measure_deal_thread = std::make_shared<std::thread>(&ParameterMeasure::measure_thread_func, this);
}

void ParameterMeasure::measure_thread_func()
{
    RCLCPP_INFO(node_->get_logger(), "Parameter measurement started");

    auto finish = [this]() {
        measure_running_ = false;
        resetting_start_measure_ = true;
        node_->set_parameter(rclcpp::Parameter("start_measure", false));
        resetting_start_measure_ = false;
    };

    const std::string trajectory_file_path = trajectory_file_path_;
    const double move_to_start_duration_sec = move_to_start_duration_sec_;
    const int discard_initial_samples = discard_initial_samples_;

    std::vector<float> init_pos;
    {
        std::unique_lock<std::mutex> lock(state_mutex_);
        if (!state_cv_.wait_for(lock, 3s, [this]() { return has_joint_state_ || exit_requested_.load(); })) {
            RCLCPP_ERROR(node_->get_logger(), "No joint state received before measurement");
            finish();
            return;
        }
        if (exit_requested_) {
            finish();
            return;
        }
        init_pos = latest_joint_pos_;
    }

    if (trajectory_file_path.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "trajectory_file_path is empty; generate or provide an expected trajectory CSV first");
        finish();
        return;
    }

    Trajectory trajectory(trajectory_file_path);
    if (!trajectory.load(init_pos, move_to_start_duration_sec)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load expected trajectory CSV: %s", trajectory_file_path.c_str());
        finish();
        return;
    }

    Record record;
    const std::string csv_file_path = build_csv_file_path();
    const auto start_time = std::chrono::high_resolution_clock::now();
    trajectory.start(start_time);
    const auto period = SecondsToDuration(control_period_sec_);
    const auto end_time = start_time + SecondsToDuration(trajectory.total_duration());
    const auto record_start_time = start_time + SecondsToDuration(trajectory.move_to_start_duration());

    std::vector<float> command_pos;
    bool recording_started = false;
    int discarded_samples = 0;
    while (!exit_requested_ && std::chrono::high_resolution_clock::now() <= end_time) {
        const auto now = std::chrono::high_resolution_clock::now();
        if (trajectory.sample(now, command_pos)) {
            publish_joint_target(command_pos);
        }

        if (!recording_started && now >= record_start_time) {
            if (!record.start(joint_dof_, csv_file_path, record_start_time)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to open measurement CSV: %s", csv_file_path.c_str());
                finish();
                return;
            }
            recording_started = true;
            RCLCPP_INFO(node_->get_logger(), "Started recording identification data to: %s", csv_file_path.c_str());
            std::this_thread::sleep_until(now + period);
            continue;
        }

        std::vector<float> joint_pos;
        std::vector<float> joint_vel;
        std::vector<float> joint_torque;
        if (recording_started && snapshot_joint_state(joint_pos, joint_vel, joint_torque)) {
            if (discarded_samples < discard_initial_samples) {
                ++discarded_samples;
            } else {
                record.record(now, joint_pos, joint_vel, joint_torque);
            }
        }

        std::this_thread::sleep_until(now + period);
    }

    if (!command_pos.empty()) {
        publish_joint_target(command_pos);
    }
    record.stop();
    RCLCPP_INFO(node_->get_logger(), "Parameter measurement finished, measurement CSV saved to: %s", csv_file_path.c_str());
    finish();
}

std::string ParameterMeasure::build_csv_file_path() const
{
    if (!csv_file_path_.empty()) {
        return csv_file_path_;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&now_time, &local_time);

    std::ostringstream stream;
    stream << "parameter_measure_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
    return (std::filesystem::current_path() / stream.str()).string();
}

std::vector<float> ParameterMeasure::current_joint_position() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_joint_pos_;
}

void ParameterMeasure::publish_joint_target(const std::vector<float>& joint_pos) const
{
    robot_interfaces::msg::Arm msg;
    const std::size_t count = std::min(joint_pos.size(), static_cast<std::size_t>(joint_dof_));
    for (std::size_t i = 0; i < count; ++i) {
        msg.motor[i].rad = joint_pos[i];
        msg.motor[i].omega = 0.0F;
        msg.motor[i].torque = 0.0F;
    }
    joint_target_pub_->publish(msg);
}

bool ParameterMeasure::snapshot_joint_state(std::vector<float>& joint_pos, std::vector<float>& joint_vel,
                                            std::vector<float>& joint_torque)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_joint_state_) {
        return false;
    }

    joint_pos = latest_joint_pos_;
    joint_vel = latest_joint_vel_;
    joint_torque = latest_joint_torque_;
    return true;
}
