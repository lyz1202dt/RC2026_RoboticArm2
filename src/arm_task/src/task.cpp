#include "arm_task/task.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace arm_task
{

namespace
{

constexpr char kNodeName[] = "arm_task";
constexpr char kArmCmdTopic[] = "arm_cmd";
constexpr char kVisionTopic[] = "pnp_move";
constexpr char kPlaceTargetTopic[] = "place_target_pose";
constexpr char kTaskParameter[] = "arm_task";
constexpr char kPresetPositionIdParameter[] = "preset_position_id";

constexpr char kBaseFrame[] = "base_link";
constexpr char kCameraLeftFrame[] = "camera_left_link";
constexpr char kCameraRightFrame[] = "camera_right_link";
constexpr char kAirPumpNode[] = "arm_node";
constexpr char kAirPumpParameter[] = "enable_air_pump";

constexpr int32_t kTaskStandby = 0;
constexpr int32_t kTaskLeftGrasp = 1;
constexpr int32_t kTaskRightGrasp = 2;
constexpr int32_t kTaskLeftPlace = 3;
constexpr int32_t kTaskRightPlace = 4;
constexpr int32_t kTaskFixedRelease = 5;
constexpr int32_t kTaskMovePreset = 10;

constexpr int32_t kMotionStop = 0;
constexpr int32_t kMotionJointSpace = 1;
constexpr int32_t kMotionCartesianSpace = 2;

constexpr std::size_t kCommandValueCount = 4;
constexpr double kTrajectoryDurationSec = 3.0;
constexpr double kTaskCompletionMarginSec = 0.5;
constexpr double kVisionStabilizationSec = 2.0;
constexpr double kVisualTargetFixedZ = -0.23;
constexpr double kDefaultCartesianPitchRad = -1.57079632679489661923;

bool load_vector_if_present(
  const YAML::Node & config, const char * key,
  std::vector<double> & output)
{
  if (!config[key]) {
    return false;
  }

  const auto values = config[key].as<std::vector<double>>();
  if (values.size() != kCommandValueCount) {
    throw std::runtime_error(std::string(key) + " must contain exactly 4 values");
  }

  output = values;
  return true;
}

} // namespace

ArmTaskNode::ArmTaskNode(const rclcpp::NodeOptions & options)
: Node(kNodeName, options)
{
  RCLCPP_INFO(get_logger(), "Initializing ArmTaskNode");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter<int32_t>(kTaskParameter, kTaskStandby);
  declare_parameter<int>(kPresetPositionIdParameter, 0);
  preset_position_id_ = get_parameter(kPresetPositionIdParameter).as_int();

  load_arm_positions_from_yaml();

  arm_cmd_pub_ = create_publisher<robot_interfaces::msg::ArmCmd>(kArmCmdTopic, 10);
  vision_sub_ = create_subscription<robot_interfaces::msg::Vis>(
    kVisionTopic, 10, std::bind(&ArmTaskNode::vision_callback, this, std::placeholders::_1));
  place_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    kPlaceTargetTopic, 10, std::bind(
      &ArmTaskNode::on_place_target_pose, this,
      std::placeholders::_1));

  param_callback_ = add_on_set_parameters_callback(
    std::bind(&ArmTaskNode::on_parameters_changed, this, std::placeholders::_1));

  task_thread_ = std::thread(&ArmTaskNode::task_execution_thread, this);

  RCLCPP_INFO(get_logger(), "ArmTaskNode initialized");
}

ArmTaskNode::~ArmTaskNode()
{
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    shutdown_requested_ = true;
  }
  task_cv_.notify_all();

  if (task_thread_.joinable()) {
    task_thread_.join();
  }
}

void ArmTaskNode::load_arm_positions_from_yaml()
{
  try {
    const std::string package_share = ament_index_cpp::get_package_share_directory("arm_task");
    const std::string yaml_path = package_share + "/config/arm_position.yaml";
    const YAML::Node config = YAML::LoadFile(yaml_path);

        //load_vector_if_present(config, "ready_position", ready_position_);
        load_vector_if_present(config, "home_position", home_position_);
        load_vector_if_present(config, "place_position", place_position_);

    if (config["arm_positions"]) {
      for (const auto & pos : config["arm_positions"]) {
        const int index = pos["index"].as<int>();
        const auto joints = pos["joints"].as<std::vector<double>>();
        if (joints.size() != kCommandValueCount) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring preset position %d: expected %zu joints, got %zu",
            index,
            kCommandValueCount,
            joints.size());
          continue;
        }
        arm_positions_[index] = joints;
        RCLCPP_INFO(get_logger(), "Loaded preset position %d", index);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load arm positions: %s", e.what());
  }
}

void ArmTaskNode::task_execution_thread()
{
  RCLCPP_INFO(get_logger(), "Arm task execution thread started");

  while (rclcpp::ok()) {
    int32_t task_mode = kTaskStandby;
    int preset_position_id = 0;

    {
      std::unique_lock<std::mutex> lock(task_mutex_);
      task_cv_.wait(
        lock, [this]() {
          return shutdown_requested_ || requested_task_mode_ != kTaskStandby;
        });

      if (shutdown_requested_) {
        break;
      }

      task_mode = requested_task_mode_;
      preset_position_id = preset_position_id_;
      requested_task_mode_ = kTaskStandby;
      active_task_mode_ = task_mode;
      task_running_ = true;
    }

    try {
      execute_task_state_machine(task_mode, preset_position_id);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Task execution failed: %s", e.what());
    }

    active_task_mode_ = kTaskStandby;
    reset_task_parameter();
    mark_task_finished();
  }

  RCLCPP_INFO(get_logger(), "Arm task execution thread stopped");
}

void ArmTaskNode::execute_task_state_machine(int32_t task_mode, int preset_position_id)
{
  switch (task_mode) {
    case kTaskLeftGrasp:
      RCLCPP_INFO(get_logger(), "Starting left grasp task");
      execute_grasp_flow(ArmSide::kLeft);
      break;
    case kTaskRightGrasp:
      RCLCPP_INFO(get_logger(), "Starting right grasp task");
      execute_grasp_flow(ArmSide::kRight);
      break;
    case kTaskLeftPlace:
      RCLCPP_INFO(get_logger(), "Starting left place task");
      execute_place_flow(ArmSide::kLeft);
      break;
    case kTaskRightPlace:
      RCLCPP_INFO(get_logger(), "Starting right place task");
      execute_place_flow(ArmSide::kRight);
      break;
    case kTaskFixedRelease:
      RCLCPP_INFO(get_logger(), "Starting fixed release task");
      execute_fixed_release_flow();
      break;
    case kTaskMovePreset:
      RCLCPP_INFO(get_logger(), "Moving to preset position %d", preset_position_id);
      execute_move_to_position(preset_position_id);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Ignoring unsupported task mode: %d", task_mode);
      break;
  }
}

void ArmTaskNode::execute_grasp_flow(ArmSide side) {
    RCLCPP_INFO(get_logger(), "Moving %s arm to ready position", side_name(side));
    execute_joint_space_trajectory(side, ready_position);
    std::this_thread::sleep_for(std::chrono::duration<double>(kTrajectoryDurationSec + kTaskCompletionMarginSec));

  std::this_thread::sleep_for(std::chrono::duration<double>(kVisionStabilizationSec));

  geometry_msgs::msg::PoseStamped object_pose;
  int retry_count = 0;
  while (!get_latest_object_pose(object_pose) && retry_count < 20) {
    std::this_thread::sleep_for(100ms);
    ++retry_count;
  }

  if (retry_count >= 20) {
    RCLCPP_ERROR(get_logger(), "No valid visual target received for grasp");
    execute_joint_space_trajectory(side, home_position_);
    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        kTrajectoryDurationSec +
        kTaskCompletionMarginSec));
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Visual target in base frame: [%.3f, %.3f, %.3f]",
    object_pose.pose.position.x,
    object_pose.pose.position.y,
    object_pose.pose.position.z);

  const auto approach_pose = create_approach_pose(object_pose, 0.0);
  execute_cartesian_space_trajectory(side, approach_pose);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));

  set_air_pump_enabled(true);
  std::this_thread::sleep_for(500ms);

  execute_joint_space_trajectory(side, home_position_);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));

  RCLCPP_INFO(get_logger(), "%s grasp task completed", side_name(side));
}

void ArmTaskNode::execute_place_flow(ArmSide side) {
    RCLCPP_INFO(get_logger(), "Moving %s arm to ready position", side_name(side));
    execute_joint_space_trajectory(side, ready_position);
    std::this_thread::sleep_for(std::chrono::duration<double>(kTrajectoryDurationSec + kTaskCompletionMarginSec));

  geometry_msgs::msg::PoseStamped place_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!has_place_target_) {
      RCLCPP_ERROR(get_logger(), "No place target pose has been received");
      return;
    }
    place_pose = place_target_pose_;
  }

  RCLCPP_INFO(
    get_logger(),
    "Place target: [%.3f, %.3f, %.3f]",
    place_pose.pose.position.x,
    place_pose.pose.position.y,
    place_pose.pose.position.z);

  execute_cartesian_space_trajectory(side, place_pose);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));

  set_air_pump_enabled(false);
  std::this_thread::sleep_for(500ms);

  execute_joint_space_trajectory(side, home_position_);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));

  RCLCPP_INFO(get_logger(), "%s place task completed", side_name(side));
}

void ArmTaskNode::execute_move_to_position(int preset_position_id)
{
  const auto iter = arm_positions_.find(preset_position_id);
  if (iter == arm_positions_.end()) {
    RCLCPP_ERROR(
      get_logger(), "Preset position %d not found in arm_position.yaml", preset_position_id);
    return;
  }

  execute_joint_space_trajectory(ArmSide::kLeft, iter->second);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));
}

void ArmTaskNode::execute_fixed_release_flow()
{
  execute_joint_space_trajectory(ArmSide::kLeft, place_position_);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));

  set_air_pump_enabled(false);
  std::this_thread::sleep_for(500ms);

  execute_joint_space_trajectory(ArmSide::kLeft, home_position_);
  std::this_thread::sleep_for(
    std::chrono::duration<double>(
      kTrajectoryDurationSec +
      kTaskCompletionMarginSec));
}

void ArmTaskNode::execute_joint_space_trajectory(
  ArmSide side,
  const std::vector<double> & joint_angles)
{
  publish_arm_command(side, kMotionJointSpace, joint_angles, kTrajectoryDurationSec);
}

void ArmTaskNode::execute_cartesian_space_trajectory(
    ArmSide side,
    const geometry_msgs::msg::PoseStamped& target_pose)
{
    std::vector<double> position;
    position.reserve(kCommandValueCount);

    position.push_back(target_pose.pose.position.x);
    position.push_back(target_pose.pose.position.y);
    position.push_back(target_pose.pose.position.z);

    const double pitch =
        (side == ArmSide::kLeft)
            ? kDefaultCartesianPitchRad
            : -kDefaultCartesianPitchRad;

    position.push_back(pitch);

    publish_arm_command(
        side,
        kMotionCartesianSpace,
        position,
        kTrajectoryDurationSec);
}

void ArmTaskNode::stop_arm_motion(ArmSide side)
{
  publish_arm_command(
    side, kMotionStop, std::vector<double>(
      kCommandValueCount,
      0.0), kTrajectoryDurationSec);
}

void ArmTaskNode::publish_arm_command(
  ArmSide side,
  int32_t motion_mode,
  const std::vector<double> & position,
  double duration_sec)
{
  if (motion_mode != kMotionStop && position.size() < kCommandValueCount) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to publish ArmCmd: mode %d requires at least %zu values, got %zu",
      motion_mode,
      kCommandValueCount,
      position.size());
    return;
  }

  robot_interfaces::msg::ArmCmd msg;
  msg.stamp = now();
  msg.arm_id = static_cast<int32_t>(side);
  msg.mode = motion_mode;
  msg.duration = static_cast<float>(std::max(duration_sec, 1e-3));
  msg.position.data = position;

  arm_cmd_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(),
    "Published ArmCmd: arm=%s mode=%d duration=%.2f",
    side_name(side),
    msg.mode,
    static_cast<double>(msg.duration));
}

bool ArmTaskNode::get_latest_object_pose(geometry_msgs::msg::PoseStamped & pose_out)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!has_visual_pose_) {
    return false;
  }

  pose_out = latest_visual_pose_;
  pose_out.pose.position.z = kVisualTargetFixedZ;
  return true;
}

geometry_msgs::msg::PoseStamped ArmTaskNode::create_approach_pose(
  const geometry_msgs::msg::PoseStamped & target_pose,
  double distance) const
{
  auto approach_pose = target_pose;
  approach_pose.pose.position.z += distance;
  return approach_pose;
}

void ArmTaskNode::set_air_pump_enabled(bool enabled)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, kAirPumpNode);
  if (!param_client->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(), "Air pump parameter service '%s' is not available", kAirPumpNode);
    return;
  }

  param_client->set_parameters({rclcpp::Parameter(kAirPumpParameter, enabled)});
  RCLCPP_INFO(get_logger(), "Set air pump %s", enabled ? "on" : "off");
}

void ArmTaskNode::reset_task_parameter()
{
  try {
    set_parameter(rclcpp::Parameter(kTaskParameter, kTaskStandby));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Failed to reset %s parameter: %s", kTaskParameter, e.what());
  }
}

void ArmTaskNode::mark_task_finished()
{
  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_running_ = false;
  }
  task_cv_.notify_all();
}

void ArmTaskNode::vision_callback(const robot_interfaces::msg::Vis & msg)
{
  const int32_t mode = active_task_mode_;
  const char * camera_frame = nullptr;

  if (mode == kTaskLeftGrasp || mode == kTaskLeftPlace) {
    camera_frame = kCameraLeftFrame;
  } else if (mode == kTaskRightGrasp || mode == kTaskRightPlace) {
    camera_frame = kCameraRightFrame;
  } else {
    return;
  }

  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header.stamp = now();
  camera_pose.header.frame_id = camera_frame;
  camera_pose.pose.position.x = msg.x;
  camera_pose.pose.position.y = msg.y;
  camera_pose.pose.position.z = msg.z;
  camera_pose.pose.orientation.w = 1.0;

  try {
    const auto transform = tf_buffer_->lookupTransform(
      kBaseFrame, camera_frame, tf2::TimePointZero, tf2::durationFromSec(
        0.05));

    geometry_msgs::msg::PoseStamped base_pose;
    tf2::doTransform(camera_pose, base_pose, transform);
    base_pose.header.stamp = now();
    base_pose.header.frame_id = kBaseFrame;
    base_pose.pose.position.z = kVisualTargetFixedZ;

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      latest_visual_pose_ = base_pose;
      has_visual_pose_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "Vision target transformed to base: [%.4f, %.4f, %.4f]",
      base_pose.pose.position.x,
      base_pose.pose.position.y,
      base_pose.pose.position.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Failed to transform vision target from %s to %s: %s",
      camera_frame,
      kBaseFrame,
      ex.what());
  }
}

void ArmTaskNode::on_place_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  place_target_pose_ = *msg;
  has_place_target_ = true;
  RCLCPP_INFO(get_logger(), "Received place target pose");
}

rcl_interfaces::msg::SetParametersResult ArmTaskNode::on_parameters_changed(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(task_mutex_);
  for (const auto & param : params) {
    if (param.get_name() == kPresetPositionIdParameter) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "preset_position_id must be an integer";
        return result;
      }
      preset_position_id_ = param.as_int();
      RCLCPP_INFO(get_logger(), "Preset position id changed to %d", preset_position_id_);
    } else if (param.get_name() == kTaskParameter) {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "arm_task must be an integer";
        return result;
      }

      const int32_t task_mode = static_cast<int32_t>(param.as_int());
      if (!is_supported_task_mode(task_mode)) {
        result.successful = false;
        result.reason = "unsupported arm_task mode";
        return result;
      }

      if (task_running_ && task_mode != kTaskStandby) {
        result.successful = false;
        result.reason = "arm_task is already running";
        return result;
      }

      requested_task_mode_ = task_mode;
      RCLCPP_INFO(get_logger(), "Requested task mode changed to %d", requested_task_mode_);
    }
  }

  if (requested_task_mode_ != kTaskStandby) {
    task_cv_.notify_all();
  }

  return result;
}

bool ArmTaskNode::is_supported_task_mode(int32_t task_mode)
{
  return task_mode == kTaskStandby || task_mode == kTaskLeftGrasp || task_mode == kTaskRightGrasp ||
         task_mode == kTaskLeftPlace || task_mode == kTaskRightPlace ||
         task_mode == kTaskFixedRelease ||
         task_mode == kTaskMovePreset;
}

const char * ArmTaskNode::side_name(ArmSide side)
{
  return side == ArmSide::kLeft ? "left" : "right";
}

} // namespace arm_task
