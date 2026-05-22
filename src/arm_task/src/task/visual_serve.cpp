#include "robot.hpp"
#include <task/visual_serve.hpp>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.h>
#include <chrono>
#include <string>
#include <thread>
#include <cmath>

VisualServe::VisualServe(Robot* context, const std::string name)
	: BaseTask(context, name) {}

VisualServe::~VisualServe() {}


std::string VisualServe::process(const std::string /*last_task_name*/) {
	// Try to obtain active action context if present
	Robot::ActiveTaskContext context;
	const bool has_action_context = robot->get_active_task_context(context);
	const auto goal_handle = has_action_context ? context.goal_handle : nullptr;

	auto fail_task = [&](const std::string& msg) {
		if (goal_handle) {
			robot->finish_current_task(goal_handle, false, msg);
		}
		return std::string("idel");
	};

	auto stop_visual_servo_if_needed = [&]() {
		robot->stop_visual_servo();
	};

	// 1. Lookup object pose in base frame via TF
	geometry_msgs::msg::PoseStamped object_pose;
	try {
		if (!robot->tf_buffer_->canTransform(robot->base_frame_, robot->object_frame_, tf2::TimePointZero, std::chrono::seconds(2))) {
			RCLCPP_ERROR(robot->node_->get_logger(), "等待 TF %s -> %s 超时", robot->base_frame_.c_str(), robot->object_frame_.c_str());
			return fail_task("TF timeout");
		}

		const geometry_msgs::msg::TransformStamped target_tf =
			robot->tf_buffer_->lookupTransform(robot->base_frame_, robot->object_frame_, tf2::TimePointZero);

		object_pose.header.frame_id = robot->base_frame_;
		object_pose.header.stamp = robot->node_->now();
		object_pose.pose.position.x = target_tf.transform.translation.x;
		object_pose.pose.position.y = -(target_tf.transform.translation.y + 0.05);
		object_pose.pose.position.z = target_tf.transform.translation.z;

	} catch (const tf2::TransformException& ex) {
		RCLCPP_ERROR(robot->node_->get_logger(), "读取 TF 目标失败: %s", ex.what());
		return fail_task("TF lookup failed");
	}

	// Force a reasonable end-effector orientation (match visual_serve_test.cpp)
	tf2::Quaternion quat;
	quat.setRPY(0.0, M_PI / 2.0, 0.0);
	object_pose.pose.orientation.w = quat.getW();
	object_pose.pose.orientation.x = quat.getX();
	object_pose.pose.orientation.y = quat.getY();
	object_pose.pose.orientation.z = quat.getZ();

	// 2. Start visual servo using Robot API and monitor convergence/cancel state
	RCLCPP_INFO(robot->node_->get_logger(), "启动视觉伺服至目标 (%.3f, %.3f, %.3f)", object_pose.pose.position.x,
				object_pose.pose.position.y, object_pose.pose.position.z);

	// Read optional parameters from node (override defaults)
	try {
		rclcpp::Parameter p;
		if (robot->node_->get_parameter("vs_timeout_unlock", p)) {
			vs_timeout_unlock_ = p.as_double();
		}
		if (robot->node_->get_parameter("vs_timeout_lock", p)) {
			vs_timeout_lock_ = p.as_double();
		}
		if (robot->node_->get_parameter("vs_monitor_interval_ms", p)) {
			vs_monitor_interval_ms_ = p.as_int();
		}
	} catch (...) {
		// ignore parameter read failures and use defaults
	}

	robot->execute_visual_servo(object_pose);

	const double kPositionTolerance = kVisualServoConvergencePositionToleranceMeters;

	const auto start_time = std::chrono::steady_clock::now();
	const auto unlock_deadline = start_time + std::chrono::duration<double>(vs_timeout_unlock_);

	bool converged = false;
	bool camera_locked = false;
	std::chrono::steady_clock::time_point lock_start;
	double min_distance_seen = std::numeric_limits<double>::infinity();

	while (rclcpp::ok()) {
		// Query current distance via Robot helper
		double current_distance = 0.0;
		robot->is_visual_servo_converged(kPositionTolerance, &current_distance);
		if (current_distance > 0.0 && current_distance < min_distance_seen) {
			min_distance_seen = current_distance;
		}

		// Check convergence
		if (robot->is_visual_servo_converged(kPositionTolerance, &current_distance)) {
			RCLCPP_INFO(robot->node_->get_logger(), "视觉伺服已收敛，当前误差 %.4f m", current_distance);
			converged = true;
			break;
		}

		// Check active state
		if (!robot->is_visual_servo_active()) {
			RCLCPP_WARN(robot->node_->get_logger(), "视觉伺服已被外部取消或提前结束");
			break;
		}

		// Detect camera lock (distance from camera to object)
		bool now_locked = is_camera_data_locked();
		if (now_locked && !camera_locked) {
			camera_locked = true;
			lock_start = std::chrono::steady_clock::now();
			RCLCPP_INFO(robot->node_->get_logger(), "camera_data_locked=true (进入已锁定阶段)");
		}

		const auto now = std::chrono::steady_clock::now();
		if (!camera_locked && now >= unlock_deadline) {
			RCLCPP_WARN(robot->node_->get_logger(), "未能在 %.1f s 内锁定相机数据，停止视觉伺服", vs_timeout_unlock_);
			break;
		}

		if (camera_locked) {
			if (now - lock_start >= std::chrono::duration<double>(vs_timeout_lock_)) {
				RCLCPP_WARN(robot->node_->get_logger(), "已锁定但在 %.1f s 内未收敛到 %.3f m，停止视觉伺服", vs_timeout_lock_, kPositionTolerance);
				break;
			}
		}

		// Throttled monitoring log
		RCLCPP_INFO_THROTTLE(robot->node_->get_logger(), *robot->node_->get_clock(), 500,
							 "视觉伺服监测: dist=%.4f m, min=%.4f m, locked=%d", current_distance, min_distance_seen,
							 static_cast<int>(camera_locked));

		std::this_thread::sleep_for(std::chrono::milliseconds(vs_monitor_interval_ms_));
	}

	if (!converged) {
		const bool was_active = robot->is_visual_servo_active();
		stop_visual_servo_if_needed();
		return fail_task(was_active ? "visual servo timeout" : "visual servo canceled or not converged");
	}

	stop_visual_servo_if_needed();

	// Optional: enable pump / mark grasped if this task is meant to grasp
	if (!robot->set_air_pump(true)) {
		RCLCPP_WARN(robot->node_->get_logger(), "启动气泵失败");
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	if (goal_handle) {
		robot->finish_current_task(goal_handle, true, "视觉伺服完成");
	}

	return "idel";
}

bool VisualServe::is_camera_data_locked() {
	try {
		const auto tf = robot->tf_buffer_->lookupTransform(robot->camera_frame_, robot->object_frame_, tf2::TimePointZero,
															tf2::durationFromSec(0.1));
		const auto& t = tf.transform.translation;
		const double dist = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
		return dist < kCameraDataLockDistanceMeters;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_THROTTLE(robot->node_->get_logger(), *robot->node_->get_clock(), 1000, "检测 camera->object TF 失败: %s",
							 ex.what());
		return false;
	}
}

bool VisualServe::query_current_distance(double* out_distance_m) {
	if (!out_distance_m) return false;
	// Reuse Robot's helper which computes distance between target and end effector
	return robot->is_visual_servo_converged(kVisualServoConvergencePositionToleranceMeters, out_distance_m);
}


