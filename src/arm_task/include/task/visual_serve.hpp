#pragma once

#include "task/base_task.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>


class VisualServe : public BaseTask {
public:
    VisualServe(Robot* context, const std::string name);
    ~VisualServe() override;
    std::string process(const std::string last_task_name) override;

private:
    // Parameters (can be overridden via node parameters)
    double vs_timeout_unlock_{15.0}; // seconds waiting for camera lock
    double vs_timeout_lock_{20.0};   // seconds waiting for convergence after lock
    int vs_monitor_interval_ms_{100};

    // Thresholds mirror Robot implementation
    static constexpr double kCameraDataLockDistanceMeters = 0.35;
    static constexpr double kVisualServoConvergencePositionToleranceMeters = 0.05;

    // Helper: try to query camera->object distance to detect lock
    bool is_camera_data_locked();
    // Helper: query current end-effector to target distance via Robot API
    bool query_current_distance(double* out_distance_m);
};
