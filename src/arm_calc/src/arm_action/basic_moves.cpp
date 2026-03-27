#include "arm_action/basic_moves.hpp"

#include <algorithm>
#include <cmath>

namespace arm_action {

namespace {

JointVector QuaternionToRotationVector(const Eigen::Quaterniond& start, const Eigen::Quaterniond& goal) {
    Eigen::Quaterniond delta = arm_calc::NormalizeQuaternion(start).conjugate() * arm_calc::NormalizeQuaternion(goal);
    if (delta.w() < 0.0) {
        delta.coeffs() *= -1.0;
    }
    Eigen::AngleAxisd angle_axis(delta);
    JointVector vector = JointVector::Zero();
    vector.head<3>() = angle_axis.axis() * angle_axis.angle();
    return vector;
}

Eigen::Quaterniond RotationVectorToQuaternion(const Eigen::Quaterniond& reference, const Eigen::Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < 1e-9) {
        return arm_calc::NormalizeQuaternion(reference);
    }
    const Eigen::Vector3d axis = rotation_vector / angle;
    return arm_calc::NormalizeQuaternion(reference * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)));
}

Eigen::Vector3d ClampVectorNorm(const Eigen::Vector3d& vector, double limit) {
    if (limit <= 0.0) {
        return Eigen::Vector3d::Zero();
    }
    const double norm = vector.norm();
    if (norm <= limit || norm < 1e-9) {
        return vector;
    }
    return vector * (limit / norm);
}

}  // namespace

JointSpaceMove::JointSpaceMove() = default;

void JointSpaceMove::set_start_state(const JointState& state) {
    start_state_ = state;
    trajectory_.set_start_state(state.position, state.velocity);
}

void JointSpaceMove::set_goal_state(const JointState& state, double duration) {
    goal_state_ = state;
    duration_sec_ = std::max(duration, 1e-3);
    trajectory_.set_goal_state(state.position, duration_sec_, state.velocity);
}

void JointSpaceMove::start(double start_time_sec) {
    start_time_sec_ = start_time_sec;
    started_ = true;
}

JointTrajectoryPoint JointSpaceMove::sample(double current_time_sec) const {
    if (!started_) {
        JointTrajectoryPoint point;
        point.position = start_state_.position;
        point.velocity = start_state_.velocity;
        point.torque = start_state_.torque;
        return point;
    }

    JointTrajectoryPoint point = trajectory_.sample(current_time_sec - start_time_sec_);
    point.torque = goal_state_.torque;
    return point;
}

bool JointSpaceMove::active(double current_time_sec) const {
    return started_ && (current_time_sec - start_time_sec_) <= duration_sec_;
}

bool JointSpaceMove::started() const {
    return started_;
}

JCartesianSpaceMove::JCartesianSpaceMove(std::shared_ptr<ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void JCartesianSpaceMove::set_start_state(const JointState& state) {
    start_joint_state_ = state;
    if (arm_calc_) {
        start_cartesian_state_ = arm_calc_->end_state(state.position, state.velocity);
    }

    JointVector pos_start = JointVector::Zero();
    pos_start.head<3>() = start_cartesian_state_.pose.position;
    position_trajectory_.set_start_state(pos_start);

    JointVector rot_start = JointVector::Zero();
    orientation_trajectory_.set_start_state(rot_start);
}

void JCartesianSpaceMove::set_goal_state(const CartesianPose& pose, double duration) {
    goal_pose_ = pose;
    duration_sec_ = std::max(duration, 1e-3);

    JointVector pos_goal = JointVector::Zero();
    pos_goal.head<3>() = pose.position;
    position_trajectory_.set_goal_state(pos_goal, duration_sec_);

    JointVector rot_goal = QuaternionToRotationVector(start_cartesian_state_.pose.orientation, goal_pose_.orientation);
    orientation_trajectory_.set_goal_state(rot_goal, duration_sec_);
}

void JCartesianSpaceMove::start(double start_time_sec) {
    start_time_sec_ = start_time_sec;
    started_ = true;
}

JointTrajectoryPoint JCartesianSpaceMove::sample(double current_time_sec) {
    if (!started_ || !arm_calc_) {
        JointTrajectoryPoint point;
        point.position = start_joint_state_.position;
        point.velocity = start_joint_state_.velocity;
        return point;
    }
    return arm_calc_->signal_arm_calc(build_cartesian_target(current_time_sec));
}

bool JCartesianSpaceMove::active(double current_time_sec) const {
    return started_ && (current_time_sec - start_time_sec_) <= duration_sec_;
}

bool JCartesianSpaceMove::started() const {
    return started_;
}

CartesianTrajectoryPoint JCartesianSpaceMove::build_cartesian_target(double current_time_sec) const {
    const double elapsed = std::clamp(current_time_sec - start_time_sec_, 0.0, duration_sec_);
    const JointTrajectoryPoint pos_sample = position_trajectory_.sample(elapsed);
    const JointTrajectoryPoint rot_sample = orientation_trajectory_.sample(elapsed);

    CartesianTrajectoryPoint target;
    target.pose.position = pos_sample.position.head<3>();
    target.linear_velocity = pos_sample.velocity.head<3>();
    target.linear_acceleration = pos_sample.acceleration.head<3>();

    const Eigen::Vector3d rotation_vector = rot_sample.position.head<3>();
    target.pose.orientation = RotationVectorToQuaternion(start_cartesian_state_.pose.orientation, rotation_vector);
    target.angular_velocity = rot_sample.velocity.head<3>();
    target.angular_acceleration = rot_sample.acceleration.head<3>();
    return target;
}

VisualServoMove::VisualServoMove(std::shared_ptr<ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void VisualServoMove::set_current_joint_state(const JointState& state) {
    current_joint_state_ = state;
    has_joint_state_ = true;
    if (arm_calc_) {
        current_cartesian_state_ = arm_calc_->end_state(state.position, state.velocity);
    }
}

void VisualServoMove::set_target_pose(const CartesianPose& pose) {
    target_pose_ = pose;
    has_target_pose_ = true;
}

void VisualServoMove::set_kp(double kp) {
    kp_ = std::max(kp, 0.0);
}

void VisualServoMove::set_max_linear_acceleration(double max_linear_acceleration) {
    max_linear_acceleration_ = std::max(max_linear_acceleration, 0.0);
}

JointTrajectoryPoint VisualServoMove::sample(double current_time_sec) {
    JointTrajectoryPoint point;
    if (!arm_calc_ || !has_joint_state_) {
        point.position = current_joint_state_.position;
        point.velocity = current_joint_state_.velocity;
        point.torque = current_joint_state_.torque;
        return point;
    }

    current_cartesian_state_ = arm_calc_->end_state(current_joint_state_.position, current_joint_state_.velocity);

    if (!servo_initialized_) {
        desired_position_ = current_cartesian_state_.pose.position;
        desired_velocity_.setZero();
        desired_acceleration_.setZero();
        last_sample_time_sec_ = current_time_sec;
        servo_initialized_ = true;
    }

    if (!has_target_pose_) {
        point.position = current_joint_state_.position;
        point.velocity = current_joint_state_.velocity;
        point.torque = current_joint_state_.torque;
        return point;
    }

    const double dt = std::clamp(current_time_sec - last_sample_time_sec_, 1e-4, 0.1);
    const Eigen::Vector3d last_desired_velocity = desired_velocity_;

    Eigen::Vector3d commanded_velocity = (target_pose_.position - desired_position_) * kp_;
    desired_acceleration_ = ClampVectorNorm((commanded_velocity - last_desired_velocity) / dt, max_linear_acceleration_);
    desired_velocity_ = last_desired_velocity + desired_acceleration_ * dt;
    desired_position_ = desired_position_ + desired_velocity_ * dt;
    last_sample_time_sec_ = current_time_sec;

    CartesianTrajectoryPoint cartesian_target;
    cartesian_target.pose.position = desired_position_;
    cartesian_target.linear_velocity = desired_velocity_;
    cartesian_target.linear_acceleration = desired_acceleration_;
    cartesian_target.pose.orientation = target_pose_.orientation;
    cartesian_target.angular_velocity.setZero();
    cartesian_target.angular_acceleration.setZero();

    return arm_calc_->signal_arm_calc(cartesian_target);
}

}  // namespace arm_action
