#include "arm_action/cartesian_space_move.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace arm_action {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

CartesianSpaceMove::CartesianSpaceMove(std::shared_ptr<arm_calc::ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void CartesianSpaceMove::start(
    arm_calc::ArmSide side,
    const arm_calc::JointPosition& start_joint_position,
    const arm_calc::CartesianTarget& goal_target,
    double duration,
    double start_time_sec) {
    side_ = side;
    duration_sec_ = std::max(duration, 1e-3);
    start_time_sec_ = start_time_sec;

    arm_calc::CartesianTarget start_target;
    if (arm_calc_) {
        const arm_calc::CartesianPose start_pose = arm_calc_->end_pose(side_, start_joint_position);
        start_target.position = start_pose.position;
        start_target.pitch = pitch_from_pose(
            start_pose,
            start_joint_position[static_cast<int>(arm_calc::kArmJointDof - 1)]);
        arm_calc_->set_last_joint_pos(side_, start_joint_position);
    } else {
        start_target.pitch = start_joint_position[static_cast<int>(arm_calc::kArmJointDof - 1)];
    }

    arm_calc::CartesianTarget normalized_goal = goal_target;
    normalized_goal.pitch = normalize_pitch_near(goal_target.pitch, start_target.pitch);

    position_trajectory_.reset(
        position_to_vector(start_target.position),
        position_to_vector(normalized_goal.position),
        duration_sec_);
    pitch_trajectory_.reset(
        pitch_to_vector(start_target.pitch),
        pitch_to_vector(normalized_goal.pitch),
        duration_sec_);
    started_ = true;
}

arm_calc::JointPosition CartesianSpaceMove::sample(
    double current_time_sec,
    const arm_calc::JointPosition& seed_joint_position,
    bool* ik_ok) {
    if (ik_ok) {
        *ik_ok = true;
    }
    if (!started_ || !arm_calc_) {
        return seed_joint_position;
    }

    const arm_calc::CartesianTarget target = sample_target(current_time_sec - start_time_sec_);
    int result = -1;
    const arm_calc::JointPosition solved = arm_calc_->joint_pos(side_, target, &result, seed_joint_position);
    if (result < 0) {
        if (ik_ok) {
            *ik_ok = false;
        }
        return seed_joint_position;
    }

    return solved;
}

bool CartesianSpaceMove::active(double current_time_sec) const {
    const double time_from_start = current_time_sec - start_time_sec_;
    return started_ && (position_trajectory_.active(time_from_start) || pitch_trajectory_.active(time_from_start));
}

void CartesianSpaceMove::stop() {
    started_ = false;
}

arm_calc::CartesianTarget CartesianSpaceMove::sample_target(double time_from_start) const {
    arm_calc::CartesianTarget target;
    target.position = vector_to_position(position_trajectory_.sample(time_from_start));
    target.pitch = vector_to_pitch(pitch_trajectory_.sample(time_from_start));
    return target;
}

arm_calc::TrajectoryVector CartesianSpaceMove::position_to_vector(const Eigen::Vector3d& position) {
    arm_calc::TrajectoryVector vector = arm_calc::TrajectoryVector::Zero();
    vector[0] = position.x();
    vector[1] = position.y();
    vector[2] = position.z();
    return vector;
}

Eigen::Vector3d CartesianSpaceMove::vector_to_position(const arm_calc::TrajectoryVector& vector) {
    return Eigen::Vector3d(vector[0], vector[1], vector[2]);
}

arm_calc::TrajectoryVector CartesianSpaceMove::pitch_to_vector(double pitch) {
    arm_calc::TrajectoryVector vector = arm_calc::TrajectoryVector::Zero();
    vector[0] = pitch;
    return vector;
}

double CartesianSpaceMove::vector_to_pitch(const arm_calc::TrajectoryVector& vector) {
    return vector[0];
}

double CartesianSpaceMove::pitch_from_pose(const arm_calc::CartesianPose& pose, double pitch_reference) {
    Eigen::Vector3d pitch_axis(-pose.position.y(), pose.position.x(), 0.0);
    if (pitch_axis.norm() < 1e-8) {
        pitch_axis = Eigen::Vector3d::UnitY();
    } else {
        pitch_axis.normalize();
    }

    Eigen::Quaterniond orientation = pose.orientation;
    if (orientation.norm() < 1e-9) {
        return pitch_reference;
    }
    orientation.normalize();
    if (orientation.w() < 0.0) {
        orientation.coeffs() *= -1.0;
    }

    const double sin_half_pitch = orientation.vec().dot(pitch_axis);
    const double pitch = 2.0 * std::atan2(sin_half_pitch, orientation.w());
    return normalize_pitch_near(pitch, pitch_reference);
}

double CartesianSpaceMove::normalize_pitch_near(double pitch, double pitch_reference) {
    while (pitch - pitch_reference > kPi) {
        pitch -= 2.0 * kPi;
    }
    while (pitch - pitch_reference < -kPi) {
        pitch += 2.0 * kPi;
    }
    return pitch;
}

}  // namespace arm_action
