#include "arm_action/cartesian_space_move.hpp"

#include <algorithm>
#include <utility>

namespace arm_action {

CartesianSpaceMove::CartesianSpaceMove(std::shared_ptr<arm_calc::ArmCalc> arm_calc)
    : arm_calc_(std::move(arm_calc)) {}

void CartesianSpaceMove::start(
    arm_calc::ArmSide side,
    const arm_calc::JointPosition& start_joint_position,
    const arm_calc::CartesianTarget& goal_target,
    double duration,
    double start_time_sec) {
    side_ = side;
    last_joint_position_ = start_joint_position;
    duration_sec_ = std::max(duration, 1e-3);
    start_time_sec_ = start_time_sec;

    arm_calc::CartesianTarget start_target;
    if (arm_calc_) {
        const arm_calc::CartesianPose start_pose = arm_calc_->end_pose(side_, start_joint_position);
        start_target.position = start_pose.position;
        arm_calc_->set_last_joint_pos(side_, start_joint_position);
    }
    start_target.pitch = start_joint_position[static_cast<int>(arm_calc::kArmJointDof - 1)];

    trajectory_.reset(to_vector(start_target), to_vector(goal_target), duration_sec_);
    started_ = true;
}

arm_calc::JointPosition CartesianSpaceMove::sample(double current_time_sec, bool* ik_ok) {
    if (ik_ok) {
        *ik_ok = true;
    }
    if (!started_ || !arm_calc_) {
        return last_joint_position_;
    }

    const arm_calc::CartesianTarget target = to_target(trajectory_.sample(current_time_sec - start_time_sec_));
    int result = -1;
    const arm_calc::JointPosition solved = arm_calc_->joint_pos(side_, target, &result, last_joint_position_);
    if (result < 0) {
        if (ik_ok) {
            *ik_ok = false;
        }
        return last_joint_position_;
    }

    last_joint_position_ = solved;
    return last_joint_position_;
}

bool CartesianSpaceMove::active(double current_time_sec) const {
    return started_ && trajectory_.active(current_time_sec - start_time_sec_);
}

void CartesianSpaceMove::stop() {
    started_ = false;
}

arm_calc::TrajectoryVector CartesianSpaceMove::to_vector(const arm_calc::CartesianTarget& target) {
    arm_calc::TrajectoryVector vector = arm_calc::TrajectoryVector::Zero();
    vector[0] = target.position.x();
    vector[1] = target.position.y();
    vector[2] = target.position.z();
    vector[3] = target.pitch;
    return vector;
}

arm_calc::CartesianTarget CartesianSpaceMove::to_target(const arm_calc::TrajectoryVector& vector) {
    arm_calc::CartesianTarget target;
    target.position = Eigen::Vector3d(vector[0], vector[1], vector[2]);
    target.pitch = vector[3];
    return target;
}

}  // namespace arm_action
