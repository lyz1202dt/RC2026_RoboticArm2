#include "arm_action/joint_space_move.hpp"

#include <algorithm>

namespace arm_action {

void JointSpaceMove::start(
    const arm_calc::JointPosition& start_position,
    const arm_calc::JointPosition& goal_position,
    double duration,
    double start_time_sec) {
    hold_position_ = start_position;
    duration_sec_ = std::max(duration, 1e-3);
    start_time_sec_ = start_time_sec;
    trajectory_.reset(start_position, goal_position, duration_sec_);
    started_ = true;
}

arm_calc::JointPosition JointSpaceMove::sample(double current_time_sec) const {
    if (!started_) {
        return hold_position_;
    }
    return trajectory_.sample(current_time_sec - start_time_sec_);
}

bool JointSpaceMove::active(double current_time_sec) const {
    return started_ && trajectory_.active(current_time_sec - start_time_sec_);
}

void JointSpaceMove::stop() {
    started_ = false;
}

}  // namespace arm_action
