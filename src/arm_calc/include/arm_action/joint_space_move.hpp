#pragma once

#include "arm_calc/arm_calc.hpp"
#include "arm_calc/trajectory_calc.hpp"

namespace arm_action {

class JointSpaceMove {
public:
    void start(const arm_calc::JointPosition& start_position,
               const arm_calc::JointPosition& goal_position,
               double duration,
               double start_time_sec);

    arm_calc::JointPosition sample(double current_time_sec) const;
    bool active(double current_time_sec) const;
    bool started() const { return started_; }
    void stop();

private:
    arm_calc::TrajectoryCalc trajectory_;
    arm_calc::JointPosition hold_position_{arm_calc::JointPosition::Zero()};
    double start_time_sec_{0.0};
    double duration_sec_{0.0};
    bool started_{false};
};

}  // namespace arm_action
