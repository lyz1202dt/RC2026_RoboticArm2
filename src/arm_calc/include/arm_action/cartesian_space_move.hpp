#pragma once

#include "arm_calc/arm_calc.hpp"
#include "arm_calc/trajectory_calc.hpp"

#include <memory>

namespace arm_action {

class CartesianSpaceMove {
public:
    explicit CartesianSpaceMove(std::shared_ptr<arm_calc::ArmCalc> arm_calc);

    void start(arm_calc::ArmSide side,
               const arm_calc::JointPosition& start_joint_position,
               const arm_calc::CartesianTarget& goal_target,
               double duration,
               double start_time_sec);

    arm_calc::JointPosition sample(double current_time_sec, bool* ik_ok = nullptr);
    bool active(double current_time_sec) const;
    bool started() const { return started_; }
    void stop();

private:
    static arm_calc::TrajectoryVector to_vector(const arm_calc::CartesianTarget& target);
    static arm_calc::CartesianTarget to_target(const arm_calc::TrajectoryVector& vector);

    std::shared_ptr<arm_calc::ArmCalc> arm_calc_;
    arm_calc::TrajectoryCalc trajectory_;
    arm_calc::ArmSide side_{arm_calc::ArmSide::kLeft};
    arm_calc::JointPosition last_joint_position_{arm_calc::JointPosition::Zero()};
    double start_time_sec_{0.0};
    double duration_sec_{0.0};
    bool started_{false};
};

}  // namespace arm_action
