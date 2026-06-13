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

    arm_calc::JointPosition sample(double current_time_sec,
                                   const arm_calc::JointPosition& seed_joint_position,
                                   bool* ik_ok = nullptr);
    bool active(double current_time_sec) const;
    bool started() const { return started_; }
    void stop();

private:
    static arm_calc::TrajectoryVector position_to_vector(const Eigen::Vector3d& position);
    static Eigen::Vector3d vector_to_position(const arm_calc::TrajectoryVector& vector);
    static arm_calc::TrajectoryVector pitch_to_vector(double pitch);
    static double vector_to_pitch(const arm_calc::TrajectoryVector& vector);
    static double pitch_from_pose(const arm_calc::CartesianPose& pose, double pitch_reference);
    static double normalize_pitch_near(double pitch, double pitch_reference);

    arm_calc::CartesianTarget sample_target(double time_from_start) const;

    std::shared_ptr<arm_calc::ArmCalc> arm_calc_;
    arm_calc::TrajectoryCalc position_trajectory_;
    arm_calc::TrajectoryCalc pitch_trajectory_;
    arm_calc::ArmSide side_{arm_calc::ArmSide::kLeft};
    double start_time_sec_{0.0};
    double duration_sec_{0.0};
    bool started_{false};
};

}  // namespace arm_action
