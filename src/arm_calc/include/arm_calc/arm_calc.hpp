#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <cstddef>

namespace arm_calc {

constexpr std::size_t kArmJointDof = 4;

using JointPosition = Eigen::Matrix<double, static_cast<int>(kArmJointDof), 1>;

enum class ArmSide { kLeft = 0, kRight = 1 };

struct CartesianTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double pitch{0.0};
};

struct CartesianPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class ArmCalc {
public:
    ArmCalc(const KDL::Chain& left_chain, const KDL::Chain& right_chain);
    ~ArmCalc() = default;

    JointPosition joint_pos(ArmSide side, const CartesianTarget& target, int* result);
    JointPosition joint_pos(ArmSide side, const CartesianTarget& target, int* result, const JointPosition& seed_joint_pos);

    CartesianPose end_pose(ArmSide side, const JointPosition& joint_pos);
    void set_last_joint_pos(ArmSide side, const JointPosition& joint_pos);

private:
    struct ChainContext {
        explicit ChainContext(const KDL::Chain& input_chain);

        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive fk_solver;
        KDL::ChainIkSolverPos_LMA ik_solver;
        KDL::JntArray last_solution;
    };

    ChainContext& context(ArmSide side);

    static KDL::Frame target_to_kdl_frame(const CartesianTarget& target, double pitch_reference);
    static CartesianPose from_kdl_frame(const KDL::Frame& frame);
    static KDL::JntArray to_kdl_joints(const JointPosition& joints);
    static JointPosition from_kdl_joints(const KDL::JntArray& joints);

    ChainContext left_;
    ChainContext right_;
};

}  // namespace arm_calc
