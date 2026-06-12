#include "arm_calc/arm_calc.hpp"

#include <cmath>

namespace arm_calc {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix<double, 6, 1> IkWeights() {
    Eigen::Matrix<double, 6, 1> weights;
    weights << 1.0, 1.0, 1.0, 0.0, 1.0, 0.0;
    return weights;
}

Eigen::Quaterniond NormalizeQuaternion(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond normalized = q;
    if (normalized.norm() < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }
    normalized.normalize();
    return normalized;
}

} // namespace

ArmCalc::ChainContext::ChainContext(const KDL::Chain& input_chain)
    : chain(input_chain)
    , fk_solver(chain)
    , ik_solver(chain, IkWeights(), 1e-6, 200, 1e-10)
    , last_solution(chain.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
        last_solution(i) = 0.0;
    }
}

ArmCalc::ArmCalc(const KDL::Chain& left_chain, const KDL::Chain& right_chain)
    : left_(left_chain)
    , right_(right_chain) {}

JointPosition ArmCalc::joint_pos(ArmSide side, const CartesianTarget& target, int* result) {
    auto& chain = context(side);
    return joint_pos(side, target, result, from_kdl_joints(chain.last_solution));
}

JointPosition ArmCalc::joint_pos(
    ArmSide side,
    const CartesianTarget& target,
    int* result,
    const JointPosition& seed_joint_pos) {
    auto& chain = context(side);
    KDL::JntArray seed = to_kdl_joints(seed_joint_pos);
    const KDL::Frame frame = target_to_kdl_frame(target, seed_joint_pos[static_cast<int>(kArmJointDof - 1)]);
    const int ik_result = chain.ik_solver.CartToJnt(seed, frame, seed);
    if (result) {
        *result = ik_result;
    }

    if (ik_result < 0) {
        return seed_joint_pos;
    }

    chain.last_solution = seed;
    return from_kdl_joints(seed);
}

CartesianPose ArmCalc::end_pose(ArmSide side, const JointPosition& joint_pos) {
    auto& chain = context(side);
    KDL::Frame frame;
    chain.fk_solver.JntToCart(to_kdl_joints(joint_pos), frame);
    return from_kdl_frame(frame);
}

void ArmCalc::set_last_joint_pos(ArmSide side, const JointPosition& joint_pos) {
    context(side).last_solution = to_kdl_joints(joint_pos);
}

ArmCalc::ChainContext& ArmCalc::context(ArmSide side) {
    return side == ArmSide::kLeft ? left_ : right_;
}

KDL::Frame ArmCalc::target_to_kdl_frame(const CartesianTarget& target, double pitch_reference) {
    KDL::Frame frame;
    frame.p = KDL::Vector(target.position.x(), target.position.y(), target.position.z());

    double pitch = target.pitch;
    while (pitch - pitch_reference > kPi) {
        pitch -= 2.0 * kPi;
    }
    while (pitch - pitch_reference < -kPi) {
        pitch += 2.0 * kPi;
    }

    double ax = -target.position.y();
    double ay = target.position.x();
    constexpr double az = 0.0;
    const double len = std::sqrt(ax * ax + ay * ay);

    if (len < 1e-8) {
        frame.M = KDL::Rotation::RPY(0.0, pitch, 0.0);
        return frame;
    }

    ax /= len;
    ay /= len;

    const double half = pitch * 0.5;
    double qw = std::cos(half);
    double qx = ax * std::sin(half);
    double qy = ay * std::sin(half);
    double qz = az * std::sin(half);

    const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qnorm > 1e-12) {
        qw /= qnorm;
        qx /= qnorm;
        qy /= qnorm;
        qz /= qnorm;
    }

    frame.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);
    return frame;
}

CartesianPose ArmCalc::from_kdl_frame(const KDL::Frame& frame) {
    CartesianPose pose;
    pose.position = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation = NormalizeQuaternion(Eigen::Quaterniond(w, x, y, z));
    return pose;
}

KDL::JntArray ArmCalc::to_kdl_joints(const JointPosition& joints) {
    KDL::JntArray output(static_cast<unsigned int>(kArmJointDof));
    for (std::size_t i = 0; i < kArmJointDof; ++i) {
        output(static_cast<unsigned int>(i)) = joints[static_cast<int>(i)];
    }
    return output;
}

JointPosition ArmCalc::from_kdl_joints(const KDL::JntArray& joints) {
    JointPosition output = JointPosition::Zero();
    for (std::size_t i = 0; i < kArmJointDof; ++i) {
        output[static_cast<int>(i)] = joints(static_cast<unsigned int>(i));
    }
    return output;
}

} // namespace arm_calc
