#include "arm_calc/arm_calc.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <vector>

namespace arm_calc {

namespace {

CartesianVector ToCartesianVector(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
    CartesianVector vector = CartesianVector::Zero();
    vector.head<3>()       = linear;
    vector.tail<3>()       = angular;
    return vector;
}

} // namespace

ArmCalc::ArmCalc(const KDL::Chain& chain)
    : chain_(chain)
    , fk_solver_(chain_)
    , jacobian_solver_(chain_)
    , jdot_solver_(chain_)
    , vel_solver_(chain_)
    , ik_solver_(chain_, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 1.0, 0.0), 1e-6, 200, 1e-10)
    , dynamic_solver_(chain_, KDL::Vector(0.0, 0.0, -9.81))
    , mass_matrix_(chain_.getNrOfJoints())
    , coriolis_(chain_.getNrOfJoints())
    , gravity_(chain_.getNrOfJoints())
    , jacobian_cache_(chain_.getNrOfJoints())
    , last_joint_solution_(chain_.getNrOfJoints())
    , joint_vel_cache_(chain_.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        last_joint_solution_(i) = 0.0;
    }
   
}


JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result) {
    return joint_pos(pose, result, from_kdl_joints(last_joint_solution_));
}


Eigen::Vector4d ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
    KDL::JntArray seed = to_kdl_joints(seed_joint_pos);
    KDL::Frame frame;

    // 1️⃣ 位置
    const Eigen::Vector3d& target_pos = pose.position;
    frame.p = KDL::Vector(target_pos[0], target_pos[1], target_pos[2]);

    // 2️⃣ 从输入四元数提取 pitch
    KDL::Rotation input_rot =
        KDL::Rotation::Quaternion(pose.orientation.x(), pose.orientation.y(),
                                  pose.orientation.z(), pose.orientation.w());
    double roll, pitch, yaw;
    input_rot.GetRPY(roll, pitch, yaw);

    // ===== wrist pitch 连续性修正 =====
    double prev_pitch = last_joint_solution_(3); // 上一帧 joint4
    if (std::abs(pitch - prev_pitch) > M_PI/2.0) {
        if (pitch > prev_pitch) {
            pitch -= 2*M_PI;
        } else {
            pitch += 2*M_PI;
        }
    }
    // =================================

    // 3️⃣ 先确定平面 → Axis-Angle 构建旋转矩阵
    KDL::Vector P(target_pos[0], target_pos[1], target_pos[2]);
    KDL::Vector U(0.0, 0.0, 1.0);

    double ax  = -target_pos[1];
    double ay  = target_pos[0];
    double az  = 0.0;
    double len = std::sqrt(ax * ax + ay * ay);

    if (len < 1e-8) {
        // r≈0 奇异情况
        frame.M = KDL::Rotation::RPY(0.0, pitch, 0.0);
    } else {
        ax /= len;
        ay /= len;

        // Axis-Angle → 四元数
        double half = pitch * 0.5;
        double qw   = std::cos(half);
        double qx   = ax * std::sin(half);
        double qy   = ay * std::sin(half);
        double qz   = az * std::sin(half);

        // 四元数归一化
        double qnorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (qnorm > 1e-12) {
            qw /= qnorm;
            qx /= qnorm;
            qy /= qnorm;
            qz /= qnorm;
        }

        frame.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);
    }

    // 4️⃣ IK 求解
    *result = ik_solver_.CartToJnt(seed, frame, seed);

    if (*result >= 0) {
        last_joint_solution_ = seed; // 更新上一帧解
    }

    return from_kdl_joints(seed);
}

// Eigen::Vector4d ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
//     KDL::JntArray seed = to_kdl_joints(seed_joint_pos);
//     KDL::Frame frame;
//     // 位置（加上末端偏移，如果以后有偏移量在这里加）
//     const Eigen::Vector3d& target_pos = pose.position;
//     frame.p =
//         KDL::Vector(target_pos[0], target_pos[1], target_pos[2]); // ★★★ 从输入的四元数中提取 wrist pitch（只关心 pitch，忽略 roll/yaw）★★★
//     KDL::Rotation input_rot =
//         KDL::Rotation::Quaternion(pose.orientation.x(), pose.orientation.y(), pose.orientation.z(), pose.orientation.w());
//     double roll, pitch, yaw;
//     input_rot.GetRPY(roll, pitch, yaw);
//     // ★★★ 先确定平面 → 叉乘得到末端旋转轴 → 四元数构建精确旋转矩阵 ★★★
//     KDL::Vector P(target_pos[0], target_pos[1], target_pos[2]);
//     KDL::Vector U(0.0, 0.0, 1.0);
//     // 计算旋转轴：U × P = (-py, px, 0) → 正好与你原来 RPY(0, pitch, 0) 的 +Y 轴一致
//     double ax  = -target_pos[1];
//     double ay  = target_pos[0];
//     double az  = 0.0;
//     double len = std::sqrt(ax * ax + ay * ay);
//     if (len < 1e-8) {
//         // 正上方奇异情况（r≈0），任意选一个轴（这里用世界Y，与旧代码完全一致）
//         frame.M = KDL::Rotation::RPY(0.0, pitch, 0.0);
//     } else {
//         ax /= len;
//         ay /= len;
//         // Axis-Angle → 四元数
//         double half = pitch * 0.5;
//         double qw   = std::cos(half);
//         double qx   = ax * std::sin(half);
//         double qy   = ay * std::sin(half);
//         double qz   = az * std::sin(half);
//         // 四元数归一化（防止浮点误差导致矩阵不正交）
//         double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
//         if (qnorm > 1e-12) {
//             qw /= qnorm;
//             qx /= qnorm;
//             qy /= qnorm;
//             qz /= qnorm;
//         }
//         frame.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);
//     }
//     // 调用IK求解（保持你原来的逻辑）
//     *result = ik_solver_.CartToJnt(seed, frame, seed);
//     if (*result >= 0) {
//         last_joint_solution_ = seed;
//     }
//     return from_kdl_joints(seed);
// }







// JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result) {
//     return joint_pos(pose, result, from_kdl_joints(last_joint_solution_));
// }

// // ====================== 【方案2：加强 IK 连续性】 ======================
// // 在 arm_calc/arm_calc.hpp 中找到原来的 joint_pos 函数，替换为下面这段
// JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
//     KDL::Frame target_frame = to_kdl_frame(pose);
//     JointVector best_solution = seed_joint_pos;  // 默认回退到输入的 seed
//     double best_distance = std::numeric_limits<double>::max();
//     int best_result = -1;

//     // 准备 3 个候选 seed（当前 seed + joint4 ±2π）
//     std::vector<JointVector> candidate_seeds = {seed_joint_pos};

//     JointVector seed_plus = seed_joint_pos;
//     JointVector seed_minus = seed_joint_pos;
//     seed_plus[3]  += 2.0 * M_PI;   // joint4（索引3）正转一圈
//     seed_minus[3] -= 2.0 * M_PI;   // joint4 反转一圈
//     candidate_seeds.push_back(seed_plus);
//     candidate_seeds.push_back(seed_minus);

//    // 依次尝试每个 seed
//     for (const auto& candidate : candidate_seeds) {
//         KDL::JntArray try_seed = to_kdl_joints(candidate);
//         int ik_result = ik_solver_.CartToJnt(try_seed, target_frame, try_seed);

//         if (ik_result >= 0) {  // IK 求解成功
//             JointVector sol = from_kdl_joints(try_seed);
//             // 计算与上一帧实际解的距离（L2范数），选最接近的
//             //double dist = (sol - last_joint_solution_).norm();
//             double dist = (sol - from_kdl_joints(last_joint_solution_)).norm();
//             if (dist < best_distance) {
//                 best_distance = dist;
//                 best_solution = sol;
//                 best_result = ik_result;
//             }
//         }
//     }




//     *result = best_result;

//     if (best_result >= 0) {
//         last_joint_solution_ = to_kdl_joints(best_solution);  // 更新全局 last solution
//         return best_solution;
//     }

//     // 所有尝试都失败，退回到输入的 seed
//     *result = -1;

//     return seed_joint_pos;
// }



// JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
//     KDL::JntArray seed      = to_kdl_joints(seed_joint_pos);
//     KDL::Frame target_frame = to_kdl_frame(pose);
//     *result                 = ik_solver_.CartToJnt(seed, target_frame, seed);
//     if (*result >= 0) {
//         last_joint_solution_ = seed;
//     }
//     return from_kdl_joints(seed);
// }

// JointVector ArmCalc::joint_vel(const JointVector& joint_pos, const CartesianVector& cartesian_twist) {
//     KDL::JntArray joints = to_kdl_joints(joint_pos);
//     jacobian_solver_.JntToJac(joints, jacobian_cache_);
//     Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
//     return jacobian.completeOrthogonalDecomposition().solve(cartesian_twist);
// }

// JointVector ArmCalc::joint_acc(const JointVector& joint_pos, const JointVector& joint_vel, const CartesianVector& cartesian_acc) {
//     joint_vel_cache_.q = to_kdl_joints(joint_pos);
//     joint_vel_cache_.qdot = to_kdl_joints(joint_vel);
//     jacobian_solver_.JntToJac(joint_vel_cache_.q, jacobian_cache_);
//     jdot_solver_.JntToJacDot(joint_vel_cache_, jdot_qdot_cache_);

//     Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
//     const CartesianVector jdot_qdot = twist_to_vector(jdot_qdot_cache_);
//     return jacobian.completeOrthogonalDecomposition().solve(cartesian_acc - jdot_qdot);
// }

// JointVector ArmCalc::joint_torque_dynamic(const JointVector& joint_pos,
//                                           const JointVector& joint_vel,
//                                           const CartesianVector& cartesian_acc) {
//     return joint_torque_inverse_dynamics(joint_pos, joint_vel, joint_acc(joint_pos, joint_vel, cartesian_acc));
// }

// JointVector ArmCalc::joint_torque_inverse_dynamics(const JointVector& joint_pos,
//                                                    const JointVector& joint_vel,
//                                                    const JointVector& joint_acc) {
//     const KDL::JntArray kdl_joint_pos = to_kdl_joints(joint_pos);
//     const KDL::JntArray kdl_joint_vel = to_kdl_joints(joint_vel);

//     dynamic_solver_.JntToGravity(kdl_joint_pos, gravity_);
//     dynamic_solver_.JntToCoriolis(kdl_joint_pos, kdl_joint_vel, coriolis_);
//     dynamic_solver_.JntToMass(kdl_joint_pos, mass_matrix_);

//     JointMatrix mass = mass_matrix_.data;
//     JointVector coriolis = from_kdl_joints(coriolis_);
//     JointVector gravity = from_kdl_joints(gravity_);
//     return mass * joint_acc + coriolis + gravity;
// }

// JointVector ArmCalc::joint_torque_cartesian_wrench(const JointVector& joint_pos, const CartesianVector& cartesian_wrench) {
//     KDL::JntArray joints = to_kdl_joints(joint_pos);
//     jacobian_solver_.JntToJac(joints, jacobian_cache_);
//     Eigen::Matrix<double, 6, 6> jacobian = jacobian_cache_.data;
//     return jacobian.transpose() * cartesian_wrench;
// }

CartesianPose ArmCalc::end_pose(const JointVector& joint_pos) {
    KDL::Frame frame;
    fk_solver_.JntToCart(to_kdl_joints(joint_pos), frame);
    return from_kdl_frame(frame);
}

CartesianState ArmCalc::end_state(const JointVector& joint_pos) {
    CartesianState state;
    state.pose = end_pose(joint_pos);
    // KDL::JntArray joints = to_kdl_joints(joint_pos);
    // jacobian_solver_.JntToJac(joints, jacobian_cache_);
    // const CartesianVector twist = jacobian_cache_.data * joint_vel;
    // state.linear_velocity = twist.head<3>();
    // state.angular_velocity = twist.tail<3>();
    return state;
}

KDL::Jacobian ArmCalc::jacobian(const JointVector& joint_pos) {
    KDL::JntArray joints = to_kdl_joints(joint_pos);
    jacobian_solver_.JntToJac(joints, jacobian_cache_);
    return jacobian_cache_;
}

void ArmCalc::set_joint_pd(std::size_t index, double kp, double kd) {
    if (index >= kJointDoF) {
        return;
    }
    kp_[static_cast<int>(index)] = kp;
    kd_[static_cast<int>(index)] = kd;
}

void ArmCalc::get_joint_pd(std::size_t index, double& kp, double& kd) const {
    if (index >= kJointDoF) {
        kp = 0.0;
        kd = 0.0;
        return;
    }
    kp = kp_[static_cast<int>(index)];
    kd = kd_[static_cast<int>(index)];
}

JointTrajectoryPoint ArmCalc::signal_arm_calc(const CartesianTrajectoryPoint& cartesian_target) {
    JointTrajectoryPoint point;
    int result     = -1;
    point.position = joint_pos(cartesian_target.pose, &result);
    // if (result < 0) {
    //     point.velocity.setZero();
    //     point.acceleration.setZero();
    //     point.torque = joint_torque_inverse_dynamics(point.position, point.velocity, point.acceleration);
    //     return point;
    // }

    // const CartesianVector twist = ToCartesianVector(cartesian_target.linear_velocity, cartesian_target.angular_velocity);
    // const CartesianVector acceleration =
    //     ToCartesianVector(cartesian_target.linear_acceleration, cartesian_target.angular_acceleration);

    // point.velocity = joint_vel(point.position, twist);
    // point.acceleration = joint_acc(point.position, point.velocity, acceleration);
    // point.torque = joint_torque_dynamic(point.position, point.velocity, acceleration);
    return point;
}

KDL::Frame ArmCalc::to_kdl_frame(const CartesianPose& pose) {
    const Eigen::Quaterniond q = NormalizeQuaternion(pose.orientation);
    return KDL::Frame(
        KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()), KDL::Vector(pose.position.x(), pose.position.y(), pose.position.z()));
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

KDL::JntArray ArmCalc::to_kdl_joints(const JointVector& joints) {
    KDL::JntArray output(static_cast<unsigned int>(kJointDoF));
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output(static_cast<unsigned int>(i)) = joints[static_cast<int>(i)];
    }
    return output;
}

JointVector ArmCalc::from_kdl_joints(const KDL::JntArray& joints) {
    JointVector output = JointVector::Zero();
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output[static_cast<int>(i)] = joints(static_cast<unsigned int>(i));
    }
    return output;
}

CartesianVector ArmCalc::twist_to_vector(const KDL::Twist& twist) {
    CartesianVector output = CartesianVector::Zero();
    output << twist.vel.x(), twist.vel.y(), twist.vel.z(), twist.rot.x(), twist.rot.y(), twist.rot.z();
    return output;
}

} // namespace arm_calc
