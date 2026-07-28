#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace {

constexpr std::size_t kHarmonicCount = 5;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLimitMarginRatio = 0.1;
constexpr double kDefaultLowerLimit = -kPi;
constexpr double kDefaultUpperLimit = kPi;
constexpr double kMinDurationSec = 1e-3;
constexpr double kMinMoveToCenterDurationSec = 2.0;
constexpr double kMoveVelocityLimitRatio = 0.35;
constexpr double kExcitationVelocityLimitRatio = 0.8;

bool IsFiniteLimit(double value)
{
    return std::isfinite(value) && std::abs(value) < 1e8;
}

}  // namespace

Trajectory::Trajectory(const std::string urdf_file_path)
    : urdf_file_path_(urdf_file_path)
{
}

bool Trajectory::generate(std::chrono::high_resolution_clock::duration trajectory_time, const std::vector<float>& init_pos)
{
    if (!load_joint_limits()) {
        return false;
    }

    if (init_pos.size() != lower_limits_.size()) {
        return false;
    }

    duration_sec_ = std::chrono::duration<double>(trajectory_time).count();
    if (duration_sec_ <= kMinDurationSec) {
        return false;
    }

    base_frequency_ = 2.0 * kPi / duration_sec_;
    coefficients_.assign(lower_limits_.size(), std::vector<FourierTerm>(kHarmonicCount));
    initial_positions_.resize(lower_limits_.size());
    center_positions_.resize(lower_limits_.size());
    move_to_center_duration_sec_ = kMinMoveToCenterDurationSec;

    for (std::size_t joint = 0; joint < coefficients_.size(); ++joint) {
        const double range = upper_limits_[joint] - lower_limits_[joint];
        if (range <= 0.0) {
            return false;
        }

        const double initial_position = static_cast<double>(init_pos[joint]);
        if (initial_position < lower_limits_[joint] || initial_position > upper_limits_[joint]) {
            return false;
        }

        initial_positions_[joint] = initial_position;
        center_positions_[joint] = 0.5 * (lower_limits_[joint] + upper_limits_[joint]);

        if (velocity_limits_[joint] > 0.0 && IsFiniteLimit(velocity_limits_[joint])) {
            const double displacement = std::abs(center_positions_[joint] - initial_positions_[joint]);
            const double velocity_limit = kMoveVelocityLimitRatio * velocity_limits_[joint];
            if (velocity_limit > std::numeric_limits<double>::epsilon()) {
                move_to_center_duration_sec_ =
                    std::max(move_to_center_duration_sec_, 1.875 * displacement / velocity_limit);
            }
        }
    }

    for (std::size_t joint = 0; joint < coefficients_.size(); ++joint) {
        const double range = upper_limits_[joint] - lower_limits_[joint];
        const double limit_margin = range * kLimitMarginRatio;
        const double soft_lower = lower_limits_[joint] + limit_margin;
        const double soft_upper = upper_limits_[joint] - limit_margin;
        const double center_position = center_positions_[joint];

        const double upper_room = soft_upper - center_position;
        const double lower_room = center_position - soft_lower;
        const double position_amplitude_limit = std::max(0.0, std::min(upper_room, lower_room));
        if (position_amplitude_limit <= std::numeric_limits<double>::epsilon()) {
            continue;
        }

        double raw_position_bound = 0.0;
        double raw_velocity_bound = 0.0;

        for (std::size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
            const double harmonic_index = static_cast<double>(harmonic + 1);
            const double joint_index = static_cast<double>(joint + 1);
            FourierTerm& term = coefficients_[joint][harmonic];

            term.sin_coeff = std::sin(0.73 * joint_index * harmonic_index) / harmonic_index;
            term.cos_coeff = std::cos(1.17 * joint_index + 0.41 * harmonic_index) / (harmonic_index * harmonic_index);
        }

        double initial_velocity_sum = 0.0;
        double initial_acceleration_sum = 0.0;
        for (std::size_t harmonic = 1; harmonic < kHarmonicCount; ++harmonic) {
            const double harmonic_index = static_cast<double>(harmonic + 1);
            initial_velocity_sum += harmonic_index * coefficients_[joint][harmonic].sin_coeff;
            initial_acceleration_sum += harmonic_index * harmonic_index * coefficients_[joint][harmonic].cos_coeff;
        }
        coefficients_[joint][0].sin_coeff = -initial_velocity_sum;
        coefficients_[joint][0].cos_coeff = -initial_acceleration_sum;

        for (std::size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
            const double harmonic_index = static_cast<double>(harmonic + 1);
            const FourierTerm& term = coefficients_[joint][harmonic];
            raw_position_bound += std::abs(term.sin_coeff) + 2.0 * std::abs(term.cos_coeff);
            raw_velocity_bound += harmonic_index * base_frequency_ *
                                  (std::abs(term.sin_coeff) + std::abs(term.cos_coeff));
        }

        double scale = position_amplitude_limit / std::max(raw_position_bound, std::numeric_limits<double>::epsilon());
        if (velocity_limits_[joint] > 0.0 && IsFiniteLimit(velocity_limits_[joint])) {
            const double velocity_amplitude_limit = kExcitationVelocityLimitRatio * velocity_limits_[joint];
            scale = std::min(scale, velocity_amplitude_limit /
                                        std::max(raw_velocity_bound, std::numeric_limits<double>::epsilon()));
        }

        for (auto& term : coefficients_[joint]) {
            term.sin_coeff *= scale;
            term.cos_coeff *= scale;
        }
    }

    generated_ = true;
    started_ = false;
    return true;
}

bool Trajectory::start(std::chrono::time_point<std::chrono::high_resolution_clock> time_point)
{
    if (!generated_) {
        return false;
    }

    start_time_ = time_point;
    started_ = true;
    return true;
}

bool Trajectory::sample(std::chrono::time_point<std::chrono::high_resolution_clock> time_point,
                        std::vector<float>& joint_exp_pos)
{
    if (!generated_ || !started_) {
        return false;
    }

    double elapsed_sec = std::chrono::duration<double>(time_point - start_time_).count();
    if (elapsed_sec < 0.0) {
        elapsed_sec = 0.0;
    }

    joint_exp_pos.resize(coefficients_.size());
    if (elapsed_sec < move_to_center_duration_sec_) {
        for (std::size_t joint = 0; joint < coefficients_.size(); ++joint) {
            joint_exp_pos[joint] = static_cast<float>(evaluate_move_to_center_joint(joint, elapsed_sec));
        }
    } else {
        const double excitation_elapsed_sec = std::fmod(elapsed_sec - move_to_center_duration_sec_, duration_sec_);
        for (std::size_t joint = 0; joint < coefficients_.size(); ++joint) {
            joint_exp_pos[joint] = static_cast<float>(evaluate_joint(joint, excitation_elapsed_sec));
        }
    }

    return true;
}

double Trajectory::total_duration() const
{
    return move_to_center_duration_sec_ + duration_sec_;
}

bool Trajectory::load_joint_limits()
{
    if (limits_loaded_) {
        return true;
    }

    pinocchio::Model model;
    try {
        pinocchio::urdf::buildModel(urdf_file_path_, model);
    } catch (...) {
        return false;
    }

    if (model.nq == 0) {
        return false;
    }

    lower_limits_.resize(static_cast<std::size_t>(model.nq));
    upper_limits_.resize(static_cast<std::size_t>(model.nq));
    velocity_limits_.resize(static_cast<std::size_t>(model.nv), 0.0);

    for (Eigen::DenseIndex i = 0; i < model.nq; ++i) {
        double lower = model.lowerPositionLimit[i];
        double upper = model.upperPositionLimit[i];

        if (!IsFiniteLimit(lower) || !IsFiniteLimit(upper) || lower >= upper) {
            lower = kDefaultLowerLimit;
            upper = kDefaultUpperLimit;
        }

        lower_limits_[static_cast<std::size_t>(i)] = lower;
        upper_limits_[static_cast<std::size_t>(i)] = upper;
    }

    for (Eigen::DenseIndex i = 0; i < model.nv; ++i) {
        velocity_limits_[static_cast<std::size_t>(i)] = model.velocityLimit[i];
    }

    limits_loaded_ = true;
    return true;
}

double Trajectory::evaluate_joint(std::size_t joint_index, double elapsed_sec) const
{
    const double lower = lower_limits_[joint_index];
    const double upper = upper_limits_[joint_index];
    double position = center_positions_[joint_index];

    for (std::size_t harmonic = 0; harmonic < coefficients_[joint_index].size(); ++harmonic) {
        const double harmonic_index = static_cast<double>(harmonic + 1);
        const double phase = harmonic_index * base_frequency_ * elapsed_sec;
        const FourierTerm& term = coefficients_[joint_index][harmonic];
        position += term.sin_coeff * std::sin(phase) + term.cos_coeff * (std::cos(phase) - 1.0);
    }

    return std::clamp(position, lower, upper);
}

double Trajectory::evaluate_move_to_center_joint(std::size_t joint_index, double elapsed_sec) const
{
    const double normalized_time =
        std::clamp(elapsed_sec / std::max(move_to_center_duration_sec_, kMinDurationSec), 0.0, 1.0);
    const double blend = smooth_step_quintic(normalized_time);
    const double position =
        initial_positions_[joint_index] + (center_positions_[joint_index] - initial_positions_[joint_index]) * blend;
    return std::clamp(position, lower_limits_[joint_index], upper_limits_[joint_index]);
}

double Trajectory::smooth_step_quintic(double normalized_time)
{
    const double t = std::clamp(normalized_time, 0.0, 1.0);
    return 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
}
