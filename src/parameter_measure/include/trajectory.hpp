#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

class Trajectory{
public:
    Trajectory(const std::string urdf_file_path);
    bool generate(std::chrono::high_resolution_clock::duration trajectory_time,const std::vector<float> &init_pos);
    bool start(std::chrono::time_point<std::chrono::high_resolution_clock> time_point);
    bool sample(std::chrono::time_point<std::chrono::high_resolution_clock> time_point,std::vector<float>& joint_exp_pos);
    double total_duration() const;

private:
    struct FourierTerm {
        double sin_coeff{0.0};
        double cos_coeff{0.0};
    };

    bool load_joint_limits();
    double evaluate_joint(std::size_t joint_index, double elapsed_sec) const;
    double evaluate_move_to_center_joint(std::size_t joint_index, double elapsed_sec) const;
    static double smooth_step_quintic(double normalized_time);

    std::string urdf_file_path_;
    bool limits_loaded_{false};
    bool generated_{false};
    bool started_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    double duration_sec_{0.0};
    double move_to_center_duration_sec_{0.0};
    double base_frequency_{0.0};
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    std::vector<double> velocity_limits_;
    std::vector<double> initial_positions_;
    std::vector<double> center_positions_;
    std::vector<std::vector<FourierTerm>> coefficients_;
};
