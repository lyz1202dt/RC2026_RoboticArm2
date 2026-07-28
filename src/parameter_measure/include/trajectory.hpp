#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

class Trajectory
{
public:
    explicit Trajectory(std::string csv_file_path);

    bool load(const std::vector<float>& init_pos, double move_to_start_duration_sec);
    bool start(std::chrono::time_point<std::chrono::high_resolution_clock> time_point);
    bool sample(std::chrono::time_point<std::chrono::high_resolution_clock> time_point, std::vector<float>& joint_exp_pos);
    double total_duration() const;
    double move_to_start_duration() const;

private:
    double evaluate_move_to_start_joint(std::size_t joint_index, double elapsed_sec) const;
    void evaluate_playback(double elapsed_sec, std::vector<float>& joint_exp_pos) const;
    static double smooth_step_quintic(double normalized_time);

    std::string csv_file_path_;
    bool loaded_{false};
    bool started_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    double move_to_start_duration_sec_{0.0};
    std::vector<float> initial_positions_;
    std::vector<double> sample_times_;
    std::vector<std::vector<float>> sample_positions_;
};
