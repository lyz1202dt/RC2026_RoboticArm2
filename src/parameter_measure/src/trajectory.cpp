#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace {

constexpr double kMinDurationSec = 1e-3;

std::vector<std::string> SplitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        while (!field.empty() && (field.back() == '\r' || field.back() == ' ' || field.back() == '\t')) {
            field.pop_back();
        }
        while (!field.empty() && (field.front() == ' ' || field.front() == '\t')) {
            field.erase(field.begin());
        }
        fields.push_back(field);
    }
    return fields;
}

bool ParseDouble(const std::string& text, double& value)
{
    try {
        std::size_t parsed = 0;
        value = std::stod(text, &parsed);
        return parsed == text.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

}  // namespace

Trajectory::Trajectory(std::string csv_file_path)
    : csv_file_path_(std::move(csv_file_path))
{
}

bool Trajectory::load(const std::vector<float>& init_pos, double move_to_start_duration_sec)
{
    std::ifstream csv(csv_file_path_);
    if (!csv.is_open() || init_pos.empty()) {
        return false;
    }

    std::string line;
    if (!std::getline(csv, line)) {
        return false;
    }

    const std::vector<std::string> header = SplitCsvLine(line);
    if (header.size() != init_pos.size() + 1 || header.front() != "time") {
        return false;
    }
    for (std::size_t joint = 0; joint < init_pos.size(); ++joint) {
        if (header[joint + 1] != "pos_" + std::to_string(joint)) {
            return false;
        }
    }

    std::vector<double> times;
    std::vector<std::vector<float>> positions;
    while (std::getline(csv, line)) {
        if (line.empty()) {
            continue;
        }

        const std::vector<std::string> fields = SplitCsvLine(line);
        if (fields.size() != init_pos.size() + 1) {
            return false;
        }

        double time_sec = 0.0;
        if (!ParseDouble(fields.front(), time_sec)) {
            return false;
        }
        if (!times.empty() && time_sec <= times.back()) {
            return false;
        }

        std::vector<float> row(init_pos.size(), 0.0F);
        for (std::size_t joint = 0; joint < init_pos.size(); ++joint) {
            double value = 0.0;
            if (!ParseDouble(fields[joint + 1], value)) {
                return false;
            }
            row[joint] = static_cast<float>(value);
        }

        times.push_back(time_sec);
        positions.push_back(std::move(row));
    }

    if (times.size() < 2 || times.front() < -kMinDurationSec) {
        return false;
    }

    const double first_time = times.front();
    if (std::abs(first_time) > kMinDurationSec) {
        for (double& time_sec : times) {
            time_sec -= first_time;
        }
    }

    initial_positions_ = init_pos;
    sample_times_ = std::move(times);
    sample_positions_ = std::move(positions);
    move_to_start_duration_sec_ = std::max(move_to_start_duration_sec, kMinDurationSec);
    loaded_ = true;
    started_ = false;
    return true;
}

bool Trajectory::start(std::chrono::time_point<std::chrono::high_resolution_clock> time_point)
{
    if (!loaded_) {
        return false;
    }

    start_time_ = time_point;
    started_ = true;
    return true;
}

bool Trajectory::sample(std::chrono::time_point<std::chrono::high_resolution_clock> time_point,
                        std::vector<float>& joint_exp_pos)
{
    if (!loaded_ || !started_) {
        return false;
    }

    double elapsed_sec = std::chrono::duration<double>(time_point - start_time_).count();
    if (elapsed_sec < 0.0) {
        elapsed_sec = 0.0;
    }

    joint_exp_pos.resize(initial_positions_.size());
    if (elapsed_sec < move_to_start_duration_sec_) {
        for (std::size_t joint = 0; joint < initial_positions_.size(); ++joint) {
            joint_exp_pos[joint] = static_cast<float>(evaluate_move_to_start_joint(joint, elapsed_sec));
        }
    } else {
        evaluate_playback(elapsed_sec - move_to_start_duration_sec_, joint_exp_pos);
    }

    return true;
}

double Trajectory::total_duration() const
{
    if (sample_times_.empty()) {
        return move_to_start_duration_sec_;
    }
    return move_to_start_duration_sec_ + sample_times_.back();
}

double Trajectory::move_to_start_duration() const
{
    return move_to_start_duration_sec_;
}

double Trajectory::evaluate_move_to_start_joint(std::size_t joint_index, double elapsed_sec) const
{
    const double normalized_time =
        std::clamp(elapsed_sec / std::max(move_to_start_duration_sec_, kMinDurationSec), 0.0, 1.0);
    const double blend = smooth_step_quintic(normalized_time);
    return initial_positions_[joint_index] +
           (sample_positions_.front()[joint_index] - initial_positions_[joint_index]) * blend;
}

void Trajectory::evaluate_playback(double elapsed_sec, std::vector<float>& joint_exp_pos) const
{
    if (elapsed_sec <= sample_times_.front()) {
        joint_exp_pos = sample_positions_.front();
        return;
    }
    if (elapsed_sec >= sample_times_.back()) {
        joint_exp_pos = sample_positions_.back();
        return;
    }

    const auto upper =
        std::upper_bound(sample_times_.begin(), sample_times_.end(), elapsed_sec);
    const std::size_t next_index = static_cast<std::size_t>(upper - sample_times_.begin());
    const std::size_t prev_index = next_index - 1;
    const double segment_duration = sample_times_[next_index] - sample_times_[prev_index];
    const double alpha = (elapsed_sec - sample_times_[prev_index]) / std::max(segment_duration, kMinDurationSec);

    for (std::size_t joint = 0; joint < joint_exp_pos.size(); ++joint) {
        const double start = static_cast<double>(sample_positions_[prev_index][joint]);
        const double end = static_cast<double>(sample_positions_[next_index][joint]);
        joint_exp_pos[joint] = static_cast<float>(start + (end - start) * alpha);
    }
}

double Trajectory::smooth_step_quintic(double normalized_time)
{
    const double t = std::clamp(normalized_time, 0.0, 1.0);
    return 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
}
