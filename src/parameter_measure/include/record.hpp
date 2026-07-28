#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

/**
* @brief 将采集到的关节位置速度加速度力矩信息记录到CSV
 */

class Record {
public:
    explicit Record();
    bool start(int joint_dof, const std::string csv_file_path);
    bool stop();
    bool record(
        std::chrono::time_point<std::chrono::high_resolution_clock> time_point, const std::vector<float>& joint_pos,
        const std::vector<float>& joint_vel, const std::vector<float>& joint_acc, const std::vector<float>& joint_torque);

private:
    void write_header();
    bool check_joint_data(const std::vector<float>& joint_pos, const std::vector<float>& joint_vel,
                          const std::vector<float>& joint_acc, const std::vector<float>& joint_torque) const;

    int joint_dof_{0};
    bool is_recording_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
    std::ofstream csv_file_;
};
