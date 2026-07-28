#include "record.hpp"

#include <iomanip>
#include <utility>

Record::Record()
{
}

bool Record::start(int joint_dof, const std::string csv_file_path,
                   std::chrono::time_point<std::chrono::high_resolution_clock> start_time)
{
    if (joint_dof <= 0) {
        return false;
    }

    if (is_recording_) {
        stop();
    }

    csv_file_.open(csv_file_path, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        return false;
    }

    joint_dof_ = joint_dof;
    start_time_ = start_time;
    is_recording_ = true;
    write_header();
    return csv_file_.good();
}

bool Record::stop()
{
    if (!is_recording_) {
        return true;
    }

    csv_file_.flush();
    const bool success = csv_file_.good();
    csv_file_.close();
    is_recording_ = false;
    joint_dof_ = 0;
    return success;
}

bool Record::record(std::chrono::time_point<std::chrono::high_resolution_clock> time_point,
                    const std::vector<float>& joint_pos, const std::vector<float>& joint_vel,
                    const std::vector<float>& joint_torque)
{
    if (!is_recording_ || !csv_file_.is_open() || !check_joint_data(joint_pos, joint_vel, joint_torque)) {
        return false;
    }

    const std::chrono::duration<double> elapsed = time_point - start_time_;
    csv_file_ << std::fixed << std::setprecision(9) << elapsed.count();

    const auto write_vector = [this](const std::vector<float>& values) {
        for (const float value : values) {
            csv_file_ << ',' << value;
        }
    };

    write_vector(joint_pos);
    write_vector(joint_vel);
    write_vector(joint_torque);
    csv_file_ << '\n';

    return csv_file_.good();
}

void Record::write_header()
{
    csv_file_ << "time";

    const auto write_joint_names = [this](const char* prefix) {
        for (int i = 0; i < joint_dof_; ++i) {
            csv_file_ << ',' << prefix << '_' << i;
        }
    };

    write_joint_names("pos");
    write_joint_names("vel");
    write_joint_names("torque");
    csv_file_ << '\n';
}

bool Record::check_joint_data(const std::vector<float>& joint_pos, const std::vector<float>& joint_vel,
                              const std::vector<float>& joint_torque) const
{
    const auto dof = static_cast<std::size_t>(joint_dof_);
    return joint_pos.size() == dof && joint_vel.size() == dof && joint_torque.size() == dof;
}
