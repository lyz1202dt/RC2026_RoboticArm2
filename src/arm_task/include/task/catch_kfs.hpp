#pragma once

#include "task/base_task.hpp"
#include <Eigen/Core>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using Vector3D = Eigen::Vector3d;

class CatchKFS : public BaseTask {
public:
    CatchKFS(Robot* context, const std::string name);
    ~CatchKFS() override;
    std::string process(const std::string last_task_name) override;

private:
};
