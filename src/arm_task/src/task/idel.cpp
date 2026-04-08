#include "task/idel.hpp"
#include "robot.hpp"
#include <chrono>

IdelTask::IdelTask(Robot* context,const std::string name) : BaseTask(context,name)
{
    declare_parameters_if_needed();
}

IdelTask::~IdelTask()
{

}

void IdelTask::declare_parameters_if_needed()
{
    if (parameters_declared_ || !robot || !robot->node_) {
        return;
    }

    robot->node_->declare_parameter<std::string>("idel.next_task", "idel");
    parameters_declared_ = true;
}

std::string IdelTask::process(const std::string last_task_name)
{
    (void)last_task_name;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return "idel";
}
