#pragma once

#include "task/base_task.hpp"

class IdelTask :public BaseTask{
public:
    IdelTask(Robot* context,const std::string name);
    ~IdelTask() override;
    std::string process(const std::string last_task_name) override;

private:
    void declare_parameters_if_needed();
    bool parameters_declared_{false};
};
