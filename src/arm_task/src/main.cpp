#include "arm_task/task.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<arm_task::ArmTaskNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
