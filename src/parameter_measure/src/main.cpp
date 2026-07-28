#include "parameter_measure_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node=std::make_shared<rclcpp::Node>("arm_measure");
  ParameterMeasure measure(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
