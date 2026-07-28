#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

class ParameterCalcNode : public  rclcpp::Node{
public:
    ParameterCalcNode(): rclcpp::Node("arm_param_calc_node"){

    }
private:
    std::string init_urdf_path_;
    std::string csv_path_;
    std::string recognized_csv_path_;
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ParameterCalcNode>());
    rclcpp::shutdown();
    return 0;
}