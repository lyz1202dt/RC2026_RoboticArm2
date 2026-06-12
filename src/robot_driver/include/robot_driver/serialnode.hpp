#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "cdc_trans.hpp"
#include "data_pack.h"


class ArmNode : public rclcpp::Node
{
public:
    ArmNode();
    ~ArmNode();
private:
    void jointStateCallback(const sensor_msgs::msg::JointState& msg);
    rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& params);
    void updateAirPumpTarget();
    void sendTarget();

    bool exit_thread{false};
    bool enable_air_pump{false};
    bool has_joint_target{false};

    std::unique_ptr<CDCTrans> cdc_trans;
    std::unique_ptr<std::thread> usb_event_handle_thread;
    target_pack_t arm_target{};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server;
};

#endif
