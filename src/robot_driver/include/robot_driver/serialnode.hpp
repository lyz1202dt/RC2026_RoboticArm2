// Copyright 2026 RoboticArm Project Authors
// SPDX-License-Identifier: Apache-2.0
//
// ROS2 节点，负责上位机与下位机之间的双向数据通信。
// 该节点通过 USB CDC 协议与下位机通信，订阅机械臂目标状态并发送给下位机，
// 同时接收下位机上报的实际状态并发布到 ROS2 话题。

#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <cdc_trans.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <data_pack.h>

// SerialNode 是一个 ROS2 节点，负责上位机与下位机之间的双向数据通信。
//
// 该节点使用 USB CDC（虚拟串口）协议与下位机通信，实现以下功能：
// 1. 订阅机械臂目标状态话题，将目标位置、速度、力矩发送给下位机
// 2. 接收下位机上报的电机实际状态，发布到 ROS2 话题
// 3. 支持运行时动态参数配置（如气泵开关）
//
// 该节点在独立的线程中处理 USB CDC 事件，确保实时性和稳定性。
//
// 话题:
//   - 订阅: "myjoints_target" (robot_interfaces::msg::Arm)
//     机械臂目标状态，包含6个关节的位置、速度、力矩
//   - 发布: "myjoints_state" (robot_interfaces::msg::Arm)
//     机械臂实际状态，包含6个关节的位置、速度、力矩
//
// 参数:
//   - enable_air_pump (bool): 气泵使能开关，默认为 false
//
// 使用示例:
//   SerialNode node;
//   rclcpp::spin(node);
class SerialNode : public rclcpp::Node
{
public:
    // 构造函数，初始化 ROS2 节点、USB CDC 通信、发布者和订阅者。
    SerialNode();

    // 析构函数，安全关闭 USB CDC 通信并等待线程结束。
    ~SerialNode();

private:
    // 线程退出标志，用于通知 USB 事件处理线程退出。
    bool exit_thread;

    // 机械臂目标状态的订阅回调函数。
    //
    // 接收机械臂目标状态消息，将其打包为 ArmTarget_t 结构体，
    // 并通过 USB CDC 发送给下位机。
    //
    // Args:
    //   msg: 机械臂目标状态消息
    void legsSubscribCb(const robot_interfaces::msg::Arm &msg);

    // 发布机械臂实际状态。
    //
    // 将下位机上报的 ArmState_t 结构体转换为 ROS2 消息并发布。
    //
    // Args:
    //   arm_state: 机械臂实际状态数据指针
    void publishLegState(const ArmState_t *arm_state);

    // USB CDC 传输对象，负责与下位机的 USB 通信。
    std::unique_ptr<CDCTrans> cdc_trans;

    // USB 事件处理线程，独立处理 USB CDC 事件。
    std::unique_ptr<std::thread> usb_event_handle_thread;

    // 机械臂实际状态的发布者。
    rclcpp::Publisher<robot_interfaces::msg::Arm>::SharedPtr joint_publisher;

    // 机械臂目标状态的订阅者。
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_subscriber;

    // ROS2 参数服务器回调句柄，用于处理动态参数变更。
    OnSetParametersCallbackHandle::SharedPtr param_server_handle;

    // 机械臂目标状态数据包，待发送给下位机。
    ArmTarget_t arm_target;

    // 关节位置缓存（暂未使用）。
    std::vector<double> joint_pos;

    // 关节速度缓存（暂未使用）。
    std::vector<double> joint_vel;

    // 订阅计数阈值，用于控制日志输出频率。
    int subscrib_cnt{20};

    // 发布计数阈值，用于控制日志输出频率。
    int publish_cnt{100};

    // 当前订阅计数。
    int cur_sub_cnt{0};

    // 当前发布计数。
    int cur_pub_cnt{0};

    // 气泵使能标志，由参数 "enable_air_pump" 控制。
    bool enable_air_pump{false};
};

#endif