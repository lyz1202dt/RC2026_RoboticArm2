#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <robot_interfaces/msg/arm4.hpp>
#include <thread>



using namespace std::chrono_literals;



ArmNode::ArmNode()
    : Node("arm_node") {   

         exit_thread = false;
    
   
    //arm_pub = this->create_publisher<robot_interfaces::msg::Arm>("arm_status", 10);

    arm_sub = this->create_subscription<robot_interfaces::msg::Arm>(
        "myjoints_target", 10, std::bind(&ArmNode::armSubscribCb, this, std::placeholders::_1));


    

     cdc_trans = std::make_unique<CDCTrans>();  

     /*                         // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        // RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(state_pack_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const state_pack_t* pack = reinterpret_cast<const state_pack_t*>(data);
            if (pack->pack_type == 0)         // 确认包类型正确
                publishArmState(pack);        // 一旦接收，立即发布狗臂状态
            else
                RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack->pack_type);
        }
    });
      */

    if (!cdc_trans->open(0x0483, 0x5740))     // 开启USB_CDC传输接口
        exit_thread = true;

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do {
            cdc_trans->process_once();
        } while (!exit_thread);
    });

    base_time=this->get_clock()->now();
}




ArmNode::~ArmNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

/*
void ArmNode::publishArmState(const state_pack_t *arm_state){
    robot_interfaces::msg::Arm msg;
    msg.servo2.low=arm_state->servo2.low;
    msg.servo2.up=arm_state->servo2.up;
    msg.rob01.rad=arm_state->robstride01.state.rad;
    msg.rob01.omega=arm_state->robstride01.state.omega;
    msg.rob01.torque=arm_state->robstride01.state.torque;
    msg.rob02.rad=arm_state->GM6020.Angle_DEG;
    msg.rob02.omega=arm_state->GM6020.Speed;
    msg.rob02.torque=arm_state->GM6020.TorqueCurrent;

    arm_pub->publish(msg);
     
}
*/


void ArmNode::armSubscribCb(const robot_interfaces::msg::Arm& msg) {
   
    arm_target.servo1.up=msg.motor[3].rad;
    arm_target.servo1.low=msg.motor[2].rad;
    arm_target.rob01.except_pos=-msg.motor[1].rad;
    arm_target.rob02.target_pos=msg.motor[0].rad;
    arm_target.rob02.target_pos=arm_target.rob02.target_pos+0.0f;
    arm_target.pack_type=0x01; // 0x01 代表这是一个机械臂目标数据包    
    
    cdc_trans->send_struct(arm_target); // 一旦订阅到最新的包，立即发送到下位机

    target_log_print_cnt++;
    if (target_log_update_cnt/4 == target_log_print_cnt) {
        target_log_print_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "订阅到电机目标值 %f %f %f %f",
            arm_target.rob02.target_pos,
            arm_target.rob01.except_pos,
            arm_target.servo1.low,
            arm_target.servo1.up);
    }

    first_update = false;
}

