// 负责上位机和下位机双向数据通信

#include "robot_interfaces/msg/arm.hpp" // 机械臂数据结构
#include "sensor_msgs/msg/joint_state.hpp" // 关节状态
#include <chrono>
#include <rclcpp/parameter.hpp> // 在运行时动态修改节点配置
#include <rclcpp/time.hpp>
#include <serialnode.hpp>

using namespace std::chrono_literals;

SerialNode::SerialNode()
    : Node("driver_node") {

    arm_target.pack_type = 1; // 初始化目标数据包的消息类型为1

    this->declare_parameter<bool>("enable_air_pump", false); // 使能气泵

    // 参数变化的回调函数
    param_server_handle = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& param) {
        rcl_interfaces::msg::SetParametersResult result;
        (void)param;
        result.successful = true;
        RCLCPP_INFO(this->get_logger(), "参数更新");
        return result;
    });

    // 初始化状态
    exit_thread = false;

    // 先创建 publisher/subscriber，确保回调中 publish 时 publisher 已就绪
    // joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_publisher = this->create_publisher<robot_interfaces::msg::Arm>("myjoints_state", 10);

    joint_subscriber = this->create_subscription<robot_interfaces::msg::Arm>(
        "myjoints_target", 10, std::bind(&SerialNode::legsSubscribCb, this, std::placeholders::_1)
    );


    cdc_trans = std::make_unique<CDCTrans>();                           // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        // RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(Arm_t))        // 验证包长度，可以被视作四条腿的状态数据包
        {
            const Arm_t* pack = reinterpret_cast<const Arm_t*>(data);
            if (pack->pack_type == 1)     // 确认包类型正确
                publishLegState(pack);
        }
    });
    if (!cdc_trans->open(0x0483, 0x5740)) // 开启USB_CDC传输接口
    {
        RCLCPP_ERROR(get_logger(), "串口打开失败，无法驱动物理机械臂！");
        exit_thread = true;
    }

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do {
            cdc_trans->process_once();
        } while (!exit_thread);
    });
}

SerialNode::~SerialNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}

void SerialNode::publishLegState(const Arm_t* arm_state) {
    robot_interfaces::msg::Arm msg;
    for (int i = 0; i < 6; i++) {
        msg.motor[i].rad    = arm_state->joints[i].rad;
        msg.motor[i].omega  = arm_state->joints[i].omega;
        msg.motor[i].torque = arm_state->joints[i].torque;
    }
    msg.motor[5].rad = arm_target.joints[5].rad; // 欺骗Moveit认为不存在的关节6到达目标位置
    joint_publisher->publish(msg);

    cur_pub_cnt++;
    if (cur_pub_cnt == publish_cnt) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50, "发布电机状态");
        cur_pub_cnt = 0;
    }
}

void SerialNode::legsSubscribCb(const robot_interfaces::msg::Arm& msg) {
    for (int i = 0; i < 6; i++) {
        arm_target.joints[i].rad    = msg.motor[i].rad;
        arm_target.joints[i].omega  = msg.motor[i].omega;
        arm_target.joints[i].torque = msg.motor[i].torque;
    }
    cdc_trans->send_struct(arm_target); // 一旦订阅到最新的包，立即发送到下位机（下位机用定时器保证匀速率发送方便断开检测）
}
