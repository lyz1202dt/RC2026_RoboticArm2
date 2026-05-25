// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 移动任务测试节点
//
// 本文件实现了一个 ROS2 测试客户端节点，用于测试机械臂的移动任务功能。
// 支持关节空间移动(mode=0)和笛卡尔空间移动(mode=1)两种模式。
//
// data 格式:
//   mode=0 (关节空间): [0, j1, j2, j3, j4, j5, j6, duration, (pump)]
//   mode=1 (笛卡尔空间-仅位置): [1, x, y, z, duration, (pump)]
//   mode=1 (笛卡尔空间-位置+姿态): [1, x, y, z, roll, pitch, yaw, duration, (pump)]
//
// 使用方法：
//   1. 确保 robotic_task 动作服务器已启动
//   2. 启动本节点：ros2 run arm_task move_kfs_test

#include <chrono>
#include <array>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <robot_interfaces/msg/arm.hpp>

using namespace std::chrono_literals;

namespace {
constexpr int32_t kMoveTaskId = 1;
constexpr size_t kJointCount = 6;
constexpr std::chrono::seconds kJointStateWaitTimeout(5);
}  // namespace

class MoveKfsTestNode : public rclcpp::Node {
public:
    using ArmTask = robot_interfaces::action::ArmTask;
    using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

    MoveKfsTestNode()
        : Node("move_kfs_test_node") {
        action_client_ = rclcpp_action::create_client<ArmTask>(this, "robotic_task");
        joint_state_sub_ = this->create_subscription<robot_interfaces::msg::Arm>(
            "myjoints_state", rclcpp::SensorDataQoS(),
            std::bind(&MoveKfsTestNode::on_joint_state, this, std::placeholders::_1));
        startup_timer_ = this->create_wall_timer(500ms, std::bind(&MoveKfsTestNode::run_once, this));
    }

private:
    static bool read_or_default(const std::string& prompt, double default_value, double& output_value) {
        std::cout << prompt << " (默认 " << default_value << ", 直接回车使用默认): " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            return false;
        }

        if (line.empty()) {
            output_value = default_value;
            return true;
        }

        std::istringstream iss(line);
        double parsed_value = 0.0;
        char extra = '\0';
        if (!(iss >> parsed_value) || (iss >> extra)) {
            return false;
        }

        output_value = parsed_value;
        return true;
    }

    void run_once() {
        if (request_started_) {
            return;
        }
        if (!action_server_ready_) {
            if (!action_client_->wait_for_action_server(0s)) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000, "等待动作服务 robotic_task 就绪...");
                return;
            }

            action_server_ready_ = true;
            joint_wait_start_ = this->now();
            RCLCPP_INFO(this->get_logger(), "动作服务 robotic_task 已就绪，开始等待当前关节状态");
        }

        if (!has_joint_state_) {
            const bool wait_timeout = (this->now() - joint_wait_start_) >= rclcpp::Duration::from_seconds(kJointStateWaitTimeout.count());
            if (!wait_timeout) {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000, "等待 myjoints_state 当前关节状态...");
                return;
            }

            if (!joint_state_timeout_logged_) {
                RCLCPP_WARN(this->get_logger(), "5秒内未收到 myjoints_state，默认关节角回退为 0.0");
                joint_state_timeout_logged_ = true;
            }
        }

        request_started_ = true;
        startup_timer_->cancel();

        std::array<double, kJointCount> default_joints{};
        if (has_joint_state_) {
            default_joints = current_joint_rads_;
        }

        // ---- 选择模式 ----
        double mode = 0.0;
        std::cout << "\n========== 移动任务模式选择 ==========\n";
        std::cout << "  0 - 关节空间移动\n";
        std::cout << "  1 - 笛卡尔空间移动\n";
        std::cout << "========================================\n";
        if (!read_or_default("请选择模式(0/1)", 0.0, mode)) {
            RCLCPP_ERROR(this->get_logger(), "读取模式失败，输入必须是 0 或 1");
            rclcpp::shutdown();
            return;
        }
        int imode = static_cast<int>(mode);
        if (imode != 0 && imode != 1) {
            RCLCPP_ERROR(this->get_logger(), "模式只能为 0(关节空间) 或 1(笛卡尔空间)");
            rclcpp::shutdown();
            return;
        }

        ArmTask::Goal goal_msg;
        goal_msg.task_id = kMoveTaskId;

        if (imode == 0) {
            // ---- 关节空间模式 ----
            double joint_1 = default_joints[0];
            double joint_2 = default_joints[1];
            double joint_3 = default_joints[2];
            double joint_4 = default_joints[3];
            double joint_5 = default_joints[4];
            double joint_6 = default_joints[5];

            if (has_joint_state_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "默认关节角使用当前状态: joints=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
                    joint_1, joint_2, joint_3, joint_4, joint_5, joint_6);
            } else {
                RCLCPP_INFO(this->get_logger(), "默认关节角使用回退值: joints=(0.000, 0.000, 0.000, 0.000, 0.000, 0.000)");
            }

            if (!read_or_default("请输入关节1角度(弧度)", joint_1, joint_1) ||
                !read_or_default("请输入关节2角度(弧度)", joint_2, joint_2) ||
                !read_or_default("请输入关节3角度(弧度)", joint_3, joint_3) ||
                !read_or_default("请输入关节4角度(弧度)", joint_4, joint_4) ||
                !read_or_default("请输入关节5角度(弧度)", joint_5, joint_5) ||
                !read_or_default("请输入关节6角度(弧度)", joint_6, joint_6)) {
                RCLCPP_ERROR(this->get_logger(), "读取关节角度失败，输入必须是数字或空行");
                rclcpp::shutdown();
                return;
            }

            double move_duration = 3.0;
            if (!read_or_default("请输入移动时长(秒)", 3.0, move_duration)) {
                RCLCPP_ERROR(this->get_logger(), "读取移动时长失败");
                rclcpp::shutdown();
                return;
            }

            double pump_switch = 0.0;
            if (!read_or_default("请输入气泵开关(0关,1开,3自动)", 0.0, pump_switch)) {
                RCLCPP_ERROR(this->get_logger(), "读取气泵开关失败");
                rclcpp::shutdown();
                return;
            }
            if (!(pump_switch == 0.0 || pump_switch == 1.0 || pump_switch == 2.0 || pump_switch == 3.0)) {
                RCLCPP_ERROR(this->get_logger(), "气泵开关只能为 0(关) 或 1(开) 或 2(半自动) 或 3(自动)");
                rclcpp::shutdown();
                return;
            }

            // [0, j1, j2, j3, j4, j5, j6, duration, pump]
            goal_msg.data = {
                0.0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,
                move_duration, pump_switch,
            };

            RCLCPP_INFO(
                this->get_logger(),
                "发送关节空间移动请求: mode=0, joints=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f), duration=%.3f, pump=%d",
                joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,
                move_duration, static_cast<int>(pump_switch));

        } else {
            // ---- 笛卡尔空间模式 ----
            double x = 0.0, y = 0.0, z = 0.0;

            if (!read_or_default("请输入目标位置X(米)", 0.0, x) ||
                !read_or_default("请输入目标位置Y(米)", 0.0, y) ||
                !read_or_default("请输入目标位置Z(米)", 0.0, z)) {
                RCLCPP_ERROR(this->get_logger(), "读取目标位置失败");
                rclcpp::shutdown();
                return;
            }

            // 询问是否输入姿态欧拉角
            double input_ori = 0.0;
            std::cout << "\n是否输入目标姿态欧拉角？\n";
            std::cout << "  0 - 不输入，使用当前末端姿态\n";
            std::cout << "  1 - 手动输入欧拉角(roll, pitch, yaw，单位：弧度)\n";
            if (!read_or_default("请选择(0/1)", 0.0, input_ori)) {
                RCLCPP_ERROR(this->get_logger(), "读取姿态选项失败");
                rclcpp::shutdown();
                return;
            }

            if (static_cast<int>(input_ori) == 1) {
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                if (!read_or_default("请输入roll(弧度)", 0.0, roll) ||
                    !read_or_default("请输入pitch(弧度)", 0.0, pitch) ||
                    !read_or_default("请输入yaw(弧度)", 0.0, yaw)) {
                    RCLCPP_ERROR(this->get_logger(), "读取欧拉角失败");
                    rclcpp::shutdown();
                    return;
                }

                double move_duration = 3.0;
                if (!read_or_default("请输入移动时长(秒)", 3.0, move_duration)) {
                    rclcpp::shutdown();
                    return;
                }

                double pump_switch = 0.0;
                if (!read_or_default("请输入气泵开关(0关,1开,3自动)", 0.0, pump_switch)) {
                    rclcpp::shutdown();
                    return;
                }
                if (!(pump_switch == 0.0 || pump_switch == 1.0 || pump_switch == 2.0 || pump_switch == 3.0)) {
                    RCLCPP_ERROR(this->get_logger(), "气泵开关只能为 0(关) 或 1(开) 或 2(半自动) 或 3(自动)");
                    rclcpp::shutdown();
                    return;
                }

                // [1, x, y, z, roll, pitch, yaw, duration, pump]
                goal_msg.data = {
                    1.0, x, y, z, roll, pitch, yaw,
                    move_duration, pump_switch,
                };

                RCLCPP_INFO(
                    this->get_logger(),
                    "发送笛卡尔空间移动请求(含姿态): mode=1, pos=(%.3f, %.3f, %.3f), rpy=(roll=%.3f, pitch=%.3f, yaw=%.3f), duration=%.3f, pump=%d",
                    x, y, z, roll, pitch, yaw,
                    move_duration, static_cast<int>(pump_switch));

            } else {
                double move_duration = 3.0;
                if (!read_or_default("请输入移动时长(秒)", 3.0, move_duration)) {
                    rclcpp::shutdown();
                    return;
                }

                double pump_switch = 0.0;
                if (!read_or_default("请输入气泵开关(0关,1开,3自动)", 0.0, pump_switch)) {
                    rclcpp::shutdown();
                    return;
                }
                if (!(pump_switch == 0.0 || pump_switch == 1.0 || pump_switch == 2.0 || pump_switch == 3.0)) {
                    RCLCPP_ERROR(this->get_logger(), "气泵开关只能为 0(关) 或 1(开) 或 2(半自动) 或 3(自动)");
                    rclcpp::shutdown();
                    return;
                }

                // [1, x, y, z, duration, pump]
                goal_msg.data = {
                    1.0, x, y, z,
                    move_duration, pump_switch,
                };

                RCLCPP_INFO(
                    this->get_logger(),
                    "发送笛卡尔空间移动请求(使用当前姿态): mode=1, pos=(%.3f, %.3f, %.3f), duration=%.3f, pump=%d",
                    x, y, z,
                    move_duration, static_cast<int>(pump_switch));
            }
        }

        rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&MoveKfsTestNode::on_goal_response, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MoveKfsTestNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveKfsTestNode::on_result, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_goal_response(const GoalHandleArmTask::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "移动目标被服务器拒绝");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "移动目标已被接受，等待执行结果");
    }

    void on_feedback(
        const GoalHandleArmTask::SharedPtr&,
        const std::shared_ptr<const ArmTask::Feedback>& feedback) {
        RCLCPP_INFO(this->get_logger(), "动作反馈: %s", feedback->describe.c_str());
    }

    void on_result(const GoalHandleArmTask::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    this->get_logger(), "移动动作成功: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    this->get_logger(), "移动动作失败: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(
                    this->get_logger(), "移动动作被取消: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "移动动作返回了未知结果码");
                break;
        }

        rclcpp::shutdown();
    }

    void on_joint_state(const std::shared_ptr<const robot_interfaces::msg::Arm>& msg) {
        for (size_t i = 0; i < kJointCount; ++i) {
            current_joint_rads_[i] = msg->motor[i].rad;
        }

        if (!has_joint_state_) {
            RCLCPP_INFO(this->get_logger(), "已收到当前关节状态，将作为默认关节角");
        }
        has_joint_state_ = true;
    }

    rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    std::array<double, kJointCount> current_joint_rads_{};
    rclcpp::Time joint_wait_start_;
    bool action_server_ready_{false};
    bool has_joint_state_{false};
    bool joint_state_timeout_logged_{false};
    bool request_started_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveKfsTestNode>();
    rclcpp::spin(node);
    return 0;
}