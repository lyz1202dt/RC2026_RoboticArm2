#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace {
constexpr int32_t kCatchTaskId = 2;
}

class CatchKfsTestNode : public rclcpp::Node {
public:
    using ArmTask = robot_interfaces::action::ArmTask;
    using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

    CatchKfsTestNode()
        : Node("catch_kfs_test_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        action_client_ = rclcpp_action::create_client<ArmTask>(this, "robotic_task");
        startup_timer_ = this->create_wall_timer(500ms, std::bind(&CatchKfsTestNode::run_once, this));
    }

private:
    void run_once() {
        if (request_started_) {
            return;
        }
        request_started_ = true;
        startup_timer_->cancel();

        if (!action_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "等待动作服务 robotic_task 超时");
            rclcpp::shutdown();
            return;
        }

        geometry_msgs::msg::TransformStamped target_tf;
        try {
            if (!tf_buffer_.canTransform("base_link", "target_object", tf2::TimePointZero, 10s)) {
                RCLCPP_ERROR(this->get_logger(), "等待 TF base_link -> target_object 超时");
                rclcpp::shutdown();
                return;
            }
            target_tf = tf_buffer_.lookupTransform("base_link", "target_object", tf2::TimePointZero);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "读取 TF 失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        ArmTask::Goal goal_msg;
        goal_msg.task_id = kCatchTaskId;
        goal_msg.data = {
            target_tf.transform.translation.x,
            target_tf.transform.translation.y,
            target_tf.transform.translation.z,
            target_tf.transform.rotation.x,
            target_tf.transform.rotation.y,
            target_tf.transform.rotation.z,
            target_tf.transform.rotation.w,
        };

        RCLCPP_INFO(
            this->get_logger(),
            "发送抓取请求: task_id=%d, target=(%.3f, %.3f, %.3f), quat=(%.3f, %.3f, %.3f, %.3f)",
            goal_msg.task_id,
            goal_msg.data[0],
            goal_msg.data[1],
            goal_msg.data[2],
            goal_msg.data[3],
            goal_msg.data[4],
            goal_msg.data[5],
            goal_msg.data[6]);

        rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&CatchKfsTestNode::on_goal_response, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&CatchKfsTestNode::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CatchKfsTestNode::on_result, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_goal_response(const GoalHandleArmTask::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "抓取目标被服务器拒绝");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "抓取目标已被接受，等待执行结果");
    }

    void on_feedback(
        GoalHandleArmTask::SharedPtr,
        const std::shared_ptr<const ArmTask::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "动作反馈: %s", feedback->describe.c_str());
    }

    void on_result(const GoalHandleArmTask::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(
                    this->get_logger(), "抓取动作成功: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(
                    this->get_logger(), "抓取动作失败: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(
                    this->get_logger(), "抓取动作被取消: err_code=%d, reason=%s",
                    result.result->err_code, result.result->reason.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "抓取动作返回了未知结果码");
                break;
        }

        rclcpp::shutdown();
    }

    rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    bool request_started_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CatchKfsTestNode>();
    rclcpp::spin(node);
    return 0;
}
