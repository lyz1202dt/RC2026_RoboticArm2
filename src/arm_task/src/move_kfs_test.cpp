
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

// 移动任务测试 GUI 节点
//
// 本文件实现了一个 ROS2 + Qt 的测试客户端窗口，用于测试机械臂的移动任务功能。
// 支持关节空间移动(mode=0)和笛卡尔空间移动(mode=1)两种模式。
//
// data 格式:
//   mode=0 (关节空间): [0, j1, j2, j3, j4, j5, j6, duration, (pump)]
//   mode=1 (笛卡尔空间-仅位置): [1, x, y, z, duration, (pump)]
//   mode=1 (笛卡尔空间-位置+姿态): [1, x, y, z, roll, pitch, yaw, duration, (pump)]

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <robot_interfaces/action/arm_task.hpp>
#include <robot_interfaces/msg/arm.hpp>

using namespace std::chrono_literals;

namespace {
constexpr int32_t kMoveTaskId = 1;
constexpr size_t kJointCount = 6;
constexpr double kValueRange = 1000.0;
constexpr double kSpinStep = 0.01;
constexpr int kSpinDecimals = 3;
constexpr int kSpinIntervalMs = 50;
constexpr int kJointStateTimeoutMs = 5000;

}  // namespace

class MoveKfsGui : public QWidget {
public:
    using ArmTask = robot_interfaces::action::ArmTask;
    using GoalHandleArmTask = rclcpp_action::ClientGoalHandle<ArmTask>;

    MoveKfsGui()
        : QWidget(nullptr),
          node_(std::make_shared<rclcpp::Node>("move_kfs_test_node")),
          action_client_(rclcpp_action::create_client<ArmTask>(node_, "robotic_task")) {
        setWindowTitle("move_kfs_test");
        resize(720, 640);
        build_ui();

        joint_state_sub_ = node_->create_subscription<robot_interfaces::msg::Arm>(
            "myjoints_state", rclcpp::SensorDataQoS(),
            std::bind(&MoveKfsGui::on_joint_state, this, std::placeholders::_1));

        executor_.add_node(node_);

        spin_timer_ = new QTimer(this);
        connect(spin_timer_, &QTimer::timeout, this, [this]() {
            executor_.spin_some();
            check_action_server();
            check_joint_state_timeout();
        });
        spin_timer_->start(kSpinIntervalMs);

        joint_wait_timer_.start();
        append_log("等待动作服务 robotic_task 就绪...");
    }

    ~MoveKfsGui() override = default;

protected:
    void closeEvent(QCloseEvent* event) override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        QWidget::closeEvent(event);
    }

private:
    static void configure_spinbox(QDoubleSpinBox* spinbox, double value) {
        spinbox->setRange(-kValueRange, kValueRange);
        spinbox->setDecimals(kSpinDecimals);
        spinbox->setSingleStep(kSpinStep);
        spinbox->setValue(value);
    }

    void build_ui() {
        auto* main_layout = new QVBoxLayout(this);

        status_label_ = new QLabel("动作服务：等待中", this);
        joint_state_label_ = new QLabel("关节状态：等待中", this);
        main_layout->addWidget(status_label_);
        main_layout->addWidget(joint_state_label_);

        auto* mode_layout = new QHBoxLayout();
        auto* mode_label = new QLabel("移动模式:", this);
        mode_combo_ = new QComboBox(this);
        mode_combo_->addItem("关节空间");
        mode_combo_->addItem("笛卡尔空间");
        mode_layout->addWidget(mode_label);
        mode_layout->addWidget(mode_combo_);
        main_layout->addLayout(mode_layout);

        mode_stack_ = new QStackedWidget(this);
        mode_stack_->addWidget(build_joint_panel());
        mode_stack_->addWidget(build_cart_panel());
        main_layout->addWidget(mode_stack_);

        auto* common_group = new QGroupBox("公共参数", this);
        auto* common_layout = new QFormLayout(common_group);
        duration_spin_ = new QDoubleSpinBox(this);
        configure_spinbox(duration_spin_, 3.0);
        duration_spin_->setMinimum(0.0);
        common_layout->addRow("移动时长(秒)", duration_spin_);

        pump_combo_ = new QComboBox(this);
        pump_combo_->addItem("0 - 关", 0);
        pump_combo_->addItem("1 - 开", 1);
        pump_combo_->addItem("2 - 半自动", 2);
        pump_combo_->addItem("3 - 自动", 3);
        common_layout->addRow("气泵开关", pump_combo_);
        main_layout->addWidget(common_group);

        start_button_ = new QPushButton("启动", this);
        start_button_->setEnabled(false);
        connect(start_button_, &QPushButton::clicked, this, [this]() { on_start_clicked(); });
        main_layout->addWidget(start_button_);

        auto* log_group = new QGroupBox("日志", this);
        auto* log_layout = new QVBoxLayout(log_group);
        log_text_ = new QPlainTextEdit(this);
        log_text_->setReadOnly(true);
        log_layout->addWidget(log_text_);
        main_layout->addWidget(log_group);

        connect(mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this](int index) { update_mode_ui(index); });
    }

    QWidget* build_joint_panel() {
        auto* group = new QGroupBox("关节空间目标", this);
        auto* layout = new QFormLayout(group);

        for (size_t i = 0; i < kJointCount; ++i) {
            auto* spin = new QDoubleSpinBox(this);
            configure_spinbox(spin, 0.0);
            layout->addRow(QString("关节%1(弧度)").arg(static_cast<int>(i + 1)), spin);
            joint_spins_[i] = spin;
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
                if (!setting_joint_defaults_) {
                    joint_inputs_edited_ = true;
                }
            });
        }

        return group;
    }

    QWidget* build_cart_panel() {
        auto* group = new QGroupBox("笛卡尔空间目标", this);
        auto* layout = new QFormLayout(group);

        x_spin_ = new QDoubleSpinBox(this);
        y_spin_ = new QDoubleSpinBox(this);
        z_spin_ = new QDoubleSpinBox(this);
        configure_spinbox(x_spin_, 0.0);
        configure_spinbox(y_spin_, 0.0);
        configure_spinbox(z_spin_, 0.0);
        layout->addRow("X(米)", x_spin_);
        layout->addRow("Y(米)", y_spin_);
        layout->addRow("Z(米)", z_spin_);

        rpy_checkbox_ = new QCheckBox("输入姿态欧拉角 (roll/pitch/yaw)", this);
        layout->addRow(rpy_checkbox_);

        rpy_widget_ = new QWidget(this);
        auto* rpy_layout = new QFormLayout(rpy_widget_);
        roll_spin_ = new QDoubleSpinBox(this);
        pitch_spin_ = new QDoubleSpinBox(this);
        yaw_spin_ = new QDoubleSpinBox(this);
        configure_spinbox(roll_spin_, 0.0);
        configure_spinbox(pitch_spin_, 0.0);
        configure_spinbox(yaw_spin_, 0.0);
        rpy_layout->addRow("Roll(弧度)", roll_spin_);
        rpy_layout->addRow("Pitch(弧度)", pitch_spin_);
        rpy_layout->addRow("Yaw(弧度)", yaw_spin_);
        rpy_widget_->setVisible(false);
        layout->addRow(rpy_widget_);

        connect(rpy_checkbox_, &QCheckBox::toggled, this, [this](bool checked) {
            rpy_widget_->setVisible(checked);
        });

        return group;
    }

    void update_mode_ui(int index) {
        mode_stack_->setCurrentIndex(index);
    }

    void check_action_server() {
        if (action_server_ready_) {
            return;
        }
        if (action_client_->wait_for_action_server(0s)) {
            action_server_ready_ = true;
            start_button_->setEnabled(true);
            status_label_->setText("动作服务：已就绪");
            append_log("动作服务 robotic_task 已就绪");
        }
    }

    void check_joint_state_timeout() {
        if (joint_state_ready_ || joint_wait_timeout_logged_) {
            return;
        }
        if (joint_wait_timer_.elapsed() >= kJointStateTimeoutMs) {
            joint_wait_timeout_logged_ = true;
            joint_defaults_applied_ = true;
            joint_state_label_->setText("关节状态：超时未收到，默认 0.0");
            append_log("5秒内未收到 myjoints_state，默认关节角回退为 0.0");
        }
    }

    void apply_joint_defaults(const std::array<double, kJointCount>& joints) {
        setting_joint_defaults_ = true;
        for (size_t i = 0; i < kJointCount; ++i) {
            joint_spins_[i]->setValue(joints[i]);
        }
        setting_joint_defaults_ = false;
    }

    void on_joint_state(const robot_interfaces::msg::Arm::ConstSharedPtr& msg) {
        for (size_t i = 0; i < kJointCount; ++i) {
            current_joint_rads_[i] = msg->motor[i].rad;
        }
        if (!joint_state_ready_) {
            append_log("已收到当前关节状态，将作为默认关节角");
        }
        joint_state_ready_ = true;
        joint_state_label_->setText("关节状态：已收到");

        if (!joint_defaults_applied_ && !joint_inputs_edited_) {
            apply_joint_defaults(current_joint_rads_);
            joint_defaults_applied_ = true;
        }
    }

    void on_start_clicked() {
        if (!action_server_ready_) {
            append_log("动作服务未就绪，无法发送请求");
            return;
        }

        ArmTask::Goal goal_msg;
        goal_msg.task_id = kMoveTaskId;

        const int mode_index = mode_combo_->currentIndex();
        const double duration = duration_spin_->value();
        const double pump_switch = static_cast<double>(pump_combo_->currentData().toInt());

        if (mode_index == 0) {
            const double joint_1 = joint_spins_[0]->value();
            const double joint_2 = joint_spins_[1]->value();
            const double joint_3 = joint_spins_[2]->value();
            const double joint_4 = joint_spins_[3]->value();
            const double joint_5 = joint_spins_[4]->value();
            const double joint_6 = joint_spins_[5]->value();

            goal_msg.data = {
                0.0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,
                duration, pump_switch,
            };

            append_log(QString(
                "发送关节空间移动请求: joints=(%1, %2, %3, %4, %5, %6), duration=%7, pump=%8")
                           .arg(joint_1, 0, 'f', 3)
                           .arg(joint_2, 0, 'f', 3)
                           .arg(joint_3, 0, 'f', 3)
                           .arg(joint_4, 0, 'f', 3)
                           .arg(joint_5, 0, 'f', 3)
                           .arg(joint_6, 0, 'f', 3)
                           .arg(duration, 0, 'f', 3)
                           .arg(static_cast<int>(pump_switch)));
        } else {
            const double x = x_spin_->value();
            const double y = y_spin_->value();
            const double z = z_spin_->value();

            if (rpy_checkbox_->isChecked()) {
                const double roll = roll_spin_->value();
                const double pitch = pitch_spin_->value();
                const double yaw = yaw_spin_->value();
                goal_msg.data = {
                    1.0, x, y, z, roll, pitch, yaw,
                    duration, pump_switch,
                };
                append_log(QString(
                    "发送笛卡尔空间移动请求(含姿态): pos=(%1, %2, %3), rpy=(%4, %5, %6), duration=%7, pump=%8")
                               .arg(x, 0, 'f', 3)
                               .arg(y, 0, 'f', 3)
                               .arg(z, 0, 'f', 3)
                               .arg(roll, 0, 'f', 3)
                               .arg(pitch, 0, 'f', 3)
                               .arg(yaw, 0, 'f', 3)
                               .arg(duration, 0, 'f', 3)
                               .arg(static_cast<int>(pump_switch)));
            } else {
                goal_msg.data = {
                    1.0, x, y, z,
                    duration, pump_switch,
                };
                append_log(QString(
                    "发送笛卡尔空间移动请求(使用当前姿态): pos=(%1, %2, %3), duration=%4, pump=%5")
                               .arg(x, 0, 'f', 3)
                               .arg(y, 0, 'f', 3)
                               .arg(z, 0, 'f', 3)
                               .arg(duration, 0, 'f', 3)
                               .arg(static_cast<int>(pump_switch)));
            }
        }

        rclcpp_action::Client<ArmTask>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback =
            std::bind(&MoveKfsGui::on_goal_response, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&MoveKfsGui::on_feedback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MoveKfsGui::on_result, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void on_goal_response(const GoalHandleArmTask::SharedPtr& goal_handle) {
        if (!goal_handle) {
            append_log("移动目标被服务器拒绝");
            return;
        }
        append_log("移动目标已被接受，等待执行结果");
    }

    void on_feedback(
        const GoalHandleArmTask::SharedPtr&,
        const std::shared_ptr<const ArmTask::Feedback>& feedback) {
        append_log(QString("动作反馈: %1").arg(QString::fromStdString(feedback->describe)));
    }

    void on_result(const GoalHandleArmTask::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                append_log(QString("移动动作成功: err_code=%1, reason=%2")
                               .arg(result.result->err_code)
                               .arg(QString::fromStdString(result.result->reason)));
                break;
            case rclcpp_action::ResultCode::ABORTED:
                append_log(QString("移动动作失败: err_code=%1, reason=%2")
                               .arg(result.result->err_code)
                               .arg(QString::fromStdString(result.result->reason)));
                break;
            case rclcpp_action::ResultCode::CANCELED:
                append_log(QString("移动动作被取消: err_code=%1, reason=%2")
                               .arg(result.result->err_code)
                               .arg(QString::fromStdString(result.result->reason)));
                break;
            default:
                append_log("移动动作返回了未知结果码");
                break;
        }
    }

    void append_log(const QString& text) {
        log_text_->appendPlainText(text);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<ArmTask>::SharedPtr action_client_;
    rclcpp::Subscription<robot_interfaces::msg::Arm>::SharedPtr joint_state_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    QTimer* spin_timer_{nullptr};
    QElapsedTimer joint_wait_timer_;
    bool action_server_ready_{false};
    bool joint_state_ready_{false};
    bool joint_wait_timeout_logged_{false};
    bool joint_defaults_applied_{false};
    bool joint_inputs_edited_{false};
    bool setting_joint_defaults_{false};
    std::array<double, kJointCount> current_joint_rads_{};

    QLabel* status_label_{nullptr};
    QLabel* joint_state_label_{nullptr};
    QComboBox* mode_combo_{nullptr};
    QStackedWidget* mode_stack_{nullptr};
    std::array<QDoubleSpinBox*, kJointCount> joint_spins_{};
    QDoubleSpinBox* x_spin_{nullptr};
    QDoubleSpinBox* y_spin_{nullptr};
    QDoubleSpinBox* z_spin_{nullptr};
    QCheckBox* rpy_checkbox_{nullptr};
    QWidget* rpy_widget_{nullptr};
    QDoubleSpinBox* roll_spin_{nullptr};
    QDoubleSpinBox* pitch_spin_{nullptr};
    QDoubleSpinBox* yaw_spin_{nullptr};
    QDoubleSpinBox* duration_spin_{nullptr};
    QComboBox* pump_combo_{nullptr};
    QPushButton* start_button_{nullptr};
    QPlainTextEdit* log_text_{nullptr};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    MoveKfsGui window;
    window.show();
    const int result = app.exec();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return result;
}
