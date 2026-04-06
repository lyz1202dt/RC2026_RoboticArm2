#include "arm_calc/arm_ctrl.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

// 命名空间：arm_calc，用于封装机械臂控制相关的类和函数
namespace arm_calc {

// 匿名命名空间：用于定义局部辅助函数和常量，避免全局污染
namespace {

// 定义常量：关节状态话题名称，用于订阅机械臂的当前关节状态
constexpr char kJointStateTopic[] = "myjoints_state";
// 定义常量：关节目标话题名称，用于发布机械臂的目标关节位置
constexpr char kJointTargetTopic[] = "myjoints_target";
// 定义常量：视觉目标话题名称，用于订阅视觉伺服的目标姿态
constexpr char kVisualTargetTopic[] = "visual_target_pose";
// 定义常量：关节空间目标话题名称，用于订阅关节空间的目标角度
constexpr char kJointSpaceTargetTopic[] = "joint_space_target";
// 定义常量：RViz关节话题名称，用于发布关节状态到RViz可视化
constexpr char kRvizJointTopic[] = "joint_states";
// 定义常量：标记话题名称，用于发布可视化标记到RViz
constexpr char kMarkerTopic[] = "visualization_marker_array";

// 辅助函数：将Eigen向量转换为ROS Point消息
geometry_msgs::msg::Point ToPoint(const Eigen::Vector3d& value) {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

// 辅助函数：将Eigen四元数转换为ROS Quaternion消息
geometry_msgs::msg::Quaternion ToMsgQuaternion(const Eigen::Quaterniond& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

// 辅助函数：构建静态关节目标点，用于预览或保持
// 参数：arm_calc - 运动学计算器，position - 目标关节位置
// 返回：JointTrajectoryPoint，包含位置、速度、加速度和扭矩
JointTrajectoryPoint BuildStaticJointTarget(const std::shared_ptr<ArmCalc>& arm_calc, const JointVector& position) {
    JointTrajectoryPoint point;
    point.position = position; // 设置目标位置
    // point.velocity.setZero();   // 速度设为零（静态）
    // point.acceleration.setZero();  // 加速度设为零
    // // 计算逆动力学扭矩，确保关节保持在目标位置
    // point.torque = arm_calc->joint_torque_inverse_dynamics(position, JointVector::Zero(), JointVector::Zero());
    return point;
}

} // namespace

// ArmCtrlNode类构造函数：初始化机械臂控制节点
// 参数：options - ROS2节点选项，用于配置节点行为
ArmCtrlNode::ArmCtrlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("arm_calc_node", options) { // 调用父类构造函数，设置节点名称
    // 声明动态可修改的参数
    declare_parameters();
    // 加载机器人描述文件并构建运动学求解器
    load_robot_description_and_build_solver();
    // 创建ROS2接口：订阅者、发布者和定时器
    create_interfaces();
    // 设置参数变化回调，用于动态更新参数
    param_callback_ = this->add_on_set_parameters_callback(std::bind(&ArmCtrlNode::on_parameters_changed, this, std::placeholders::_1));
}

// 声明动态可修改参数：这些参数可以在运行时通过ROS2参数服务修改，用于控制机械臂行为
void ArmCtrlNode::declare_parameters() {

    // 声明运动模式参数：0=空闲, 1=关节空间, 2=笛卡尔空间, 3=视觉伺服
    this->declare_parameter<int>("motion_mode", 0);

    // 声明轨迹规划总时间参数：轨迹执行的持续时间（秒）
    this->declare_parameter<double>("trajectory_duration", 3.0);

    // 声明控制周期参数：控制循环的执行间隔（秒）
    this->declare_parameter<double>("control_period", 0.02);

    // 声明是否执行轨迹参数：true=执行轨迹, false=仅预览
    this->declare_parameter<bool>("execute_trajectory", false);

    // 声明视觉伺服比例增益参数：控制视觉伺服的响应速度
    this->declare_parameter<double>("visual_servo_kp", 2.0);

    // 声明视觉伺服最大线加速度参数：限制末端执行器的最大加速度
    this->declare_parameter<double>("visual_servo_max_linear_acceleration", 0.5);

    // 声明基座连杆参数：运动学链的起始连杆名称
    this->declare_parameter<std::string>("base_link", "base_link");

    // 声明末端连杆参数：运动学链的末端连杆名称
    this->declare_parameter<std::string>("tip_link", "link6");

    // 声明关节目标参数：4个关节的目标角度（弧度）
    this->declare_parameter<std::vector<double>>("joint_target", std::vector<double>(kJointDoF, 0.0));

    // 声明笛卡尔目标位置参数：末端执行器的目标位置（x, y, z）
    this->declare_parameter<std::vector<double>>("cartesian_target_position", std::vector<double>{0.7, 0.0, 0.15});

    // 声明笛卡尔目标四元数参数：末端执行器的目标姿态（w, x, y, z）
    this->declare_parameter<std::vector<double>>("cartesian_target_quaternion", std::vector<double>{1.0, 0.0, 0.0, 0.0});
}

// 创建关键可执行函数：设置ROS2订阅者、发布者和定时器，用于与外部通信和控制循环
void ArmCtrlNode::create_interfaces() {
    // 订阅机械臂当前关节状态：从"myjoints_state"话题接收关节位置、速度、扭矩
    // joint_state_sub_ = this->create_subscription<robot_interfaces::msg::Arm>(
    //     kJointStateTopic, rclcpp::SensorDataQoS(),  // 使用传感器数据QoS，确保实时性
    //     std::bind(&ArmCtrlNode::on_joint_state, this, std::placeholders::_1));  // 绑定回调函数

    // 订阅视觉目标：从"visual_target_pose"话题接收末端执行器的目标姿态
    visual_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        kVisualTargetTopic, 10, // 队列大小10
        std::bind(&ArmCtrlNode::on_visual_target, this, std::placeholders::_1));

    // 订阅关节目标：从"joint_space_target"话题接收关节空间的目标角度
    joint_space_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        kJointSpaceTargetTopic, 10, std::bind(&ArmCtrlNode::on_joint_space_target, this, std::placeholders::_1));

    // 发布关节目标：向"myjoints_target"话题发布目标关节位置、速度、扭矩
    joint_target_pub_ = this->create_publisher<robot_interfaces::msg::Arm>(kJointTargetTopic, 10);

    // 发布RViz关节状态：向"joint_states"话题发布关节状态，用于RViz可视化
    rviz_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(kRvizJointTopic, 10);

    // 发布RViz自定义可视化：向"visualization_marker_array"话题发布标记，用于显示目标和当前姿态
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kMarkerTopic, 10);

    // 控制循环调度：创建定时器，每control_period_sec_秒调用publish_control_loop
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_period_sec_),   // 定时器间隔
        std::bind(&ArmCtrlNode::publish_control_loop, this)); // 绑定控制循环函数
}

// 初始化关键参数：加载机器人URDF并构建运动学求解器，这是节点启动的核心初始化步骤
void ArmCtrlNode::load_robot_description_and_build_solver() {
    // 获取基座和末端连杆名称，从参数中读取
    base_link_ = this->get_parameter("base_link").as_string();
    tip_link_  = this->get_parameter("tip_link").as_string();
    // 获取轨迹规划总时间
    trajectory_duration_sec_ = this->get_parameter("trajectory_duration").as_double();
    // 获取控制器运行周期
    control_period_sec_ = this->get_parameter("control_period").as_double();
    // 获取是否执行轨迹标志
    execute_trajectory_ = this->get_parameter("execute_trajectory").as_bool();
    // 获取视觉伺服比例增益，取最大值确保非负
    visual_servo_kp_ = std::max(this->get_parameter("visual_servo_kp").as_double(), 0.0);

    // 获取视觉伺服末端最大线速度，取最大值确保非负
    visual_servo_max_linear_acceleration_ = std::max(this->get_parameter("visual_servo_max_linear_acceleration").as_double(), 0.0);

    // 解析用户请求的运动模式，从参数转换为枚举
    requested_motion_mode_ = parse_motion_mode(this->get_parameter("motion_mode").as_int());

    // 初始化当前实际运动模式为闲置
    active_motion_mode_ = MotionMode::kIdle;

    // 读取每个关节目标角度，从参数中获取6个关节的目标值
    const std::vector<double> joint_target = get_double_array_param(*this, "joint_target", kJointDoF);
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        joint_target_state_.position[static_cast<int>(i)] = joint_target[i]; // 设置关节目标位置
    }

    current_joint_state_.position = joint_target_state_.position;
    has_joint_state_              = true;                                    // 启动即认为已拥有关节状态
    idle_hold_initialized_        = false;

    // 获取笛卡尔空间目标位置和姿态
    const std::vector<double> cartesian_position   = get_double_array_param(*this, "cartesian_target_position", 3);
    const std::vector<double> cartesian_quaternion = get_double_array_param(*this, "cartesian_target_quaternion", 4);
    // 设置笛卡尔目标位置
    cartesian_target_.position = Eigen::Vector3d(cartesian_position[0], cartesian_position[1], cartesian_position[2]);
    // 设置笛卡尔目标姿态，并归一化四元数
    cartesian_target_.orientation = NormalizeQuaternion(
        Eigen::Quaterniond(cartesian_quaternion[0], cartesian_quaternion[1], cartesian_quaternion[2], cartesian_quaternion[3]));
    // 初始化视觉目标为笛卡尔目标
    visual_target_ = cartesian_target_;

    // 解析URDF，建立KDL运动学链
    KDL::Tree tree;                                         // 创建KDL树结构
    const std::string urdf_xml = fetch_robot_description(); // 获取URDF XML字符串
    // 从URDF字符串解析为KDL树
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        throw std::runtime_error("failed to parse arm URDF into KDL tree"); // 解析失败抛异常
    }
    // 从树中提取运动学链，从base_link到tip_link
    if (!tree.getChain("base_link", "link5", arm_chain_)) {
        throw std::runtime_error("failed to build KDL chain from " + base_link_ + " to " + tip_link_);
    }

    // 初始化运动学求解器
    arm_calc_ = std::make_shared<ArmCalc>(arm_chain_); // 创建ArmCalc实例，基于KDL链

    // 初始化关节空间运动规划器
    joint_space_move_ = std::make_shared<arm_action::JointSpaceMove>(arm_calc_);
    // 初始化笛卡尔空间运动规划器
    cartesian_space_move_ = std::make_shared<arm_action::JCartesianSpaceMove>(arm_calc_);
    // 初始化视觉伺服运动规划器
    visual_servo_move_ = std::make_shared<arm_action::VisualServoMove>(arm_calc_);
    // 设置视觉伺服的KP增益
    visual_servo_move_->set_kp(visual_servo_kp_);
    // 设置视觉伺服的最大线加速度
    visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
}

// 获取机器人描述：尝试从/robot_state_publisher参数服务获取URDF，如果失败则从本地文件加载
std::string ArmCtrlNode::fetch_robot_description() const {
    // 创建同步参数客户端，用于从/robot_state_publisher节点获取参数
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
        const_cast<ArmCtrlNode*>(this), "/robot_state_publisher"); // 客户端指向robot_state_publisher

    // 等待服务可用，超时1秒
    if (client->wait_for_service(std::chrono::seconds(1))) {
        try {
            // 获取robot_description参数
            const auto params = client->get_parameters({"robot_description"});
            if (!params.empty()) {
                const std::string urdf_xml = params.front().as_string(); // 获取URDF字符串
                if (!urdf_xml.empty()) {
                    // 成功获取，记录日志
                    RCLCPP_INFO(this->get_logger(), "Loaded robot_description from /robot_state_publisher");
                    return urdf_xml;
                }
            }
        } catch (const std::exception& e) {
            // 获取失败，记录警告
            RCLCPP_WARN(this->get_logger(), "Failed to fetch robot_description from parameter service: %s", e.what());
        }
    }

    // 回退到本地URDF文件
    RCLCPP_WARN(this->get_logger(), "Falling back to local URDF file");
    return load_local_urdf(); // 调用本地加载函数
}

// 加载本地URDF文件：从包的share目录读取URDF文件
std::string ArmCtrlNode::load_local_urdf() const {
    // 获取arm包的share目录路径
    const std::string arm_share = ament_index_cpp::get_package_share_directory("arm");
    // 构建URDF文件路径
    const std::string urdf_path = arm_share + "/model/robotic_arm.urdf";
    // 打开文件
    std::ifstream input(urdf_path);
    if (!input.is_open()) {
        // 打开失败抛异常
        throw std::runtime_error("unable to open local URDF: " + urdf_path);
    }

    // 读取文件内容到字符串流
    std::ostringstream buffer;
    buffer << input.rdbuf(); // 将文件内容读入缓冲区
    return buffer.str();     // 返回URDF字符串
}

// 刷新规划：根据当前运动模式设置运动规划器，准备执行轨迹
// 参数：now_sec - 当前时间（秒）
void ArmCtrlNode::refresh_plan(double now_sec) {
    // 检查ArmCalc是否初始化且有关节状态
    if (!arm_calc_ || !has_joint_state_) {
        return; // 如果未初始化或无关节状态，直接返回
    }

    // 根据活动运动模式进行规划
    switch (active_motion_mode_) {
    case MotionMode::kIdle:                                                                     // 空闲模式
        enter_idle_mode();                                                                      // 进入空闲模式
        RCLCPP_INFO(get_logger(), "进入IDEL模式");                                              // 记录日志
        break;
    case MotionMode::kJointSpace:                                                               // 关节空间模式
        joint_space_move_->set_start_state(current_joint_state_);                               // 设置起点状态
        joint_space_move_->set_goal_state(joint_target_state_, trajectory_duration_sec_);       // 设置目标状态和持续时间
        if (execute_trajectory_) {                                                              // 如果执行轨迹
            joint_space_move_->start(now_sec);                                                  // 开始轨迹
        }
        RCLCPP_INFO(get_logger(), "进入关节空间轨迹执行模式");                                  // 记录日志
        break;
    case MotionMode::kCartesianSpace:                                                           // 笛卡尔空间模式
        cartesian_space_move_->set_start_state(current_joint_state_);                           // 设置起点
        cartesian_space_move_->set_goal_state(cartesian_target_, trajectory_duration_sec_);     // 设置目标姿态和时间
        if (execute_trajectory_) {                                                              // 如果执行轨迹
            cartesian_space_move_->start(now_sec);                                              // 开始轨迹
        }
        RCLCPP_INFO(get_logger(), "进入笛卡尔空间轨迹执行模式");                                // 记录日志
        break;
    case MotionMode::kVisualServo:                                                              // 视觉伺服模式
        visual_servo_move_->set_kp(visual_servo_kp_);                                           // 设置KP增益
        visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_); // 设置最大加速度
        visual_servo_move_->set_current_joint_state(current_joint_state_);                      // 设置当前关节状态
        visual_servo_move_->set_target_pose(visual_target_);                                    // 设置目标姿态
        RCLCPP_INFO(get_logger(), "进入视觉伺服模式");                                          // 记录日志
        break;
    }
    planners_ready_ = true;                                                                     // 标记规划器已准备好
}

void ArmCtrlNode::capture_idle_hold_from_current_state() {
    idle_hold_point_.position = current_joint_state_.position;                                  // 设置位置为当前关节位置
    //     idle_hold_point_.velocity.setZero();  // 速度设为零
    //     idle_hold_point_.acceleration.setZero();  // 加速度设为零
    //     // 计算逆动力学扭矩，确保保持位置
    //     idle_hold_point_.torque = arm_calc_->joint_torque_inverse_dynamics(
    //    idle_hold_point_.position, JointVector::Zero(), JointVector::Zero());
    idle_hold_initialized_ = true; // 标记空闲保持点已初始化
}

void ArmCtrlNode::set_idle_hold_point(const JointTrajectoryPoint& point) {
    idle_hold_point_ = point;      // 对当前位置进行保存
    // idle_hold_point_.velocity.setZero();  // 确保速度为零
    // idle_hold_point_.acceleration.setZero();  // 确保加速度为零
    //  重新计算扭矩
    // idle_hold_point_.torque = arm_calc_->joint_torque_inverse_dynamics(
    //   idle_hold_point_.position, JointVector::Zero(), JointVector::Zero());
    idle_hold_initialized_ = true; // 标记初始化
}

// 应用请求的模式：根据参数和状态切换运动模式
// 参数：now_sec - 当前时间
void ArmCtrlNode::apply_requested_mode(double now_sec) {
    // 检查有关节状态
    if (!has_joint_state_) {
        return; // 无状态则返回
    }

    // 如果不执行轨迹
    if (!execute_trajectory_) {
        // 如果当前不是空闲或未初始化空闲保持点
        if (active_motion_mode_ != MotionMode::kIdle || !idle_hold_initialized_) {
            capture_idle_hold_from_current_state(); // 捕获当前状态作为保持点
        }
        active_motion_mode_ = MotionMode::kIdle;    // 设置为空闲
        enter_idle_mode();                          // 进入空闲
        return;
    }

    // 如果当前模式等于请求模式
    if (active_motion_mode_ == requested_motion_mode_) {
        // 如果是从空闲切换到非空闲
        if (active_motion_mode_ == MotionMode::kIdle && requested_motion_mode_ != MotionMode::kIdle) {
            active_motion_mode_ = requested_motion_mode_; // 切换模式
            refresh_plan(now_sec);                        // 刷新规划
        }
        return;
    }

    // 如果可以立即切换或轨迹未运行
    if (can_switch_mode_immediately() && !is_trajectory_running(now_sec)) {
        active_motion_mode_ = requested_motion_mode_; // 切换到请求模式
        refresh_plan(now_sec);                        // 刷新规划
    }
}



/**
 * @brief 进入空闲保持模式
 *
 * 当机器人需要停止运动并保持当前位置时调用此函数。
 * 如果尚未初始化空闲保持点（idle_hold_initialized_ 为 false），
 * 则从当前关节状态捕获一个空闲保持点（capture_idle_hold_from_current_state()），
 * 确保下次空闲时能稳定保持该姿态。
 *
 * 注意：此函数仅在首次进入空闲模式时执行初始化，后续直接复用已捕获的点。
 */
void ArmCtrlNode::enter_idle_mode() {
    if (!idle_hold_initialized_) {
        capture_idle_hold_from_current_state(); // 首次进入空闲模式时，从当前关节状态捕获保持点
    }
}

/**
 * @brief 判断当前是否正在执行轨迹运动
 *
 * 根据当前激活的运动模式（active_motion_mode_）检查对应规划器是否处于“已启动且正在运行”状态。
 *
 * @param now_sec 当前时间（秒），用于规划器内部时间判断
 * @return true  当前模式下轨迹正在执行
 * @return false 当前模式下无轨迹运行（或处于空闲/视觉伺服等非轨迹模式）
 */
bool ArmCtrlNode::is_trajectory_running(double now_sec) const {
    switch (active_motion_mode_) {
    case MotionMode::kJointSpace:     // 关节空间轨迹模式
        return joint_space_move_ && joint_space_move_->started() && joint_space_move_->active(now_sec);
    case MotionMode::kCartesianSpace: // 笛卡尔空间轨迹模式
        return cartesian_space_move_ && cartesian_space_move_->started() && cartesian_space_move_->active(now_sec);
    case MotionMode::kVisualServo:    // 视觉伺服模式（使用单独的执行标志）
        return execute_trajectory_;
    case MotionMode::kIdle:           // 空闲模式（默认不运行轨迹）
    default: return false;
    }
}

/**
 * @brief 判断是否可以立即切换运动模式
 *
 * 仅当当前处于“空闲”或“视觉伺服”模式时允许立即切换（无需等待当前轨迹结束）。
 * 其他模式（如关节/笛卡尔轨迹）必须等待轨迹自然结束才能切换，避免中途打断运动。
 *
 * @return true  可以立即切换
 * @return false 必须等待当前轨迹完成
 */
bool ArmCtrlNode::can_switch_mode_immediately() const {
    return active_motion_mode_ == MotionMode::kIdle || active_motion_mode_ == MotionMode::kVisualServo;
}

/**
 * @brief 设置“执行轨迹”标志位并同步 ROS 参数
 *
 * 用于控制是否真正执行规划好的轨迹（execute_trajectory_）。
 * 若标志位未发生变化则直接返回，避免重复操作。
 * 同时通过 rclcpp::Parameter 更新节点参数服务器，供外部动态监控。
 *
 * @param value 新的执行标志（true=执行，false=停止）
 */
void ArmCtrlNode::set_execute_trajectory_flag(bool value) {
    if (execute_trajectory_ == value) {
        return;                         // 标志位未变化，无需重复设置
    }
    execute_trajectory_         = value;
    updating_execute_parameter_ = true; // 防止参数回调重复触发
    this->set_parameter(rclcpp::Parameter("execute_trajectory", value));
    updating_execute_parameter_ = false;
}

/**
 * @brief 构建预览目标点（用于 RViz 显示）
 *
 * 根据当前请求的运动模式（requested_motion_mode_）计算一个“预览”用的关节空间目标点，
 * 供 RViz 可视化界面显示目标位置（不会实际执行）。
 *
 * @return JointTrajectoryPoint 预览用的关节轨迹点（包含位置、速度、加速度等）
 */
JointTrajectoryPoint ArmCtrlNode::build_preview_target() const {
    switch (requested_motion_mode_) {
    case MotionMode::kJointSpace:                                               // 关节空间模式：直接使用目标关节角度
        return BuildStaticJointTarget(arm_calc_, joint_target_state_.position); // 直接使用关节目标

    case MotionMode::kCartesianSpace: {                                         // 笛卡尔空间模式：需逆运动学（IK）求解
        int result = -1;                                                        // IK 求解结果标志
        // 使用当前关节状态或空闲保持点作为 IK 初始种子（提高求解成功率）
        const JointVector seed = has_joint_state_ ? current_joint_state_.position : idle_hold_point_.position;
        // 调用机械臂正运动学库求解笛卡尔目标对应的关节角度
        const JointVector preview_position = arm_calc_->joint_pos(cartesian_target_, &result, seed);

        if (result < 0) {                                           // IK 求解失败（奇异位形或超出关节限位）
            RCLCPP_WARN(this->get_logger(), "Failed to solve IK for Cartesian preview target, keeping current display");
            RCLCPP_INFO(this->get_logger(), "has_joint_state_ = %s", has_joint_state_ ? "true" : "false");
            return idle_hold_point_;                                // 回退到空闲保持点，避免显示错误
        }
        return BuildStaticJointTarget(arm_calc_, preview_position); // 构建静态关节目标点
    }

    case MotionMode::kVisualServo: {                                // 视觉伺服模式：同样需要 IK 求解视觉目标
        int result                         = -1;
        const JointVector seed             = has_joint_state_ ? current_joint_state_.position : idle_hold_point_.position;
        const JointVector preview_position = arm_calc_->joint_pos(visual_target_, &result, seed);
        if (result < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to solve IK for visual target preview, keeping current display");
            return idle_hold_point_;
        }
        return BuildStaticJointTarget(arm_calc_, preview_position);
    }

    case MotionMode::kIdle:                                         // 空闲模式或其他未知模式
    default: return idle_hold_point_;                               // 默认返回空闲保持点
    }
}

/**
 * @brief 控制循环主函数（定时器回调）
 *
 * 这是机械臂控制节点的核心循环，每隔 control_period_sec_ 执行一次。
 * 负责：
 * 1. 检查关节状态和规划器是否就绪
 * 2. 应用最新的请求运动模式
 * 3. 根据当前激活模式采样目标关节点
 * 4. 判断轨迹是否结束并切换到空闲模式
 * 5. 发布实际控制目标 + RViz 预览/可视化信息
 */
void ArmCtrlNode::publish_control_loop() {
    // 检查关节状态和规划器是否准备好
    if (!has_joint_state_ || !planners_ready_) {
        return;                                                // 未收到关节反馈或规划器未初始化，直接跳过本次循环
    }

    const double now_sec = this->get_clock()->now().seconds(); // 获取当前 ROS 时间（秒）

    if (last_ee_log_time_sec_ < 0.0 || (now_sec - last_ee_log_time_sec_) >= 0.25) {
        const CartesianPose ee_pose  = arm_calc_->end_pose(current_joint_state_.position);
        const Eigen::Vector3d ee_rpy = ee_pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        RCLCPP_INFO(
            get_logger(), "EE pose pos=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)", ee_pose.position.x(), ee_pose.position.y(),
            ee_pose.position.z(), ee_rpy.x(), ee_rpy.y(), ee_rpy.z());
        last_ee_log_time_sec_ = now_sec;
    }

    apply_requested_mode(now_sec);
    JointTrajectoryPoint target_point;                         // 本次控制循环的目标关节点
    // 根据当前激活的运动模式采样目标点
    switch (active_motion_mode_) {
    case MotionMode::kIdle: // 空闲模式：保持当前位置
        target_point = idle_hold_point_;
        // 重新计算关节力矩（逆动力学），保证力控或仿真时姿态稳定
        // target_point.torque = arm_calc_->joint_torque_inverse_dynamics(
        //     target_point.position, JointVector::Zero(), JointVector::Zero());
        // RCLCPP_INFO(get_logger(),"target_point=(%lf,%lf,%lf)",target_point.position[0],target_point.position[1],target_point.position[2]);
        break;

    case MotionMode::kJointSpace:                                                  // 关节空间轨迹模式
        target_point = joint_space_move_->sample(now_sec);                         // 从关节规划器采样当前时刻的目标
        if (!joint_space_move_->active(now_sec) && joint_space_move_->started()) { // 轨迹已自然结束
            set_idle_hold_point(target_point);                                     // 更新空闲保持点为本次轨迹终点
            set_execute_trajectory_flag(false);                                    // 停止执行标志
            active_motion_mode_    = MotionMode::kIdle;                            // 切换到空闲模式
            requested_motion_mode_ = MotionMode::kIdle;
            enter_idle_mode();                                                     // 初始化空闲保持
            target_point = idle_hold_point_;                                       // 本次循环使用空闲点
        }
        break;

    case MotionMode::kCartesianSpace:                                              // 笛卡尔空间轨迹模式（逻辑同关节空间）
        target_point = cartesian_space_move_->sample(now_sec);
        if (!cartesian_space_move_->active(now_sec) && cartesian_space_move_->started()) {
            set_idle_hold_point(target_point);
            set_execute_trajectory_flag(false);
            active_motion_mode_    = MotionMode::kIdle;
            requested_motion_mode_ = MotionMode::kIdle;
            enter_idle_mode();
            target_point = idle_hold_point_;
        }
        break;

    case MotionMode::kVisualServo:                                                 // 视觉伺服模式（实时闭环）
        visual_servo_move_->set_current_joint_state(current_joint_state_);         // 更新当前关节反馈
        target_point = visual_servo_move_->sample(now_sec);                        // 从视觉伺服规划器采样
        //     RCLCPP_INFO(get_logger(), "进入视觉伺服joint=%lf %lf %lf %lf", target_point.position[0],
        // target_point.position[1],target_point.position[2],target_point.position[3]);
        break;
    }


    publish_joint_target(target_point);                    // 先发布给下位机/驱动器
    current_joint_state_ = from_arm_message(target_point); // 直接赋值（完美跟踪）

    // RViz 可视化部分（保持不变）
    JointTrajectoryPoint rviz_point = execute_trajectory_ ? target_point : build_preview_target();
    const rclcpp::Time stamp        = this->get_clock()->now();
    rviz_joint_pub_->publish(to_joint_state_msg(rviz_point, stamp));
    publish_visualization(rviz_point);




    // // 选择 RViz 显示点：
    // //   - 如果正在执行轨迹（execute_trajectory_ == true），显示真实目标点
    // //   - 否则显示预览目标点（build_preview_target）
    // JointTrajectoryPoint rviz_point = execute_trajectory_ ? target_point : build_preview_target();

    // publish_joint_target(target_point);                    // 发布给下位机/驱动器的真实控制指令
    // const rclcpp::Time stamp = this->get_clock()->now();  // 当前时间戳
    // rviz_joint_pub_->publish(to_joint_state_msg(rviz_point, stamp));  // 发布给 RViz 的关节状态
    // publish_visualization(rviz_point);                     // 发布当前末端与目标末端可视化 Marker
}

/**
 * @brief 发布真实的关节控制目标（给驱动器）
 *
 * 将目标关节点转换为 Arm 消息并发布，同时打印关节2的信息（调试用）。
 *
 * @param point 要发布的关节轨迹点
 */
void ArmCtrlNode::publish_joint_target(const JointTrajectoryPoint& point) {
    joint_target_pub_->publish(to_arm_message(point));
    // RCLCPP_INFO(get_logger(), "发布给与下位机通层的数据发布成功  joint=%lf %lf %lf %lf",
    // point.position[0],point.position[1],point.position[2],point.position[3]);
}

/**
 * @brief 发布可视化 Marker（RViz 调试用）
 *
 * 在 base_link 坐标系下绘制：
 * - 当前末端位置（绿色球）
 * - 目标末端位置（橙色球）
 * - 两者之间的连线（蓝色线段）
 *
 * @param target_point 当前周期的目标关节点（用于计算目标末端位姿）
 */
void ArmCtrlNode::publish_visualization(const JointTrajectoryPoint& target_point) {
    if (!arm_calc_) {
        return; // 运动学计算器未初始化，无法发布可视化
    }

    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = this->now();

    // 当前末端位置 Marker（绿色）
    visualization_msgs::msg::Marker current_marker;
    current_marker.header.frame_id    = base_link_;
    current_marker.header.stamp       = stamp;
    current_marker.ns                 = "arm_ctrl";
    current_marker.id                 = 0;
    current_marker.type               = visualization_msgs::msg::Marker::SPHERE;
    current_marker.action             = visualization_msgs::msg::Marker::ADD;
    current_marker.scale.x            = 0.04;
    current_marker.scale.y            = 0.04;
    current_marker.scale.z            = 0.04;
    current_marker.color.r            = 0.1F;
    current_marker.color.g            = 0.8F;
    current_marker.color.b            = 0.2F;
    current_marker.color.a            = 1.0F;
    current_marker.pose.position      = ToPoint(arm_calc_->end_pose(current_joint_state_.position).position);
    current_marker.pose.orientation.w = 1.0;

    // 目标末端位置 Marker（橙色）
    visualization_msgs::msg::Marker target_marker = current_marker;
    target_marker.id                              = 1;
    target_marker.color.r                         = 0.95F;
    target_marker.color.g                         = 0.25F;
    target_marker.color.b                         = 0.15F;
    target_marker.pose.position                   = ToPoint(arm_calc_->end_pose(target_point.position).position);
    target_marker.pose.orientation                = ToMsgQuaternion(arm_calc_->end_pose(target_point.position).orientation);

    // 连接线 Marker（蓝色）
    visualization_msgs::msg::Marker line_marker = current_marker;
    line_marker.id                              = 2;
    line_marker.type                            = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.scale.x                         = 0.01;
    line_marker.color.r                         = 0.1F;
    line_marker.color.g                         = 0.4F;
    line_marker.color.b                         = 0.95F;
    line_marker.points.push_back(current_marker.pose.position);
    line_marker.points.push_back(target_marker.pose.position);

    markers.markers.push_back(current_marker);
    markers.markers.push_back(target_marker);
    markers.markers.push_back(line_marker);

    marker_pub_->publish(markers); // 发布完整的 MarkerArray
}

/**
 * @brief 关节状态消息回调
 *
 * 当收到来自关节驱动器的实时状态（robot_interfaces::msg::Arm）时调用。
 * 更新内部 current_joint_state_，并在首次收到时捕获空闲保持点。
 *
 * @param msg 接收到的关节状态消息（包含位置、速度、力矩等）
 */
// void ArmCtrlNode::on_joint_state(const JointTrajectoryPoint& point) {
//     const bool was_initialized = has_joint_state_;        // 记录之前是否已初始化
//     current_joint_state_       = from_arm_message(point); // 转换消息为内部 JointTrajectoryPoint
//     has_joint_state_           = true;

//     if (!was_initialized) {                               // 首次收到关节状态
//         capture_idle_hold_from_current_state();           // 捕获初始空闲保持点
//     }

//     if (!planners_ready_) {                               // 规划器尚未准备好
//         refresh_plan(this->get_clock()->now().seconds()); // 立即刷新规划
//     }
// }

/**
 * @brief 视觉目标（PoseStamped）回调
 *
 * 接收来自视觉系统的目标位姿，更新 visual_target_ 和 cartesian_target_。
 * 同时通知视觉伺服规划器，并根据当前模式决定是否重新规划。
 *
 * @param msg 带时间戳的视觉目标位姿（位置 + 四元数）
 */
void ArmCtrlNode::on_visual_target(const geometry_msgs::msg::PoseStamped& msg) {
    const CartesianPose target_pose{
        Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
        NormalizeQuaternion(
            Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z))};

    visual_target_    = target_pose;
    cartesian_target_ = target_pose;

    if (visual_servo_move_) {
        visual_servo_move_->set_target_pose(visual_target_); // 更新视觉伺服控制器目标
    }

    // 如果当前请求的是笛卡尔模式且正在执行，则立即重新规划
    if (requested_motion_mode_ == MotionMode::kCartesianSpace && has_joint_state_) {
        planners_ready_ = false;
        if (active_motion_mode_ == MotionMode::kCartesianSpace && execute_trajectory_) {
            refresh_plan(this->get_clock()->now().seconds());
        }
    }

    // 视觉伺服模式下直接标记规划器就绪
    if (requested_motion_mode_ == MotionMode::kVisualServo && has_joint_state_) {
        planners_ready_ = true;
    }
}

/**
 * @brief 关节空间目标（Float64MultiArray）回调
 *
 * 接收 6 个关节角度目标，更新 joint_target_state_。
 * 若当前处于空闲且请求切换到关节空间模式，则立即激活规划。
 *
 * @param msg 包含至少 6 个 double 的数组（关节1~6 目标角度）
 */
void ArmCtrlNode::on_joint_space_target(const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() < kJointDoF) {
        RCLCPP_WARN(this->get_logger(), "joint_space_target requires at least 6 elements");
        return;
    }

    for (std::size_t i = 0; i < kJointDoF; ++i) {
        joint_target_state_.position[static_cast<int>(i)] = msg.data[i];
    }

    // 满足条件时立即切换到关节空间模式并刷新规划
    if (has_joint_state_ && active_motion_mode_ == MotionMode::kIdle && requested_motion_mode_ == MotionMode::kJointSpace
        && execute_trajectory_) {
        active_motion_mode_ = MotionMode::kJointSpace;
        refresh_plan(this->get_clock()->now().seconds());
    }
}

/**
 * @brief ROS2 参数动态修改回调
 *
 * 处理所有可动态修改的参数（运动模式、轨迹时长、控制周期、execute_trajectory 等）。
 * 更新对应成员变量，并在必要时重新创建控制定时器、刷新规划器。
 *
 * @param params 待修改的参数列表
 * @return SetParametersResult 成功/失败结果（含失败原因）
 */
rcl_interfaces::msg::SetParametersResult ArmCtrlNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "motion_mode") {
            requested_motion_mode_ = parse_motion_mode(param.as_int());
        } else if (param.get_name() == "trajectory_duration") {
            trajectory_duration_sec_ = std::max(param.as_double(), 0.1);
        } else if (param.get_name() == "control_period") {
            control_period_sec_ = std::max(param.as_double(), 0.005);
        } else if (param.get_name() == "execute_trajectory") {
            if (!updating_execute_parameter_) {
                execute_trajectory_ = param.as_bool();
            }
            if (!execute_trajectory_) {
                // === 关键修复：视觉伺服退出时，立即用当前真实关节位置更新idle_hold_point_ ===
                if (active_motion_mode_ == MotionMode::kVisualServo && has_joint_state_) {
                    capture_idle_hold_from_current_state();
                    RCLCPP_INFO(get_logger(), "视觉伺服结束，已更新idle_hold_point_，防止位置跳变");
                }
                requested_motion_mode_ = MotionMode::kIdle;
                active_motion_mode_    = MotionMode::kIdle;
                enter_idle_mode();
            }
        } else if (param.get_name() == "visual_servo_kp") {
            visual_servo_kp_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                visual_servo_move_->set_kp(visual_servo_kp_);
            }
        } else if (param.get_name() == "visual_servo_max_linear_acceleration") {
            visual_servo_max_linear_acceleration_ = std::max(param.as_double(), 0.0);
            if (visual_servo_move_) {
                visual_servo_move_->set_max_linear_acceleration(visual_servo_max_linear_acceleration_);
            }
        } else if (param.get_name() == "joint_target") {
            const auto values = param.as_double_array();
            if (values.size() != kJointDoF) {
                result.successful = false;
                result.reason     = "joint_target must contain 4 values";
                return result;
            }
            for (std::size_t i = 0; i < kJointDoF; ++i) {
                joint_target_state_.position[static_cast<int>(i)] = values[i];
            }
        } else if (param.get_name() == "cartesian_target_position") {
            const auto values = param.as_double_array();
            if (values.size() != 3) {
                result.successful = false;
                result.reason     = "cartesian_target_position must contain 3 values";
                return result;
            }
            cartesian_target_.position = Eigen::Vector3d(values[0], values[1], values[2]);
        } else if (param.get_name() == "cartesian_target_quaternion") {
            const auto values = param.as_double_array();
            if (values.size() != 4) {
                result.successful = false;
                result.reason     = "cartesian_target_quaternion must contain 4 values";
                return result;
            }
            cartesian_target_.orientation = NormalizeQuaternion(Eigen::Quaterniond(values[0], values[1], values[2], values[3]));
        }
    }

    // 控制周期参数变化时，重新创建 WallTimer
    if (control_timer_) {
        control_timer_->cancel();
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_sec_), std::bind(&ArmCtrlNode::publish_control_loop, this));
    }

    planners_ready_ = false; // 参数变化后需要重新规划

    if (has_joint_state_) {
        apply_requested_mode(this->get_clock()->now().seconds());
        if (active_motion_mode_ != MotionMode::kIdle) {
            refresh_plan(this->get_clock()->now().seconds());
        } else {
            planners_ready_ = true;
        }
    }

    return result;
}

// 解析运动模式：将整数参数转换为运动模式枚举
// 参数：mode_value - 模式值
// 返回：MotionMode枚举
ArmCtrlNode::MotionMode ArmCtrlNode::parse_motion_mode(int mode_value) {
    if (mode_value == 0) {                                                                       // 空闲模式
        return MotionMode::kIdle;
    }
    if (mode_value == 1) {                                                                       // 关节空间轨迹规划模式
        return MotionMode::kJointSpace;
    }
    if (mode_value == 2) {                                                                       // 笛卡尔空间轨迹规划模式
        return MotionMode::kCartesianSpace;
    }
    if (mode_value == 3) {                                                                       // 视觉伺服控制模式
        return MotionMode::kVisualServo;
    }
    throw std::invalid_argument("unsupported motion_mode value: " + std::to_string(mode_value)); // 无效值抛异常
}

// 从Arm消息转换为JointState：解析ROS消息为内部关节状态
// 参数：msg - Arm消息
// 返回：JointState结构体
// JointState ArmCtrlNode::from_arm_message(const robot_interfaces::msg::Arm& msg) {
//     JointState state;
//     for (std::size_t i = 0; i < kJointDoF; ++i) {  // 遍历6个关节
//         state.position[static_cast<int>(i)] = msg.motor[i].rad;  // 位置（弧度）
//         //state.velocity[static_cast<int>(i)] = msg.motor[i].omega;  // 速度（弧度/秒）
//         //state.torque[static_cast<int>(i)] = msg.motor[i].torque;  // 扭矩
//     }
//     return state;
// }

JointState ArmCtrlNode::from_arm_message(const JointTrajectoryPoint& point) {
    JointState state;
    for (std::size_t i = 0; i < kJointDoF; ++i) {                                                      // 遍历6个关节
        state.position[static_cast<int>(i)] = static_cast<float>(point.position[static_cast<int>(i)]); // 位置（弧度）
        // state.velocity[static_cast<int>(i)] = msg.motor[i].omega;  // 速度（弧度/秒）
        // state.torque[static_cast<int>(i)] = msg.motor[i].torque;  // 扭矩
    }
    return state;
}

// 转换为Arm消息：将JointTrajectoryPoint转换为ROS Arm消息
// 参数：point - 轨迹点
// 返回：robot_interfaces::msg::Arm消息
robot_interfaces::msg::Arm ArmCtrlNode::to_arm_message(const JointTrajectoryPoint& point) {
    robot_interfaces::msg::Arm msg;
    for (std::size_t i = 0; i < kJointDoF; ++i) {                                   // 遍历6个关节
        msg.motor[i].rad = static_cast<float>(point.position[static_cast<int>(i)]); // 位置
        // msg.motor[i].omega = static_cast<float>(point.velocity[static_cast<int>(i)]);  // 速度
        // msg.motor[i].torque = static_cast<float>(point.torque[static_cast<int>(i)]);  // 扭矩
    }
    return msg;
}

// 转换为JointState消息：将轨迹点转换为ROS JointState消息，用于RViz
// 参数：point - 轨迹点，stamp - 时间戳
// 返回：sensor_msgs::msg::JointState消息
sensor_msgs::msg::JointState ArmCtrlNode::to_joint_state_msg(const JointTrajectoryPoint& point, const rclcpp::Time& stamp) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;                                    // 设置时间戳
    msg.name         = {"joint1", "joint2", "joint3", "joint4"}; // 关节名称
    msg.position.resize(kJointDoF);                              // 位置数组
    // msg.velocity.resize(kJointDoF);  // 速度数组
    // msg.effort.resize(kJointDoF);  // 力矩数组
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        msg.position[i] = point.position[static_cast<int>(i)]; // 设置位置
        // msg.velocity[i] = point.velocity[static_cast<int>(i)];  // 设置速度
        // msg.effort[i] = point.torque[static_cast<int>(i)];  // 设置力矩
    }
    return msg;
}

// 获取双精度数组参数：安全获取参数中的数组值
// 参数：node - 节点引用，name - 参数名，expected_size - 期望大小
// 返回：std::vector<double>数组
std::vector<double> ArmCtrlNode::get_double_array_param(const rclcpp::Node& node, const std::string& name, std::size_t expected_size) {
    const auto values = node.get_parameter(name).as_double_array(); // 获取参数
    if (values.size() != expected_size) {                           // 检查大小
        throw std::runtime_error("parameter " + name + " expected size " + std::to_string(expected_size));
    }
    return values;                                                  // 返回数组
}

} // namespace arm_calc
