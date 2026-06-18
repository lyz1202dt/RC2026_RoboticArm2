# 河北科技大学2026 RC Robocon 六轴机械臂+KDL解算+Mujoco仿真方案

# 手动输入关节角度，控制机械臂运动。
1. . install/setup.bash && ros2 run arm_calc arm_calc_test
2. 在rqt中 Plugins -> Configuration -> ... -> arm_test_node 设置关节角度，点击 publish_joint_target .
3. arm_calc_node 里面调整 motion_mode , ececute_trajectory 设为真

或者更简单的方法：
启动 arm_task_move_test.launch.py:1。在工作区已经编译过的前提下，先 source 你的 ROS 2 环境和 install 目录，然后执行 ros2 launch launch_pack arm_task_move_test.launch.py。这个 launch 会先拉起仿真，再延时启动 move_kfs_test.cpp:1 里的测试节点。




检测到目标 (camera_frame): (0.194, 0.041, 0.447) 米, 距离: 0.447 米
目标(base_link): (0.768, -0.097, 1.142) 米
最右侧杆坐标: (0.194, 0.041, 0.447) 米


检测到目标 (camera_frame): (-0.001, 0.048, 0.435) 米, 距离: 0.435 米
目标(base_link): (0.712, -0.104, 0.955) 米
最右侧杆坐标: (-0.001, 0.048, 0.435) 米

检测到目标 (camera_frame): (-0.195, 0.048, 0.421) 米, 距离: 0.421 米
目标(base_link): (0.654, -0.104, 0.770) 米
最右侧杆坐标: (-0.195, 0.048, 0.421) 米



已确认架构原则：

1. Action = 主业务接口
   - controller 只能通过 Action 请求任务
   - 典型任务：
     - MoveToPose
     - PlanTrajectory
     - PickPlace
   - 支持反馈、取消、抢占

2. Service = 计算/查询辅助接口
   - 无状态
   - 快速完成
   - 不负责长时间任务
   - 典型接口：
     - ComputeIK
     - ComputeFK
     - ValidateTrajectory
   - 禁止 controller 依赖多个 Service 自行拼装运动流程

3. Topic = 状态与数据流
   - 当前关节状态
   - 末端位姿
   - 规划轨迹
   - 调试信息
   - 传感器数据
   - 不承担任务控制职责

4. arm_calc 对外只暴露任务语义
   - 外部不知道内部使用什么 IK 或规划器
   - controller 不接触 KDL/TRAC-IK/MoveIt 等实现细节

5. IK/FK/轨迹生成属于内部实现
   - 可自由替换：
       KDL → TRAC-IK
       Quintic → TOTG/Ruckig
       自研规划器 → MoveIt
   - 要求：
       Action API 保持不变

6. 约束分层
   - IK层：
       关节位置限位(position limits)
   - Trajectory层：
       速度限位(velocity limits)
       加速度限位(acceleration limits)
       jerk限位(未来)
   - Controller层：
       仅跟踪执行，不修正轨迹
