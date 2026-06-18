# 项目规则

## 代码探索限制

- **缺少相关文件时，先向用户索要，不要直接调用 Explore agent**
- 能直接 Read 已知路径的文件时，不启动任何 Agent
- 对于"某文件/节点做了什么"的问题，只读相关的 3-5 个文件
- 只有用户明确要求"搜索整个项目"或问题确实需要广泛探索时，才使用 Explore

## 项目结构

- ROS2 项目，使用 colcon 构建（`colcon build --symlink-install`）
- 包在 `src/` 目录下
- 启动文件在 `src/launch_pack/launch/`
- 构建输出：`build/`，安装输出：`install/`

## 包列表

| 包 | 职责 |
|---|---|
| `arm` | 机器人模型（URDF、STL 网格、MuJoCo XML） |
| `arm_calc` | KDL 运动学求解（FK/IK），4 种运动模式 |
| `arm_task` | 高层任务调度（状态机），action server |
| `robot_driver` | USB CDC 硬件接口（STM32） |
| `vision` | RealSense 视觉检测，发布目标 TF |
| `mujoco_ros2_control` | MuJoCo 仿真桥接 |
| `dog_controller` | 关节 PD 控制器（ros2_control 插件） |
| `robot_interfaces` | 自定义 msg/srv/action 定义 |
| `launch_pack` | 启动文件集合 |

## 关键节点与数据流

```
vision_node ──TF──→ arm_task ──参数──→ arm_calc ──myjoints_target──→ dog_controller/robot_driver
                         ↑                                              │
                         └──────────── myjoints_state ←─────────────────┘
```

- `arm_task`：订阅 TF（target_object），发布 `visual_target_pose`、`joint_space_target`
- `arm_calc`：订阅 `myjoints_state`、`visual_target_pose`、`joint_space_target`；发布 `myjoints_target`、`joint_states`
- `robot_driver`：USB CDC 通信，发布 `myjoints_state`
- `vision`：RealSense 检测红色杆，发布 `base_link→target_object`、`camera_link→target_camera`

## TF 坐标系

- 机器人链：`world` → `base_link` → `Link1` → `Link2` → `Link3` → `Link4` → `Link5` → `Link6` → `Link7`
- 相机：`Link4` → `camera_link`（静态，手眼标定）
- 目标：`camera_link` → `target_camera`（动态），`base_link` → `target_object`（动态）

## 关键配置文件

- `src/arm/model/robotic_arm.urdf` — 机器人模型
- `src/arm_task/config/arm_position.yaml` — 命名关节位置
- `src/launch_pack/config/ros2_controller.yaml` — 控制器参数
- `src/launch_pack/rviz/display_config.rviz` — RViz 显示配置

## 任务系统

- 默认任务：`idel`（空闲，等待指令）
- `task_id=1` → `move_kfs`（关节/笛卡尔运动）
- `task_id=2` → `catch_kfs`（视觉伺服抓取）
- `task_id=3` → `place_kfs`（放置到货架）
- `grasp_it` 参数（来自物理按钮）可直接触发 `catch_kfs`

## 回复风格

- 用户问具体问题，直接读文件、直接回答，不要查 git log 做铺垫
- 不要加"好的，收到"、"请问你需要我做什么"之类的套话
- 不要反问用户"你希望我怎么做"——能自己判断就直接做

## 代码规范

- C++20
- 代码风格：LLVM 基础，140 列宽，4 空格缩进（见 `.clang-format`）
