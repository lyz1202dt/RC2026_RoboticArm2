# arm_task / arm_calc 重构接口说明

## arm_task 核心职责

`arm_task` 是机械臂任务编排节点，节点名为 `arm_task`。它通过参数触发任务线程，组合视觉目标、放置目标、预设关节位、气泵开关，并向 `arm_calc` 发布统一的 `ArmCmd` 运动命令。

任务线程不轮询参数。`arm_task` 参数变化后，参数回调会唤醒正在等待的任务线程，回调本身只记录任务请求并立即返回，避免阻塞 ROS executor。

视觉输入不再广播 `target_object` TF。`arm_task` 直接查询 `camera_left_link` 或 `camera_right_link` 到 `base_link` 的变换，将 `pnp_move` 中的目标坐标转换到 `base_link` 后缓存。抓取目标的 `z` 固定保留为 `-0.256`。

## arm_task 开放接口

### 参数

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `arm_task` | `int` | 任务触发入口。任务完成后自动恢复为 `0` |
| `preset_position_id` | `int` | `arm_task=10` 时使用的预设关节位置 ID |

### arm_task 任务模式

| `arm_task` | 行为 |
| --- | --- |
| `0` | 待机 |
| `1` | 左臂抓取 |
| `2` | 右臂抓取 |
| `3` | 左臂放置 |
| `4` | 右臂放置 |
| `5` | 固定释放流程：到固定放置位、关闭气泵、回零 |
| `10` | 移动到 `preset_position_id` 指定的 YAML 预设关节位 |

### 订阅话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `pnp_move` | `robot_interfaces/msg/Vis` | 视觉目标，字段为 `x/y/z`。根据当前任务模式选择左/右相机坐标系 |
| `place_target_pose` | `geometry_msgs/msg/PoseStamped` | 放置目标位姿 |

### 发布话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `arm_cmd` | `robot_interfaces/msg/ArmCmd` | 发给 `arm_calc` 的统一运动命令 |

### 依赖的外部接口

| 接口 | 说明 |
| --- | --- |
| TF: `base_link <- camera_left_link` | 左臂视觉目标坐标转换 |
| TF: `base_link <- camera_right_link` | 右臂视觉目标坐标转换 |
| `/arm_node` 参数 `enable_air_pump` | 气泵开关。仿真中该服务可以不存在，节点只会告警 |

### 配置文件

配置文件为 `arm_task/config/arm_position.yaml`。

| 字段 | 说明 |
| --- | --- |
| `ready_position` | 抓取/放置前的准备关节位，4 个值 |
| `home_position` | 回零关节位，4 个值 |
| `place_position` | 固定释放任务使用的关节位，4 个值 |
| `arm_positions[].index` | 预设关节位 ID |
| `arm_positions[].joints` | 预设关节位，4 个值 |

## arm_calc 开放接口

### 节点

| 节点 | 可执行文件 | 说明 |
| --- | --- | --- |
| `arm_calc_node` | `arm_calc` | 机械臂轨迹控制节点 |
| `arm_test_node` | `arm_test_node` | 测试节点，可通过参数发布 `ArmCmd` |

### arm_calc 订阅话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `arm_cmd` | `robot_interfaces/msg/ArmCmd` | 运动命令 |

`ArmCmd` 字段约定：

| 字段 | 说明 |
| --- | --- |
| `arm_id` | `0` 左臂，`1` 右臂 |
| `mode` | `0` 停止，`1` 关节空间，`2` 笛卡尔空间 |
| `position.data` | 关节空间为 4 个关节目标；笛卡尔空间为 `[x, y, z, pitch]` |
| `duration` | 轨迹时长，单位秒 |

### arm_calc 发布话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | 发给仿真控制器、RViz 或 `robot_driver` 的关节目标 |

### arm_calc 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `arm_cmd_topic` | `arm_cmd` | `ArmCmd` 输入话题 |
| `joint_state_topic` | `joint_states` | 关节输出话题 |
| `control_period` | `0.02` | 控制周期 |
| `base_link` | `base_link` | KDL 基坐标 |
| `left_tip_link` | `left4` | 左臂末端 link |
| `right_tip_link` | `right4` | 右臂末端 link |
| `initial_joint_positions` | 7 个 `0.0` | 初始关节位置 |

## launch_pack 启动方式

| 启动方式 | 命令 |
| --- | --- |
| 1. 单独启动 MuJoCo 仿真和控制器 | `ros2 launch launch_pack arm_mujoco_sim.launch.py` |
| 2. 启动 1，并启动 `arm_calc` 和 `arm_test_node` | `ros2 launch launch_pack arm_mujoco_test.launch.py` |
| 3. 启动 1，并启动 `arm_calc` 和 `arm_task` | `ros2 launch launch_pack arm_task_sim.launch.py` |
| 4. 启动 `robot_driver`，并启动 `arm_calc` 和 `arm_test_node` | `ros2 launch launch_pack arm_real_test.launch.py` |
| 5. 启动 `robot_driver`，并启动 `arm_calc` 和 `arm_task` | `ros2 launch launch_pack arm_real.launch.py` |

仿真入口支持：

```bash
ros2 launch launch_pack arm_task_sim.launch.py show_gui:=false show_rviz:=false
```

真实硬件入口支持：

```bash
ros2 launch launch_pack arm_real.launch.py show_rviz:=false
```

## 单元测试方法

### 构建和基础测试

```bash
colcon build --packages-select robot_interfaces arm_calc arm_task launch_pack
source install/setup.bash
colcon test --packages-select arm_calc arm_task --event-handlers console_direct+
colcon test-result --verbose
```

### arm_calc 接口测试

启动仿真测试组合：

```bash
ros2 launch launch_pack arm_mujoco_test.launch.py show_gui:=false show_rviz:=false
```

发布关节空间命令：

```bash
ros2 param set /arm_test_node arm_id 0
ros2 param set /arm_test_node joint1 0.0
ros2 param set /arm_test_node joint2 2.4
ros2 param set /arm_test_node joint3 1.3
ros2 param set /arm_test_node joint4 1.0
ros2 param set /arm_test_node publish_joint_target true
```

发布笛卡尔空间命令：

```bash
ros2 param set /arm_test_node arm_id 0
ros2 param set /arm_test_node pose_x 0.35
ros2 param set /arm_test_node pose_y 0.05
ros2 param set /arm_test_node pose_z -0.256
ros2 param set /arm_test_node pose_pitch -1.5708
ros2 param set /arm_test_node publish_pose_target true
```

观察输出：

```bash
ros2 topic echo /joint_states
```

### arm_task 任务接口测试

启动任务仿真组合：

```bash
ros2 launch launch_pack arm_task_sim.launch.py show_gui:=false show_rviz:=false
```

测试预设关节位任务：

```bash
ros2 param set /arm_task preset_position_id 0
ros2 param set /arm_task arm_task 10
ros2 topic echo /arm_cmd
```

测试视觉抓取任务：

```bash
ros2 topic pub --once /pnp_move robot_interfaces/msg/Vis "{x: 0.20, y: 0.05, z: 0.10}"
ros2 param set /arm_task arm_task 1
ros2 topic echo /arm_cmd
```

测试放置任务：

```bash
ros2 topic pub --once /place_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.05, z: -0.256}, orientation: {w: 1.0}}}"
ros2 param set /arm_task arm_task 3
ros2 topic echo /arm_cmd
```
