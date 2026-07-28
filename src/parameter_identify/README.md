# parameter_identify

`parameter_identify` 用于 RC2026 六轴机械臂的离线动力学参数辨识。这个包主要提供两个功能：

1. 生成用于参数辨识的期望关节位置轨迹 CSV。
2. 读取 `parameter_measure` 录制的数据 CSV，辨识惯性参数，并导出校准后的 URDF。

## 依赖

FIGAROH 已作为第三方库放在仓库中：

```bash
third_party/figaroh-plus
```

程序只加载当前流程需要的 FIGAROH 源文件，因此不需要安装 FIGAROH 的可视化和完整轨迹优化依赖。但下面这些 Python 依赖必须可用：

```text
pinocchio, numpy, scipy, pyyaml, picos, cvxopt
```

其中 `pinocchio` 是必需的。轨迹生成阶段会用 Pinocchio 做正运动学检查，确保配置中指定的连杆高度满足 `z > ground_clearance`；如果当前环境不能 `import pinocchio`，轨迹生成会直接报错。

## 配置文件

默认配置文件是：

```bash
src/parameter_identify/config/identify.yaml
```

如果使用已经 `source install/setup.bash` 的 ROS 工作空间，两个命令默认会从包的 share 目录读取安装后的配置。修改源码中的配置后，建议重新构建：

```bash
colcon build --packages-select parameter_identify --symlink-install
source install/setup.bash
```

也可以在命令行中显式指定配置文件：

```bash
--config /space2/Project/RC2026_RoboticArm2/src/parameter_identify/config/identify.yaml
```

常用配置项：

```yaml
model:
  active_joints: [joint1, joint2, joint3, joint4, joint5, joint6]

data:
  sample_time: 0.02
  acceleration_source: computed
  acceleration_smoothing_window: 11
  acceleration_smoothing_polyorder: 3

trajectory_generation:
  duration: 10.0
  sample_time: 0.02
  harmonics: 5
  attempts: 250
  limit_margin_ratio: 0.12
  velocity_limit_ratio: 0.35
  acceleration_limit: 8.0
  ground_clearance: 0.02
  ground_check_links: [link2, link3, link4, link5, link6]

reconstruction:
  enabled: true
  method: sdp
  prior_source: urdf
  physical_projection: false
  strict_base_constraints: false
  base_residual_weight: 1.0e6
  require_physical: true
  cad_constraints:
    source: urdf
    mass_scale_min: 0.8
    mass_scale_max: 1.2
    com_margin_abs: 0.002
    com_margin_rel: 0.1
```

## 生成激励轨迹

生成轨迹的入口是 `generate_trajectory`。它会根据 URDF 中的关节限位随机采样多谐波轨迹，计算辨识回归矩阵的条件数，并在多次尝试中选择条件数较好的轨迹。输出 CSV 只包含期望位置：

```text
time,pos_0,pos_1,pos_2,pos_3,pos_4,pos_5
```

推荐命令：

```bash
source install/setup.bash

ros2 run parameter_identify generate_trajectory \
  --urdf src/arm/model/robotic_arm.urdf \
  --output /tmp/optimized_expected_trajectory.csv \
  --config src/parameter_identify/config/identify.yaml
```

如果没有安装 ROS 包，也可以从源码目录直接运行：

```bash
cd /space2/Project/RC2026_RoboticArm2/src/parameter_identify

python3 -m parameter_identify.generate_trajectory \
  --urdf /space2/Project/RC2026_RoboticArm2/src/arm/model/robotic_arm.urdf \
  --output /tmp/optimized_expected_trajectory.csv \
  --config /space2/Project/RC2026_RoboticArm2/src/parameter_identify/config/identify.yaml
```

命令输出中会打印：

```text
trajectory CSV: /tmp/optimized_expected_trajectory.csv
samples: ...
duration: ... s
condition number: ...
rejected by ground clearance: ...
```

其中 `condition number` 越小，一般说明这条轨迹对参数辨识越友好；`rejected by ground clearance` 表示有多少候选轨迹因为正运动学检查中连杆高度低于 `ground_clearance` 被丢弃。

生成的 `/tmp/optimized_expected_trajectory.csv` 可以交给 `parameter_measure` 使用。`parameter_measure` 会先通过五次多项式插值移动到 CSV 的起点，然后按 CSV 中的时间和期望位置发送轨迹，同时录制实际关节位置、速度和力矩。

## 对录制数据进行参数辨识

`identify_arm` 读取 `parameter_measure` 录制的数据 CSV。当前推荐的录制 CSV 格式是：

```text
time,pos_0..pos_5,vel_0..vel_5,torque_0..torque_5
```

加速度默认不从录制文件中读取，而是由 `vel_*` 离线平滑微分得到。旧格式中如果包含 `acc_0..acc_5` 也可以读取，但只有在确认录制加速度干净可信时，才建议在配置中设置：

```yaml
data:
  acceleration_source: recorded
```

推荐命令：

```bash
source install/setup.bash

ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf src/arm/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --report /tmp/robotic_arm_identified_report.yaml \
  --config src/parameter_identify/config/identify.yaml
```

如果没有安装 ROS 包，也可以从源码目录直接运行：

```bash
cd /space2/Project/RC2026_RoboticArm2/src/parameter_identify

python3 -m parameter_identify.identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf /space2/Project/RC2026_RoboticArm2/src/arm/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --report /tmp/robotic_arm_identified_report.yaml \
  --config /space2/Project/RC2026_RoboticArm2/src/parameter_identify/config/identify.yaml
```

参数含义：

`--csv`：`parameter_measure` 录制得到的 CSV 数据。

`--urdf`：辨识前的原始 URDF，作为动力学模型和参数先验。

`--output`：写出的校准后 URDF。

`--report`：写出的 YAML 辨识报告；如果不指定，会默认写到输出 URDF 同目录下。

`--config`：辨识配置文件；不指定时使用包内安装后的默认配置。

命令成功后会打印类似下面的信息：

```text
identified URDF: /tmp/robotic_arm_identified.urdf
report: /tmp/robotic_arm_identified_report.yaml
updated links: link1, link2, link3, link4, link5, link6
base parameter count: 36
rmse: ...
correlation: ...
```

报告中重点看这些字段：

`rmse`：力矩拟合误差，越小越好。

`correlation`：预测力矩和录制力矩的相关性，越接近 1 越好。

`reconstruction_status`：完整参数重构状态。

`reconstruction_objective`：全局 SDP 重构时相对 URDF 先验的加权距离目标值；`method: nullspace` 时通常为空。

`strict_base_constraints`：为 `true` 时，SDP 严格满足已辨识基参数等式；为 `false` 时，SDP 在目标函数中惩罚基参数残差。录制数据存在噪声或未建模摩擦时，严格等式可能与物理可行约束冲突，推荐先使用默认的松弛模式。

`base_residual_weight`：`strict_base_constraints: false` 时基参数残差项的权重。值越大，输出参数越贴近辨识出的基参数；值过大时可能导致求解更困难或结果更接近物理边界。

`cad_constraints`：从 URDF/CAD 先验派生质量和质心一阶矩边界，并加入全局 SDP。默认质量允许 CAD 值的 80% 到 120%，质心一阶矩允许相对 10% 或绝对 `0.002 kg*m` 的余量，避免噪声基参数把完整参数推到明显不可用的范围。

`physical_projection`：逐连杆物理可行投影状态。

`physical_validation.passed`：最终写出的 URDF 是否通过质量、惯量和伪惯量物理检查。

`reconstruction_base_residual_after_projection`：启用后置逐连杆物理投影时，相对已辨识基参数的偏差。默认使用全局 SDP 重构并关闭后置投影，因此该字段通常为空；如果手动开启逐连杆投影，这个值不一定为 0，因为逐连杆投影会优先保证 URDF 参数物理可用。

## 工作流程示例

完整流程通常是：

```bash
source install/setup.bash

ros2 run parameter_identify generate_trajectory \
  --urdf src/arm/model/robotic_arm.urdf \
  --output /tmp/optimized_expected_trajectory.csv \
  --config src/parameter_identify/config/identify.yaml
```

然后启动测量节点，让机械臂执行 `/tmp/optimized_expected_trajectory.csv` 并录制数据，例如保存为：

```text
/tmp/measured_for_identification.csv
```

最后运行辨识：

```bash
ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_for_identification.csv \
  --urdf src/arm/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --report /tmp/robotic_arm_identified_report.yaml \
  --config src/parameter_identify/config/identify.yaml
```

## 注意事项

当前流程只辨识惯性参数。除非已经确认 `torque_*` 是可靠的关节侧真实力矩，并且代码中已经扩展了额外参数命名，否则建议保持下面这些配置关闭：

```yaml
identification:
  has_friction: false
  has_actuator_inertia: false
  has_joint_offset: false
```

默认情况下，完整参数重构会先使用 URDF 作为先验做零空间投影，再对每个连杆的惯性参数做物理可行投影。相比“全局 SDP 精确满足所有基参数等式”，这种方式更能容忍实机或仿真录制数据中的噪声和控制误差。

如果物理投影失败，或者最终质量、惯量、伪惯量检查失败，程序会拒绝写出校准后的 URDF，避免生成不可用于仿真的非物理模型。
