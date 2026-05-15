# AGENTS.md — AI 代理快速指南

目的：为 AI 编程代理提供足够的上下文，使其能在本 ROS2 工作区里安全、有效地执行常见开发任务（构建、运行、查找入口文件、排查常见问题），并指引到详细文档而不重复已有内容。

- **主 README**：请参阅 [README.md](README.md) 获取项目总体说明和部分运行示例。

快速指令（在工作区根目录执行）：

- 设置环境：

  source install/local_setup.bash

- 构建：

  colcon build --symlink-install

- 运行（示例）：

  ros2 launch launch_pack arm_task_move_test.launch.py

- 测试：

  colcon test --event-handlers console_direct+

关键目录（常查）：

- `robot_driver` — 驱动层
- `arm_calc` — 运动学/解算
- `arm_task` — 任务/高层控制
- `launch_pack` — ROS2 launch 脚本集合
- `mujoco_ros2_control` — 仿真/控制桥

代码/约定要点：

- 仓库使用 ROS2+colcon。默认构建工具为 `colcon`。
- Launch 脚本多为 Python 文件，修改后建议先运行 `python3 -m py_compile <launch-file>` 来检测语法/缩进问题（colcon build 不一定会暴露这些错误）。
- 仿真相关启动通常在 `launch_pack` 下，优先查找该目录。

常见陷阱：

- 未 `source` install 或系统的 ROS2 环境会导致节点找不到或 launch 失败。
- Python launch 文件的缩进/语法错误常被忽略；对这些文件使用 `python3 -m py_compile` 可快速验证。

AI 代理行为指南（对代理的明确指令）：

- 优先在本文件（`AGENTS.md`）与项目 README 中查找信息，避免复制冗余文档。
- 任何构建/运行命令先在本地终端复现并报告错误输出；若需要更改代码，先做小范围修改并运行单元/集成测试。
- 不要执行破坏性操作（删除文件、强制覆盖远程仓库等）除非获得明确许可。
- 修改或新增 launch 脚本时，先用 `python3 -m py_compile` 检查，再运行相应的 `ros2 launch` 验证。

进一步阅读与排查入口：

- 项目主说明： [README.md](README.md)
- 需要查找的启动包： `launch_pack` 目录

若你是人类协作者，想让我继续：建议我可创建 `.github/copilot-instructions.md`（用于在 PR/CI 中提示 agent 行为），或为 `launch_pack` 单独写一份更详尽的 skill 文件。欢迎指示下一步。