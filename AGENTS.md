# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 (Humble) project for a 6-axis robotic arm with KDL kinematics and MuJoCo simulation. Source code lives under `src/` with these packages:

| Package | Purpose |
|---|---|
| `arm` | URDF robot model & config |
| `arm_calc` | KDL kinematics solver & arm controller (`arm_calc`, `arm_action`) |
| `arm_task` | Task state machine (idle, catch_kfs, move_kfs, place_kfs) |
| `robot_driver` | Serial/CDC communication with real hardware |
| `robot_interfaces` | Custom ROS 2 msgs (`Arm.msg`, `Joint.msg`) & actions (`ArmTask.action`) |
| `vision` | Visual target detection & TF publishing |
| `dog_controller` | Quadruped controller plugin |
| `mujoco_ros2_control` | MuJoCo simulation hardware interface |
| `launch_pack` | All launch files & RViz configs |

Tests are embedded within packages (e.g. `arm_task/src/catch_kfs_test.cpp`). Build output goes to `build/`, `install/`, `log/` (all gitignored).

## Build, Test, and Development Commands

```bash
# Build all packages
colcon build --symlink-install

# Build a single package
colcon build --packages-select arm_task

# Run tests for a package
colcon test --packages-select arm_task

# Launch MuJoCo simulation
ros2 launch launch_pack arm_task_sim.launch.py

# Launch real robot
ros2 launch launch_pack arm_real.launch.py

# Static URDF display (no simulation)
ros2 launch launch_pack arm_static_display.launch.py
```

## Coding Style & Naming Conventions

- **Standard:** C++20
- **Formatting:** `.clang-format` (LLVM-based, 4-space indent, 140 column limit, pointer left-aligned)
- **Linting:** `.clang-tidy` (bugprone, modernize, performance, readability checks)
- **Naming:** `snake_case` for files, functions, and variables; `CamelCase` for classes; header guards use full paths
- Run `clang-format` before committing: `find src/ -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i`

## Testing Guidelines

- Tests live alongside source in each package (e.g. `*_test.cpp` files)
- Use `colcon test` to run; `colcon test-result --verbose` to inspect results
- Test naming: `catch_kfs_test.cpp`, `place_kfs_test.cpp` — match the task they validate
- Always validate after changes per `memory-bank/rules.md`: do not proceed until results are confirmed

## Commit & Pull Request Guidelines

- Commit messages are brief, in Chinese or English; feature commits may use `feat(scope):` prefix (e.g. `feat(mujoco): 添加场景中的目标盒子`)
- Keep messages concise and task-specific
- Record ongoing changes in `memory-bank/progress.md` per project rules
- PRs should describe scope of change, reference relevant launch configs or packages affected

## Agent-Specific Instructions

Follow the rules in `memory-bank/rules.md` strictly:
1. **Think before coding** — state assumptions; ask when uncertain
2. **Simplest code first** — no speculative extensions or premature abstraction
3. **Surgical edits** — only change what is required; don't refactor unrelated code
4. **Record changes** in `memory-bank/progress.md`
5. **Validate after every change** — run relevant tests before moving on
