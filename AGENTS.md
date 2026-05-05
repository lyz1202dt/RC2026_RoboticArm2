# AI Agent Guide for RC2026 Robotic Arm Project

This is a ROS2-based 6-DOF robotic arm project for Robocon 2026, featuring inverse kinematics (KDL), Mujoco simulation, and real hardware control. AI agents should understand this architecture and development practices before making changes.

## Project Overview

**Technology Stack:**
- ROS2 (latest)
- MuJoCo simulation engine
- KDL (Kinematics and Dynamics Library) for IK solving
- CMake + colcon build system
- C++17
- Python 3 (launch files, test nodes)
- YAML configs (task definitions)

**Core Packages:**
| Package | Role |
|---------|------|
| `arm` | URDF model, static transforms |
| `arm_calc` | IK computation, Cartesian trajectory planning, joint trajectory sampling |
| `arm_task` | High-level task execution (catch, place, move KFS objects) |
| `robot_driver` | Hardware communication layer |
| `robot_interfaces` | Custom ROS2 interfaces/messages |
| `vision` | Object detection and TF broadcasting |
| `mujoco_ros2_control` | Simulation bridge to ros2_control |
| `dog_controller` | Simulated target object controller |
| `launch_pack` | Launch files for all scenarios |

## Build & Test Commands

**Full build:**
```bash
colcon build  # Builds all packages in src/
```

**Build specific package:**
```bash
colcon build --packages-select arm_calc
```

**Run tests:**
```bash
colcon build --packages-select <package> && \
  source install/setup.bash && \
  ros2 run <package> <test_node_name>
```

**Important:** After modifying ROS2 launch Python files, validate syntax:
```bash
python3 -m py_compile src/launch_pack/launch/*.py
# Note: colcon build does not always catch launch script syntax errors
```

## Launch Configurations

Choose the right launch file for your scenario. See [plan.md](plan.md) for detailed launch matrix:

- **`arm_mujoco_sim.launch.py`** – MuJoCo simulation with full arm_calc, arm_task
- **`arm_real.launch.py`** – Real hardware with arm_driver, arm_calc, vision
- **`arm_task_sim.launch.py`** – Task execution in simulation
- **`arm_task_target_test.launch.py`** – Task execution + static target TF
- **`arm_static_display.launch.py`** – URDF visualization for model debugging
- **`vision_tf_test.launch.py`** – Vision + TF broadcasting test (no arm_task)

## Architecture & Key Patterns

### Control Flow: Task → Arm Calc → Robot Driver

```
arm_task (task sequencer) 
  → publishes target poses, calls RPC
  → arm_calc (trajectory planner)
    → samples via IK (KDL)
    → publishes joint commands
  → robot_driver or mujoco_ros2_control (joint control)
```

### Inverse Kinematics (IK) Pipeline

**Location:** [src/arm_calc/src/arm_calc/](src/arm_calc/src/arm_calc/)

- **Problem:** KDL's `ChainIkSolverPos_LMA` can return multiple solutions; seed selection affects trajectory smoothness.
- **Known Issue:** Cartesian trajectory sampling was not inheriting the start joint state as IK seed, causing joint1 to "flip" at trajectory start.  
  **Fix:** Ensure `JCartesianSpaceMove::sample()` passes `start_joint_state_.position` as seed to `signal_arm_calc()`.  
  See [src/arm_calc/src/arm_action/basic_moves.cpp](src/arm_calc/src/arm_action/basic_moves.cpp#L150) and [memory-bank/progress.md](memory-bank/progress.md).

### Task Definitions

**Location:** [src/arm_task/config/](src/arm_task/config/)

- Tasks are stored as YAML configs with named joint poses and Cartesian trajectories.
- Each task (e.g., `catch_kfs`, `place_kfs`) has named waypoints like `kfs1_touch_pos`, `place_interim_pos_left`.
- Load at startup: `arm_task` parses YAML and publishes TF frames.

### Simulation vs. Hardware

- **Simulation:** `mujoco_ros2_control` bridges MuJoCo to ROS2; no real hardware latency.
- **Real:** `robot_driver` communicates with actual arm controller; watch for CAN/serial latency.
- Cartesian trajectories behave similarly in both; validate in simulation first, then test on real hardware.

## Development Rules

Follow the conventions in [memory-bank/rules.md](memory-bank/rules.md):

1. **Think before coding.** Do not assume. Write assumptions explicitly; ask if unclear.
2. **Simplicity first.** Do not add features beyond requirements. Do not over-engineer.
3. **Surgical edits.** Only change what is necessary. Do not refactor unrelated code.
4. **Log your changes.** Update [memory-bank/progress.md](memory-bank/progress.md) with what you did and why.
5. **Verify after each change.** Run relevant tests. Do not proceed to the next change until verification is complete.

## Common Development Tasks

### Adding a New Task
1. Define joint poses in YAML config → [src/arm_task/config/](src/arm_task/config/)
2. Add Cartesian trajectory waypoints if needed
3. Update [src/arm_task/src/task/](src/arm_task/src/task/) with a `.cpp` task handler
4. Register in [src/arm_task/src/robot.cpp](src/arm_task/src/robot.cpp) task dispatcher
5. Test: run `arm_task_sim.launch.py` or `arm_task_target_test.launch.py`

### Debugging a Trajectory Issue
- **Symptom:** Joint angle jumps, end-effector deviates, collision detected.
- **First check:** Is the IK seed the current joint state? See [src/arm_calc/src/arm_action/](src/arm_calc/src/arm_action/).
- **Second check:** Are Cartesian waypoints reachable? Test with `arm_test_node` (see [src/arm_calc/src/arm_test_node.cpp](src/arm_calc/src/arm_test_node.cpp)).
- **Third check:** Compare pose targets with actual arm position in RViz.

### Modifying IK Solver Behavior
- **File:** [src/arm_calc/src/arm_calc/arm_calc.cpp](src/arm_calc/src/arm_calc/arm_calc.cpp#L140)
- **Key variables:** `last_joint_solution_` (seed history), `joint_pos` (current state)
- **Rule:** Always pass current or intended seed explicitly to avoid solution flipping.

### Testing Before Pushing to Hardware
1. Run in `arm_mujoco_sim.launch.py` with the same task.
2. Verify joint trajectories in RViz match expectations.
3. Check arm_calc console for IK convergence warnings.
4. Then test on real hardware with reduced speed parameters.

## Common Pitfalls

- **Pitfall:** Assuming `colcon build` validates launch script syntax.  
  **Solution:** Always run `python3 -m py_compile src/launch_pack/launch/*.py` after edits.

- **Pitfall:** IK seed not matching start state; causes joint1 (or others) to flip at trajectory boundaries.  
  **Solution:** Explicitly pass start joint state as seed to IK solver.

- **Pitfall:** Cartesian targets out of reach; trajectory planning fails silently.  
  **Solution:** Pre-check reachability with test node before integrating into task.

- **Pitfall:** Real hardware latency vs. simulation; task timing differs.  
  **Solution:** Add safety margin to trajectory duration; test with reduced speeds first.

## Key Files by Function

**IK & Trajectory Planning:**
- [src/arm_calc/src/arm_calc/arm_calc.cpp](src/arm_calc/src/arm_calc/arm_calc.cpp) – Core IK solver interface
- [src/arm_calc/src/arm_action/basic_moves.cpp](src/arm_calc/src/arm_action/basic_moves.cpp) – Cartesian move implementation

**Task Execution:**
- [src/arm_task/src/robot.cpp](src/arm_task/src/robot.cpp) – Task dispatcher, state machine
- [src/arm_task/src/task/catch_kfs.cpp](src/arm_task/src/task/catch_kfs.cpp) – Catch task handler
- [src/arm_task/src/task/place_kfs.cpp](src/arm_task/src/task/place_kfs.cpp) – Place task handler

**Models & Config:**
- [src/arm/model/robotic_arm.urdf](src/arm/model/robotic_arm.urdf) – Robot kinematics
- [src/arm_task/config/](src/arm_task/config/) – Named poses, trajectory definitions

**Testing:**
- [src/arm_calc/src/arm_test_node.cpp](src/arm_calc/src/arm_test_node.cpp) – Manual IK/trajectory test
- [src/arm_task/src/catch_kfs_test.cpp](src/arm_task/src/catch_kfs_test.cpp) – Task scenario testing

## Progress & Known Issues

See [memory-bank/progress.md](memory-bank/progress.md) for current work status.

**Recent fix (2026-05-03):** Joint1 offset in Cartesian trajectories was caused by IK seed not being inherited from start joint state. Fixed in [src/arm_calc/src/arm_action/basic_moves.cpp](src/arm_calc/src/arm_action/basic_moves.cpp#L150) by passing `start_joint_state_.position` as seed. Runtime verification pending.

## Questions?

- Reachability tests, IK issues → Check [src/arm_calc/src/arm_calc/arm_calc.cpp](src/arm_calc/src/arm_calc/arm_calc.cpp)
- Task definitions, sequencing → Check [src/arm_task/config/](src/arm_task/config/) and [src/arm_task/src/robot.cpp](src/arm_task/src/robot.cpp)
- Model geometry, TF tree → Check [src/arm/model/robotic_arm.urdf](src/arm/model/robotic_arm.urdf) and [src/launch_pack/launch/](src/launch_pack/launch/)
- Simulation-specific → Check [src/mujoco_ros2_control/](src/mujoco_ros2_control/)
