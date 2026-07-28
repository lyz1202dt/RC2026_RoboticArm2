# parameter_identify

Offline dynamic parameter identification for the RC2026 six-axis robotic arm.

The package consumes the CSV produced by `parameter_measure`:

```text
time,pos_0..pos_5,vel_0..vel_5,acc_0..acc_5,torque_0..torque_5
```

It loads the original URDF with Pinocchio, builds the FIGAROH joint-torque
regressor, extracts identifiable base parameters with double QR, reconstructs
full inertial parameters using the URDF values as the prior, and writes an
identified URDF.

## Dependency

FIGAROH is vendored in:

```bash
third_party/figaroh-plus
```

The command loads only the needed FIGAROH source files, so optional FIGAROH
dependencies for visualisation and trajectory optimisation are not required.
Python `pinocchio`, `numpy`, `scipy`, and `pyyaml` must be available.

## Usage

From a sourced workspace after building:

```bash
ros2 run parameter_identify identify_arm \
  --csv /path/to/parameter_measure.csv \
  --urdf /space2/Project/RC2026_RoboticArm2/src/arm/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf
```

From source without installing the ROS package:

```bash
cd /space2/Project/RC2026_RoboticArm2/src/parameter_identify
python3 -m parameter_identify.identify_arm \
  --csv /path/to/parameter_measure.csv \
  --urdf /space2/Project/RC2026_RoboticArm2/src/arm/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf
```

The tool also writes a YAML report next to the output URDF unless `--report` is
provided explicitly.

## Notes

The first version identifies inertial parameters only. Keep
`has_friction`, `has_actuator_inertia`, and `has_joint_offset` disabled in the
config until the measured `torque_*` columns are verified to be true joint-side
torques and the extra parameter naming is extended.
