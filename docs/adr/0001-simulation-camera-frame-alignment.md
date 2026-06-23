# ADR-0001: Align simulation camera frame with launch-defined `Link4 -> camera_link`

- Status: Accepted
- Date: 2026-06-23

## Context

`vision.cpp` expects image and depth messages to carry `frame_id = "camera_link"`
and looks up `base_link -> Link4 -> camera_link` to project pixel detections
into `base_link` before publishing `base_link -> target_object` for `arm_task`.

The launch file `launch_pack/launch/arm_mujoco_sim.launch.py` declares a static
TF `Link4 -> camera_link` with translation `(0.1, 0.09, -0.03)` m and
quaternion `(0, 0.7071, 0, 0.7071)`. Per the project's conflict-resolution
rule, this launch declaration is the single source of truth for the
hand-eye transform.

The MJCF `src/arm/model/robotic_arm.xml` previously had a `<camera>` directly
under `Link4` with the wrong sign on the y offset and a different orientation.
This made the simulation camera's published `frame_id` equal to `Link4`
(instead of `camera_link`), and broke the TF tree the vision pipeline assumes.
The simulation also had no `Link4 -> camera_link` static transform wired in
the scene itself, only the launch-level one.

Additionally, `vision.cpp` had a hard-coded `T_flange2cam_` matrix that did
not match the launch-level static transform (y/z signs differed), so even
with a correctly-named camera frame, the projected base pose would have
been wrong.

## Decision

1. In MJCF, replace the existing `<camera>` under `Link4` with a new
   `<body name="camera_link" pos="0.1 0.09 -0.03" xyaxes="0 0 1  0 1 0">`
   containing a single `<camera name="top_rgbd" mode="fixed" fovy="55"/>`.
   This makes `MujocoDepthCamera` emit `frame_id = "camera_link"` and lets
   the camera frame move with the flange, matching the launch-level static
   transform geometrically.

2. Align `vision.cpp`'s hard-coded `T_flange2cam_` with the canonical values
   from the launch-level `static_transform_publisher`:
   translation `(0.1, 0.09, -0.03)` m, rotation about Y by +90 degrees.

3. Do not delete `vision.cpp`'s internal `publishStaticHandEyeTransform()`
   in this change. It is a redundant publisher of the same `Link4 ->
   camera_link` transform. Per the conflict-resolution rule the launch file
   wins; the internal publish is acceptable as a fallback. A follow-up ADR
   may remove it once dual-publish behavior is confirmed harmless.

## Consequences

- The vision pipeline can be driven from simulated `MujocoDepthCamera`
  images with no change to `vision.cpp`'s TF lookups.
- The simulation's geometric camera pose is defined by the launch file's
  static transform; the MJCF pose is written to match it. Future contributors
  must keep both in sync (the canonical values are recorded in `CONTEXT.md`).
- `arm_task` consumers that look up `base_link -> target_object` continue to
  work without modification.
- Until the redundant `publishStaticHandEyeTransform()` in `vision.cpp` is
  removed, two nodes publish `Link4 -> camera_link`. The launch-level
  publisher is the source of truth.
