# Context

Ubiquitous language for the robotic arm project. Implementation-free; capture
only the concepts and the canonical names that flow through the codebase.

## Vision pipeline

- **vision pipeline**: the chain of nodes and topics that turn camera pixels
  into a 3D pose in `base_link` for downstream tasks (visual servoing,
  pick-and-place).
- **frame `camera_link`**: the canonical camera frame for the `vision` node.
  All image, depth, and point-cloud messages in the vision pipeline carry
  `frame_id = "camera_link"`. The frame is defined relative to the flange
  `Link4` by the static transform `Link4 -> camera_link`.
- **canonical hand-eye transform (`T_flange2cam`)**: the value the launch
  file's `static_transform_publisher` publishes for `Link4 -> camera_link`.
  Translation `(0.1, 0.09, -0.03)` m, quaternion `(0, 0.7071, 0, 0.7071)`
  (rotation about Y by +90 degrees). This is the single source of truth. Any
  code that hard-codes a different value (currently `vision.cpp`'s
  `T_flange2cam_`) must be aligned to it.
- **object frame `target_object`**: the child frame the `vision` node
  publishes under `base_link` to carry the detected object pose. Downstream
  consumers (`arm_task`, in particular its `visual_serve` and `catch_kfs`
  components) look up `base_link -> target_object`.
- **simulation camera body**: in MJCF, `<body name="camera_link">` lives
  under `Link4` so that the `MujocoDepthCamera` plugin publishes
  `frame_id = "camera_link"` and the camera frame moves with the flange.
- **conflict resolution**: when a value appears in both a launch file and a
  node's source, the launch file wins.

## Existing terms (recorded here for clarity)

- **`base_link` / `Link4` / `camera_link`**: the three frames the vision
  pipeline needs.
- **red-bar target / `target_object`**: the red object the `vision` node
  currently detects and publishes.
