from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .identify_arm import (
    _RobotModel,
    _active_velocity_indices,
    _figaroh_identification_config,
    _remove_zero_columns,
    _select_active_joint_blocks,
    _standard_parameter_dict,
)
from .vendor import add_vendored_figaroh, load_vendored_figaroh_module


@dataclass
class TrajectoryCandidate:
    time: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    condition_number: float


@dataclass
class SimpleModel:
    nq: int
    nv: int
    lowerPositionLimit: np.ndarray
    upperPositionLimit: np.ndarray
    velocityLimit: np.ndarray


def main() -> None:
    args = _parse_args()
    config = _load_config(args.config)
    result = generate_trajectory(
        urdf_path=Path(args.urdf).resolve(),
        output_csv=Path(args.output).resolve(),
        config=config,
    )
    print(f"trajectory CSV: {result['output_csv']}")
    print(f"samples: {result['samples']}")
    print(f"duration: {result['duration']:.6g} s")
    print(f"condition number: {result['condition_number']:.6g}")
    if "rejected_by_ground" in result:
        print(f"rejected by ground clearance: {result['rejected_by_ground']}")


def generate_trajectory(*, urdf_path: Path, output_csv: Path, config: Mapping[str, Any]) -> dict[str, Any]:
    add_vendored_figaroh()

    try:
        import pinocchio as pin
    except ModuleNotFoundError:
        return _generate_surrogate_trajectory(urdf_path=urdf_path, output_csv=output_csv, config=config)

    regressor_module = load_vendored_figaroh_module(
        "_figaroh_regressor_for_trajectory", "figaroh/tools/regressor.py"
    )
    build_regressor_basic = regressor_module.build_regressor_basic

    active_joints = list(config.get("model", {}).get("active_joints", []))
    if not active_joints:
        raise ValueError("config.model.active_joints must list the trajectory joints")

    model = pin.buildModelFromUrdf(str(urdf_path))
    robot = _RobotModel(model)
    active_idxv = _active_velocity_indices(model, active_joints)
    ident_cfg = _figaroh_identification_config(config, active_joints, active_idxv)
    params_std = _standard_parameter_dict(model)

    traj_cfg = config.get("trajectory_generation", {})
    sample_time = float(traj_cfg.get("sample_time", config.get("data", {}).get("sample_time", 0.02)))
    duration = float(traj_cfg.get("duration", 12.0))
    harmonics = int(traj_cfg.get("harmonics", 5))
    attempts = int(traj_cfg.get("attempts", 250))
    seed = int(traj_cfg.get("seed", 2026))
    limit_margin = float(traj_cfg.get("limit_margin_ratio", 0.12))
    velocity_ratio = float(traj_cfg.get("velocity_limit_ratio", 0.35))
    acceleration_limit = float(traj_cfg.get("acceleration_limit", 8.0))
    ground_clearance = float(traj_cfg.get("ground_clearance", 0.0))
    ground_check_links = list(traj_cfg.get("ground_check_links", []))
    zero_tolerance = float(config.get("identification", {}).get("zero_tolerance", 1e-8))
    if not ground_check_links:
        ground_check_links = _load_urdf_child_links(urdf_path, active_joints)

    if duration <= sample_time or sample_time <= 0.0:
        raise ValueError("trajectory_generation.duration must be greater than sample_time")
    if harmonics <= 0 or attempts <= 0:
        raise ValueError("trajectory_generation.harmonics and attempts must be positive")

    rng = np.random.default_rng(seed)
    best: TrajectoryCandidate | None = None
    rejected_by_ground = 0
    for _ in range(attempts):
        candidate = _sample_multiharmonic_candidate(
            model=model,
            active_idxv=active_idxv,
            duration=duration,
            sample_time=sample_time,
            harmonics=harmonics,
            limit_margin=limit_margin,
            velocity_ratio=velocity_ratio,
            acceleration_limit=acceleration_limit,
            rng=rng,
        )
        if not _candidate_satisfies_ground_clearance(
            pin=pin,
            model=model,
            candidate=candidate,
            frame_names=ground_check_links,
            min_z=ground_clearance,
        ):
            rejected_by_ground += 1
            continue
        W_full = build_regressor_basic(robot, candidate.positions, candidate.velocities, candidate.accelerations, ident_cfg)
        W_active = _select_active_joint_blocks(W_full, active_idxv, candidate.positions.shape[0])
        tau_dummy = np.zeros(W_active.shape[0])
        W_reduced, _, _ = _remove_zero_columns(
            W_active,
            tau_dummy,
            params_std,
            zero_tolerance=zero_tolerance,
        )
        condition_number = _scaled_condition_number(W_reduced)
        candidate.condition_number = condition_number
        if best is None or candidate.condition_number < best.condition_number:
            best = candidate

    if best is None:
        raise RuntimeError(
            "Failed to generate any trajectory candidate. "
            f"Rejected {rejected_by_ground} candidates by ground clearance; "
            "try increasing attempts, reducing limit_margin_ratio, or lowering ground_clearance."
        )

    _write_position_csv(output_csv, best.time, best.positions[:, active_idxv])
    return {
        "output_csv": str(output_csv),
        "samples": int(best.positions.shape[0]),
        "duration": float(best.time[-1]),
        "condition_number": float(best.condition_number),
        "rejected_by_ground": int(rejected_by_ground),
    }


def _generate_surrogate_trajectory(*, urdf_path: Path, output_csv: Path, config: Mapping[str, Any]) -> dict[str, Any]:
    active_joints = list(config.get("model", {}).get("active_joints", []))
    if not active_joints:
        raise ValueError("config.model.active_joints must list the trajectory joints")

    lower, upper, velocity = _load_urdf_joint_limits(urdf_path, active_joints)
    model = SimpleModel(
        nq=len(active_joints),
        nv=len(active_joints),
        lowerPositionLimit=lower,
        upperPositionLimit=upper,
        velocityLimit=velocity,
    )
    active_idxv = list(range(len(active_joints)))

    traj_cfg = config.get("trajectory_generation", {})
    sample_time = float(traj_cfg.get("sample_time", config.get("data", {}).get("sample_time", 0.02)))
    duration = float(traj_cfg.get("duration", 12.0))
    harmonics = int(traj_cfg.get("harmonics", 5))
    attempts = int(traj_cfg.get("attempts", 250))
    seed = int(traj_cfg.get("seed", 2026))
    limit_margin = float(traj_cfg.get("limit_margin_ratio", 0.12))
    velocity_ratio = float(traj_cfg.get("velocity_limit_ratio", 0.35))
    acceleration_limit = float(traj_cfg.get("acceleration_limit", 8.0))
    ground_clearance = float(traj_cfg.get("ground_clearance", 0.0))
    if ground_clearance > -np.inf:
        raise RuntimeError(
            "trajectory_generation.ground_clearance requires Pinocchio so FK can be checked. "
            "Run from an environment where `import pinocchio` succeeds."
        )

    rng = np.random.default_rng(seed)
    best: TrajectoryCandidate | None = None
    for _ in range(attempts):
        candidate = _sample_multiharmonic_candidate(
            model=model,
            active_idxv=active_idxv,
            duration=duration,
            sample_time=sample_time,
            harmonics=harmonics,
            limit_margin=limit_margin,
            velocity_ratio=velocity_ratio,
            acceleration_limit=acceleration_limit,
            rng=rng,
        )
        candidate.condition_number = _effective_condition_number(_surrogate_feature_matrix(candidate))
        if best is None or candidate.condition_number < best.condition_number:
            best = candidate

    if best is None:
        raise RuntimeError("Failed to generate any trajectory candidate")

    _write_position_csv(output_csv, best.time, best.positions)
    return {
        "output_csv": str(output_csv),
        "samples": int(best.positions.shape[0]),
        "duration": float(best.time[-1]),
        "condition_number": float(best.condition_number),
    }


def _sample_multiharmonic_candidate(
    *,
    model: Any,
    active_idxv: list[int],
    duration: float,
    sample_time: float,
    harmonics: int,
    limit_margin: float,
    velocity_ratio: float,
    acceleration_limit: float,
    rng: np.random.Generator,
) -> TrajectoryCandidate:
    time = np.arange(0.0, duration + 0.5 * sample_time, sample_time)
    omega = 2.0 * np.pi / duration
    q = np.zeros((time.size, model.nq))
    dq = np.zeros((time.size, model.nv))
    ddq = np.zeros((time.size, model.nv))

    lower = np.asarray(model.lowerPositionLimit, dtype=float)
    upper = np.asarray(model.upperPositionLimit, dtype=float)
    velocity = np.asarray(model.velocityLimit, dtype=float)

    for idxv in active_idxv:
        joint_range = upper[idxv] - lower[idxv]
        center = 0.5 * (lower[idxv] + upper[idxv])
        usable_amplitude = max(0.0, 0.5 * joint_range * (1.0 - 2.0 * limit_margin))
        if usable_amplitude <= 0.0:
            continue

        raw_sin = rng.normal(0.0, 1.0, size=harmonics) / np.arange(1, harmonics + 1)
        raw_cos = rng.normal(0.0, 1.0, size=harmonics) / np.arange(1, harmonics + 1)
        phases = np.outer(time, np.arange(1, harmonics + 1) * omega)
        q_raw = np.sum(raw_sin * np.sin(phases) + raw_cos * (np.cos(phases) - 1.0), axis=1)
        dq_raw = np.sum(
            np.arange(1, harmonics + 1) * omega * (raw_sin * np.cos(phases) - raw_cos * np.sin(phases)),
            axis=1,
        )
        ddq_raw = np.sum(
            -((np.arange(1, harmonics + 1) * omega) ** 2)
            * (raw_sin * np.sin(phases) + raw_cos * np.cos(phases)),
            axis=1,
        )

        scale = usable_amplitude / max(np.max(np.abs(q_raw)), 1e-12)
        if np.isfinite(velocity[idxv]) and velocity[idxv] > 0.0:
            scale = min(scale, velocity_ratio * velocity[idxv] / max(np.max(np.abs(dq_raw)), 1e-12))
        if acceleration_limit > 0.0:
            scale = min(scale, acceleration_limit / max(np.max(np.abs(ddq_raw)), 1e-12))

        q[:, idxv] = np.clip(center + scale * q_raw, lower[idxv], upper[idxv])
        dq[:, idxv] = scale * dq_raw
        ddq[:, idxv] = scale * ddq_raw

    return TrajectoryCandidate(time=time, positions=q, velocities=dq, accelerations=ddq, condition_number=np.inf)


def _scaled_condition_number(W: np.ndarray) -> float:
    norms = np.linalg.norm(W, axis=0)
    keep = norms > 1e-12
    if not np.any(keep):
        return np.inf
    W_scaled = W[:, keep] / norms[keep]
    return float(np.linalg.cond(W_scaled))


def _effective_condition_number(W: np.ndarray) -> float:
    norms = np.linalg.norm(W, axis=0)
    keep = norms > 1e-12
    if not np.any(keep):
        return np.inf
    W_scaled = W[:, keep] / norms[keep]
    singular_values = np.linalg.svd(W_scaled, compute_uv=False)
    effective = singular_values[singular_values > singular_values[0] * 1e-9]
    if effective.size < 2:
        return np.inf
    return float(effective[0] / effective[-1])


def _surrogate_feature_matrix(candidate: TrajectoryCandidate) -> np.ndarray:
    q = candidate.positions
    dq = candidate.velocities
    ddq = candidate.accelerations
    features = [q, dq, ddq, np.sin(q), np.cos(q)]
    for joint in range(q.shape[1]):
        features.append((q[:, joint : joint + 1] * dq))
        features.append((dq[:, joint : joint + 1] * dq))
    return np.hstack(features)


def _candidate_satisfies_ground_clearance(
    *,
    pin: Any,
    model: Any,
    candidate: TrajectoryCandidate,
    frame_names: list[str],
    min_z: float,
) -> bool:
    data = model.createData()
    frame_ids = []
    for frame_name in frame_names:
        frame_id = model.getFrameId(frame_name)
        if frame_id >= len(model.frames):
            raise ValueError(f"Ground-check frame/link {frame_name!r} was not found in the URDF model")
        frame_ids.append(frame_id)

    for q in candidate.positions:
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        for frame_id in frame_ids:
            if float(data.oMf[frame_id].translation[2]) <= min_z:
                return False
    return True


def _load_urdf_child_links(urdf_path: Path, active_joints: list[str]) -> list[str]:
    tree = ET.parse(urdf_path)
    child_links = {}
    for joint in tree.getroot().findall("joint"):
        name = joint.attrib.get("name", "")
        child = joint.find("child")
        if name and child is not None:
            child_links[name] = child.attrib.get("link", "")

    links = []
    for joint_name in active_joints:
        link_name = child_links.get(joint_name, "")
        if not link_name:
            raise ValueError(f"Joint {joint_name!r} child link was not found in {urdf_path}")
        links.append(link_name)
    return links


def _load_urdf_joint_limits(urdf_path: Path, active_joints: list[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tree = ET.parse(urdf_path)
    joints = {}
    for joint in tree.getroot().findall("joint"):
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        joints[name] = (
            float(limit.attrib.get("lower", "-3.141592653589793")),
            float(limit.attrib.get("upper", "3.141592653589793")),
            float(limit.attrib.get("velocity", "1.0")),
        )

    lower = []
    upper = []
    velocity = []
    for joint_name in active_joints:
        if joint_name not in joints:
            raise ValueError(f"Joint {joint_name!r} with limits was not found in {urdf_path}")
        lo, hi, vel = joints[joint_name]
        lower.append(lo)
        upper.append(hi)
        velocity.append(vel)
    return np.asarray(lower, dtype=float), np.asarray(upper, dtype=float), np.asarray(velocity, dtype=float)


def _write_position_csv(path: Path, time: np.ndarray, positions: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time", *[f"pos_{idx}" for idx in range(positions.shape[1])]])
        for row_time, row_pos in zip(time, positions):
            writer.writerow([f"{float(row_time):.9f}", *[f"{float(value):.9f}" for value in row_pos]])


def _parse_args() -> argparse.Namespace:
    default_config = _default_config_path()
    default_output = Path.cwd() / "optimized_expected_trajectory.csv"
    parser = argparse.ArgumentParser(description="Generate an optimized expected joint-position trajectory CSV.")
    parser.add_argument("--urdf", required=True, help="Robot URDF path.")
    parser.add_argument("--output", default=str(default_output), help="Output trajectory CSV path.")
    parser.add_argument("--config", default=str(default_config), help="Identification YAML config.")
    return parser.parse_args()


def _default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("parameter_identify")) / "config" / "identify.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "identify.yaml"


def _load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


if __name__ == "__main__":
    main()
