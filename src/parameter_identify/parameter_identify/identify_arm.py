from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .csv_loader import load_measurement_csv
from .urdf_writer import INERTIAL_KEYS, write_identified_urdf
from .vendor import add_vendored_figaroh, load_vendored_figaroh_module


def main() -> None:
    args = _parse_args()
    config = _load_config(args.config)

    csv_path = Path(args.csv).resolve()
    input_urdf = Path(args.urdf).resolve()
    output_urdf = Path(args.output).resolve()
    report_path = Path(args.report).resolve() if args.report else output_urdf.with_suffix(".identify.yaml")

    result = identify(
        csv_path=csv_path,
        input_urdf=input_urdf,
        output_urdf=output_urdf,
        config=config,
        report_path=report_path,
    )

    print(f"identified URDF: {result['output_urdf']}")
    print(f"report: {result['report']}")
    print(f"updated links: {', '.join(result['updated_links'])}")
    print(f"base parameter count: {result['base_parameter_count']}")
    print(f"rmse: {result['rmse']:.6g}")
    print(f"correlation: {result['correlation']:.6g}")


def identify(
    *,
    csv_path: Path,
    input_urdf: Path,
    output_urdf: Path,
    config: Mapping[str, Any],
    report_path: Path,
) -> dict[str, Any]:
    add_vendored_figaroh()

    import pinocchio as pin

    regressor_module = load_vendored_figaroh_module(
        "_figaroh_regressor", "figaroh/tools/regressor.py"
    )
    qr_module = load_vendored_figaroh_module(
        "_figaroh_qrdecomposition", "figaroh/tools/qrdecomposition.py"
    )
    reconstruction_module = load_vendored_figaroh_module(
        "_figaroh_reconstruction", "figaroh/identification/reconstruction.py"
    )
    build_regressor_basic = regressor_module.build_regressor_basic
    QRDecomposer = qr_module.QRDecomposer
    BaseResult = reconstruction_module.BaseResult
    reconstruct_full_parameters = reconstruction_module.reconstruct_full_parameters

    active_joints = list(config.get("model", {}).get("active_joints", []))
    if not active_joints:
        raise ValueError("config.model.active_joints must list the identified joints")

    dof = len(active_joints)
    data_cfg = config.get("data", {})
    data = load_measurement_csv(csv_path, dof=dof, max_samples=int(data_cfg.get("max_samples", 0)))

    model = pin.buildModelFromUrdf(str(input_urdf))
    robot = _RobotModel(model)

    active_idxv = _active_velocity_indices(model, active_joints)
    ident_cfg = _figaroh_identification_config(config, active_joints, active_idxv)

    W_full = build_regressor_basic(
        robot,
        data.positions,
        data.velocities,
        data.accelerations,
        ident_cfg,
    )
    W_active = _select_active_joint_blocks(W_full, active_idxv, data.positions.shape[0])
    tau_active = _stack_active_torques(data.torques, len(active_idxv))

    params_std = _standard_parameter_dict(model)
    W_reduced, tau_reduced, params_reduced = _remove_zero_columns(
        W_active,
        tau_active,
        params_std,
        zero_tolerance=float(config.get("identification", {}).get("zero_tolerance", 1e-8)),
    )

    if bool(data_cfg.get("decimate", False)):
        W_reduced, tau_reduced = _decimate_stacked_rows(
            W_reduced,
            tau_reduced,
            n_active=len(active_idxv),
            factor=int(data_cfg.get("decimation_factor", 10)),
        )

    decomposer = QRDecomposer(
        tolerance=float(config.get("identification", {}).get("qr_tolerance", 1e-8))
    )
    W_base, base_param_dict, base_expr, phi_base, phi_base_nominal = decomposer.double_decomposition(
        tau_reduced,
        W_reduced,
        params_reduced,
        params_std,
    )

    tau_estimated = W_base @ phi_base
    residual = tau_reduced - tau_estimated
    full_parameters = dict(params_std)

    recon_cfg = config.get("reconstruction", {})
    if bool(recon_cfg.get("enabled", True)):
        base_result = BaseResult(
            M=np.asarray(decomposer.get_M(), dtype=float),
            phi_base=np.asarray(phi_base, dtype=float).reshape(-1),
            params_r=list(decomposer.get_M_labels()[1] or params_reduced),
        )
        recon = reconstruct_full_parameters(
            base_result,
            method=str(recon_cfg.get("method", "nullspace")),
            params_std_prior=params_std,
            prior_source=str(recon_cfg.get("prior_source", "dict")),
            model=model,
            joint_names=list(model.names[1:]),
            mass_min=float(recon_cfg.get("mass_min", 1e-6)),
        )
        full_parameters.update(recon.as_dict())
        reconstruction_status = recon.status
        reconstruction_residual = recon.base_residual_norm
    else:
        reconstruction_status = "disabled"
        reconstruction_residual = None

    updated_links = write_identified_urdf(
        input_urdf=input_urdf,
        output_urdf=output_urdf,
        parameter_dict=full_parameters,
        joint_names=list(model.names[1:]),
    )

    report = {
        "csv": str(csv_path),
        "input_urdf": str(input_urdf),
        "output_urdf": str(output_urdf),
        "samples": int(data.positions.shape[0]),
        "identified_rows": int(tau_reduced.shape[0]),
        "active_joints": active_joints,
        "model_joints": list(model.names[1:]),
        "base_parameter_count": int(len(phi_base)),
        "base_parameters": {name: float(value) for name, value in base_param_dict.items()},
        "base_expressions": list(base_expr),
        "rmse": float(np.sqrt(np.mean(residual * residual))),
        "correlation": _correlation(tau_reduced, tau_estimated),
        "reconstruction_status": reconstruction_status,
        "reconstruction_residual_norm": reconstruction_residual,
        "updated_links": updated_links,
        "identified_inertial_parameters": _inertial_parameter_subset(full_parameters, list(model.names[1:])),
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)

    return {
        "output_urdf": str(output_urdf),
        "report": str(report_path),
        "updated_links": updated_links,
        "base_parameter_count": int(len(phi_base)),
        "rmse": report["rmse"],
        "correlation": report["correlation"],
    }


def _parse_args() -> argparse.Namespace:
    default_config = Path(__file__).resolve().parents[1] / "config" / "identify.yaml"
    parser = argparse.ArgumentParser(description="Identify arm inertial parameters and export an updated URDF.")
    parser.add_argument("--csv", required=True, help="CSV recorded by parameter_measure.")
    parser.add_argument("--urdf", required=True, help="Original URDF path.")
    parser.add_argument("--output", required=True, help="Output identified URDF path.")
    parser.add_argument(
        "--config",
        default=str(default_config),
        help="Identification YAML config.",
    )
    parser.add_argument("--report", default="", help="Optional report YAML path.")
    return parser.parse_args()


def _load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


class _RobotModel:
    def __init__(self, model: Any) -> None:
        self.model = model
        self.data = model.createData()


def _figaroh_identification_config(
    config: Mapping[str, Any],
    active_joints: list[str],
    active_idxv: list[int],
) -> dict[str, Any]:
    ident = config.get("identification", {})
    sample_time = float(config.get("data", {}).get("sample_time", 0.02))
    return {
        "active_joints": active_joints,
        "act_idxv": active_idxv,
        "is_joint_torques": True,
        "is_external_wrench": False,
        "force_torque": None,
        "has_friction": bool(ident.get("has_friction", False)),
        "has_actuator_inertia": bool(ident.get("has_actuator_inertia", False)),
        "has_joint_offset": bool(ident.get("has_joint_offset", False)),
        "ts": sample_time,
    }


def _active_velocity_indices(model: Any, active_joints: list[str]) -> list[int]:
    indices: list[int] = []
    for joint_name in active_joints:
        joint_id = model.getJointId(joint_name)
        if joint_id == 0 or joint_id >= model.njoints:
            raise ValueError(f"Joint {joint_name!r} was not found in the URDF model")
        indices.append(int(model.joints[joint_id].idx_v))
    return indices


def _standard_parameter_dict(model: Any) -> dict[str, float]:
    params: dict[str, float] = {}
    for joint_index, joint_name in enumerate(model.names[1:], start=1):
        dyn = np.asarray(model.inertias[joint_index].toDynamicParameters(), dtype=float).reshape(10)
        for key, value in zip(INERTIAL_KEYS, dyn):
            params[f"{key}_{joint_name}"] = float(value)
    return params


def _select_active_joint_blocks(W: np.ndarray, active_idxv: list[int], samples: int) -> np.ndarray:
    row_indices = []
    for idxv in active_idxv:
        row_indices.extend(range(idxv * samples, (idxv + 1) * samples))
    return W[np.asarray(row_indices, dtype=int), :]


def _stack_active_torques(torques: np.ndarray, n_active: int) -> np.ndarray:
    if torques.shape[1] < n_active:
        raise ValueError(f"Torque CSV has {torques.shape[1]} columns, expected at least {n_active}")
    return np.concatenate([torques[:, idx] for idx in range(n_active)]).reshape(-1)


def _remove_zero_columns(
    W: np.ndarray,
    tau: np.ndarray,
    params_std: Mapping[str, float],
    *,
    zero_tolerance: float,
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    param_items = list(params_std.items())
    if W.shape[1] > len(param_items):
        raise ValueError(
            f"Regressor has {W.shape[1]} columns but only {len(param_items)} standard parameters. "
            "Disable extra terms or extend parameter naming before identifying friction/inertia offsets."
        )
    norms = np.sum(W * W, axis=0)
    keep = [idx for idx, norm in enumerate(norms) if norm >= zero_tolerance]
    if not keep:
        raise ValueError("All regressor columns were eliminated; check CSV excitation and units")
    reduced_params = [param_items[idx][0] for idx in keep]
    return W[:, keep], tau, reduced_params


def _decimate_stacked_rows(
    W: np.ndarray,
    tau: np.ndarray,
    *,
    n_active: int,
    factor: int,
) -> tuple[np.ndarray, np.ndarray]:
    if factor <= 1:
        return W, tau
    samples = tau.shape[0] // n_active
    indices = []
    for joint in range(n_active):
        start = joint * samples
        indices.extend(range(start, start + samples, factor))
    idx = np.asarray(indices, dtype=int)
    return W[idx, :], tau[idx]


def _correlation(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 2 or np.std(a) == 0.0 or np.std(b) == 0.0:
        return 0.0
    return float(np.corrcoef(a.reshape(-1), b.reshape(-1))[0, 1])


def _inertial_parameter_subset(parameter_dict: Mapping[str, float], joint_names: list[str]) -> dict[str, dict[str, float]]:
    subset: dict[str, dict[str, float]] = {}
    for joint_name in joint_names:
        values = {}
        for key in INERTIAL_KEYS:
            name = f"{key}_{joint_name}"
            if name in parameter_dict:
                values[key] = float(parameter_dict[name])
        if values:
            subset[joint_name] = values
    return subset


if __name__ == "__main__":
    main()
