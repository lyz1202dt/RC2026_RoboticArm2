from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class MeasurementData:
    time: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    torques: np.ndarray
    acceleration_source: str


def load_measurement_csv(
    path: str | Path,
    dof: int,
    max_samples: int = 0,
    acceleration_source: str = "computed",
    smoothing_window: int = 11,
    smoothing_polyorder: int = 3,
) -> MeasurementData:
    csv_path = Path(path)
    with csv_path.open("r", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} has no CSV header")

        required = ["time"]
        for prefix in ("pos", "vel", "torque"):
            required.extend(f"{prefix}_{idx}" for idx in range(dof))
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"{csv_path} is missing required columns: {missing}")
        has_recorded_acc = all(f"acc_{idx}" in reader.fieldnames for idx in range(dof))

        rows = []
        for row in reader:
            rows.append(row)
            if max_samples > 0 and len(rows) >= max_samples:
                break

    if not rows:
        raise ValueError(f"{csv_path} contains no samples")

    def column(name: str) -> np.ndarray:
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    def block(prefix: str) -> np.ndarray:
        return np.column_stack([column(f"{prefix}_{idx}") for idx in range(dof)])

    time = column("time")
    positions = block("pos")
    velocities = block("vel")
    torques = block("torque")
    accelerations, source = _build_accelerations(
        time,
        velocities,
        recorded_accelerations=block("acc") if has_recorded_acc else None,
        acceleration_source=acceleration_source,
        smoothing_window=smoothing_window,
        smoothing_polyorder=smoothing_polyorder,
    )

    return MeasurementData(
        time=time,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        torques=torques,
        acceleration_source=source,
    )


def _build_accelerations(
    time: np.ndarray,
    velocities: np.ndarray,
    *,
    recorded_accelerations: np.ndarray | None,
    acceleration_source: str,
    smoothing_window: int,
    smoothing_polyorder: int,
) -> tuple[np.ndarray, str]:
    source = acceleration_source.lower()
    if source == "recorded":
        if recorded_accelerations is None:
            raise ValueError("CSV does not contain acc_* columns but acceleration_source is 'recorded'")
        return recorded_accelerations, "recorded"
    if source != "computed":
        raise ValueError("acceleration_source must be 'computed' or 'recorded'")

    if time.size < 2:
        raise ValueError("At least two samples are required to compute acceleration")
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("CSV time column must be strictly increasing")

    smooth_vel = _smooth_signal(velocities, smoothing_window=smoothing_window, smoothing_polyorder=smoothing_polyorder)
    return np.gradient(smooth_vel, time, axis=0, edge_order=1), "computed_from_velocity"


def _smooth_signal(values: np.ndarray, *, smoothing_window: int, smoothing_polyorder: int) -> np.ndarray:
    window = int(smoothing_window)
    if window < 3 or values.shape[0] < 3:
        return values
    if window % 2 == 0:
        window += 1
    window = min(window, values.shape[0] if values.shape[0] % 2 == 1 else values.shape[0] - 1)
    if window <= smoothing_polyorder:
        return values

    try:
        from scipy.signal import savgol_filter

        return savgol_filter(values, window_length=window, polyorder=int(smoothing_polyorder), axis=0, mode="interp")
    except Exception:
        kernel = np.ones(window, dtype=float) / float(window)
        smoothed = np.empty_like(values)
        for joint in range(values.shape[1]):
            smoothed[:, joint] = np.convolve(values[:, joint], kernel, mode="same")
        return smoothed
