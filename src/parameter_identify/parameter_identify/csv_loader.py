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


def load_measurement_csv(path: str | Path, dof: int, max_samples: int = 0) -> MeasurementData:
    csv_path = Path(path)
    with csv_path.open("r", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} has no CSV header")

        required = ["time"]
        for prefix in ("pos", "vel", "acc", "torque"):
            required.extend(f"{prefix}_{idx}" for idx in range(dof))
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"{csv_path} is missing required columns: {missing}")

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

    return MeasurementData(
        time=column("time"),
        positions=block("pos"),
        velocities=block("vel"),
        accelerations=block("acc"),
        torques=block("torque"),
    )
