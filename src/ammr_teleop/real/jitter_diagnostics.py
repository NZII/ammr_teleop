"""Optional CSV telemetry for locating real-robot teleoperation jitter."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def _pose_columns(prefix: str) -> list[str]:
    return [f"{prefix}_m{row}{column}" for row in range(4) for column in range(4)]


class JitterCsvLogger:
    """Write one low-overhead control-chain sample per XR control cycle."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ["monotonic_s", "enabled", "reset_active", "ik_success", "ik_solve_ms"]
            + _pose_columns("xr_left")
            + _pose_columns("xr_right")
            + _pose_columns("target_raw_left")
            + _pose_columns("target_raw_right")
            + _pose_columns("target_filtered_left")
            + _pose_columns("target_filtered_right")
            + [f"ik_seed_q_{index}" for index in range(14)]
            + [f"ik_q_{index}" for index in range(14)]
            + [f"command_q_{index}" for index in range(14)]
            + [f"command_velocity_q_{index}" for index in range(14)]
            + [f"command_acceleration_q_{index}" for index in range(14)]
            + [f"feedback_q_{index}" for index in range(14)]
        )
        self._rows_since_flush = 0

    @property
    def metadata_path(self) -> Path:
        return Path(f"{self.path}.meta.json")

    def write_metadata(self, metadata: Mapping[str, Any]) -> Path:
        """Write resolved run configuration next to the diagnostic CSV."""
        with self.metadata_path.open("w", encoding="utf-8") as file:
            json.dump(metadata, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.write("\n")
        return self.metadata_path

    @staticmethod
    def _pose_values(pose: np.ndarray | None) -> list[float | str]:
        if pose is None:
            return [""] * 16
        matrix = np.asarray(pose, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError("diagnostic pose must be 4x4")
        return matrix.reshape(-1).tolist()

    @staticmethod
    def _vector_values(values: Sequence[float] | None) -> list[float | str]:
        if values is None:
            return [""] * 14
        vector = np.asarray(values, dtype=np.float64)
        if vector.shape != (14,):
            raise ValueError("diagnostic joint vector must have 14 values")
        return vector.tolist()

    def write(
        self,
        *,
        timestamp: float,
        enabled: bool,
        reset_active: bool,
        ik_success: bool | None,
        ik_solve_ms: float | None,
        xr_left: np.ndarray,
        xr_right: np.ndarray,
        target_raw: tuple[np.ndarray, np.ndarray] | None,
        target_filtered: tuple[np.ndarray, np.ndarray] | None,
        ik_seed: Sequence[float] | None,
        ik_solution: Sequence[float] | None,
        command: Sequence[float] | None,
        command_velocity: Sequence[float] | None,
        command_acceleration: Sequence[float] | None,
        feedback: Sequence[float] | None,
    ) -> None:
        raw_left, raw_right = target_raw or (None, None)
        filtered_left, filtered_right = target_filtered or (None, None)
        self._writer.writerow(
            [
                float(timestamp),
                int(enabled),
                int(reset_active),
                "" if ik_success is None else int(ik_success),
                "" if ik_solve_ms is None else float(ik_solve_ms),
            ]
            + self._pose_values(xr_left)
            + self._pose_values(xr_right)
            + self._pose_values(raw_left)
            + self._pose_values(raw_right)
            + self._pose_values(filtered_left)
            + self._pose_values(filtered_right)
            + self._vector_values(ik_seed)
            + self._vector_values(ik_solution)
            + self._vector_values(command)
            + self._vector_values(command_velocity)
            + self._vector_values(command_acceleration)
            + self._vector_values(feedback)
        )
        self._rows_since_flush += 1
        if self._rows_since_flush >= 30:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()
