"""Calibrated relative XR wrist motion mapping for AMMR."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation


def calibrated_translation_scale(
    operator_arm_length: float,
    robot_teleop_reach: float,
    override: float | None = None,
) -> float:
    """Convert normalized operator reach into a robot translation gain.

    Moving the controller by ``operator_arm_length`` meters maps to
    ``robot_teleop_reach`` meters at the robot wrist.  ``override`` preserves
    the legacy direct gain for repeatable experiments.
    """
    for name, value in (
        ("operator_arm_length", operator_arm_length),
        ("robot_teleop_reach", robot_teleop_reach),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if override is not None:
        if not math.isfinite(override) or override <= 0.0:
            raise ValueError("translation_scale override must be finite and positive")
        return float(override)
    return float(robot_teleop_reach / operator_arm_length)


def validate_pose(pose: Sequence[Sequence[float]], name: str = "pose") -> np.ndarray:
    result = np.asarray(pose, dtype=np.float64)
    if result.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 matrix")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} contains NaN or infinity")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{name} is not homogeneous")
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError(f"{name} rotation is not orthonormal")
    if not math.isclose(np.linalg.det(rotation), 1.0, abs_tol=2e-3):
        raise ValueError(f"{name} rotation determinant is not +1")
    return result.copy()


@dataclass(frozen=True)
class PoseMapperConfig:
    translation_scale: float = 1.0
    rotation_scale: float = 1.0
    max_translation: float = 0.45
    axis_map: tuple[tuple[float, float, float], ...] = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    # These are disabled by default to preserve the mapper's original API.
    # The XR simulator enables them because tracking noise is visible to IK.
    translation_deadband: float = 0.0
    rotation_deadband: float = 0.0
    filter_alpha: float = 1.0
    filter_mode: str = "ema"
    one_euro_min_cutoff: float = 0.8
    one_euro_beta: float = 5.0
    one_euro_d_cutoff: float = 0.9

    def __post_init__(self) -> None:
        if not math.isfinite(self.translation_scale) or self.translation_scale <= 0.0:
            raise ValueError("translation_scale must be finite and positive")
        if not math.isfinite(self.rotation_scale) or self.rotation_scale <= 0.0:
            raise ValueError("rotation_scale must be finite and positive")
        if not math.isfinite(self.max_translation) or self.max_translation <= 0.0:
            raise ValueError("max_translation must be finite and positive")
        if not math.isfinite(self.translation_deadband) or self.translation_deadband < 0.0:
            raise ValueError("translation_deadband must be finite and non-negative")
        if not math.isfinite(self.rotation_deadband) or self.rotation_deadband < 0.0:
            raise ValueError("rotation_deadband must be finite and non-negative")
        if not math.isfinite(self.filter_alpha) or not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if self.filter_mode not in {"ema", "one_euro"}:
            raise ValueError("filter_mode must be 'ema' or 'one_euro'")
        for name in ("one_euro_min_cutoff", "one_euro_d_cutoff"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(self.one_euro_beta) or self.one_euro_beta < 0.0:
            raise ValueError("one_euro_beta must be finite and non-negative")
        axis_map = np.asarray(self.axis_map, dtype=np.float64)
        if axis_map.shape != (3, 3):
            raise ValueError("axis_map must be 3x3")
        if not np.allclose(axis_map.T @ axis_map, np.eye(3), atol=1e-6):
            raise ValueError("axis_map must be orthonormal")
        if not math.isclose(np.linalg.det(axis_map), 1.0, abs_tol=1e-6):
            raise ValueError("axis_map must be a proper rotation")


class _OneEuroScalar:
    def __init__(self, timestamp: float, value: float, min_cutoff: float, beta: float, d_cutoff: float):
        self.timestamp = timestamp
        self.value = value
        self.derivative = 0.0
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff

    @staticmethod
    def _alpha(dt: float, cutoff: float) -> float:
        rate = 2.0 * math.pi * cutoff * dt
        return rate / (rate + 1.0)

    def update(self, timestamp: float, value: float) -> float:
        dt = timestamp - self.timestamp
        if dt <= 0.0:
            return self.value
        derivative = (value - self.value) / dt
        derivative_alpha = self._alpha(dt, self.d_cutoff)
        self.derivative = derivative_alpha * derivative + (1.0 - derivative_alpha) * self.derivative
        value_alpha = self._alpha(dt, self.min_cutoff + self.beta * abs(self.derivative))
        self.value = value_alpha * value + (1.0 - value_alpha) * self.value
        self.timestamp = timestamp
        return self.value


class _OneEuroPoseFilter:
    def __init__(self, timestamp: float, pose: np.ndarray, config: PoseMapperConfig):
        self.config = config
        quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
        self._previous_quaternion = quaternion.copy()
        self._position = [
            _OneEuroScalar(timestamp, float(value), config.one_euro_min_cutoff, config.one_euro_beta, config.one_euro_d_cutoff)
            for value in pose[:3, 3]
        ]
        self._quaternion = [
            _OneEuroScalar(timestamp, float(value), config.one_euro_min_cutoff, config.one_euro_beta, config.one_euro_d_cutoff)
            for value in quaternion
        ]

    def update(self, timestamp: float, pose: np.ndarray) -> np.ndarray:
        quaternion = Rotation.from_matrix(pose[:3, :3]).as_quat()
        if float(np.dot(quaternion, self._previous_quaternion)) < 0.0:
            quaternion = -quaternion
        result = np.eye(4, dtype=np.float64)
        result[:3, 3] = [
            filter_.update(timestamp, float(value))
            for filter_, value in zip(self._position, pose[:3, 3])
        ]
        filtered_quaternion = np.asarray(
            [filter_.update(timestamp, float(value)) for filter_, value in zip(self._quaternion, quaternion)],
            dtype=np.float64,
        )
        filtered_quaternion /= np.linalg.norm(filtered_quaternion)
        result[:3, :3] = Rotation.from_quat(filtered_quaternion).as_matrix()
        self._previous_quaternion = filtered_quaternion
        return result


class RelativeWristMapper:
    """Map XR deltas onto robot end effectors without absolute offsets."""

    def __init__(self, config: PoseMapperConfig | None = None):
        self.config = config or PoseMapperConfig()
        self._xr_reference: tuple[np.ndarray, np.ndarray] | None = None
        self._robot_reference: tuple[np.ndarray, np.ndarray] | None = None
        self._filtered_targets: tuple[np.ndarray, np.ndarray] | None = None
        self._last_raw_targets: tuple[np.ndarray, np.ndarray] | None = None
        self._one_euro_filters: tuple[_OneEuroPoseFilter, _OneEuroPoseFilter] | None = None

    @property
    def calibrated(self) -> bool:
        return self._xr_reference is not None and self._robot_reference is not None

    def clear(self) -> None:
        self._xr_reference = None
        self._robot_reference = None
        self._filtered_targets = None
        self._last_raw_targets = None
        self._one_euro_filters = None

    @property
    def last_raw_targets(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Most recent task targets before the mapper's output filter."""
        if self._last_raw_targets is None:
            return None
        return tuple(target.copy() for target in self._last_raw_targets)

    def reset_filter(self) -> None:
        """Reset only the target filter while keeping the calibration."""
        self._filtered_targets = None
        self._one_euro_filters = None

    def calibrate(
        self,
        left_xr: Sequence[Sequence[float]],
        right_xr: Sequence[Sequence[float]],
        left_robot: Sequence[Sequence[float]],
        right_robot: Sequence[Sequence[float]],
    ) -> None:
        self._xr_reference = (
            validate_pose(left_xr, "left_xr"),
            validate_pose(right_xr, "right_xr"),
        )
        self._robot_reference = (
            validate_pose(left_robot, "left_robot"),
            validate_pose(right_robot, "right_robot"),
        )
        self._filtered_targets = None
        self._one_euro_filters = None

    def map(
        self,
        left_xr: Sequence[Sequence[float]],
        right_xr: Sequence[Sequence[float]],
        timestamp: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if not self.calibrated:
            raise RuntimeError("wrist mapper has not been calibrated")
        current = (
            validate_pose(left_xr, "left_xr"),
            validate_pose(right_xr, "right_xr"),
        )
        assert self._xr_reference is not None
        assert self._robot_reference is not None
        mapped = tuple(
            self._map_one(current_pose, xr_ref, robot_ref)
            for current_pose, xr_ref, robot_ref in zip(
                current, self._xr_reference, self._robot_reference
            )
        )
        self._last_raw_targets = tuple(target.copy() for target in mapped)
        now = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(now):
            raise ValueError("timestamp must be finite")

        # Filter each wrist independently. Motion of one controller must not
        # alter the held target of the other arm.
        if self._filtered_targets is None:
            self._filtered_targets = tuple(target.copy() for target in mapped)
            if self.config.filter_mode == "one_euro":
                self._one_euro_filters = tuple(
                    _OneEuroPoseFilter(now, target, self.config) for target in mapped
                )
        else:
            deadbanded = tuple(
                self._apply_deadband(target, previous)
                for target, previous in zip(mapped, self._filtered_targets)
            )
            if self.config.filter_mode == "ema":
                self._filtered_targets = tuple(
                    self._ema_filter_target(target, previous)
                    for target, previous in zip(deadbanded, self._filtered_targets)
                )
            else:
                assert self._one_euro_filters is not None
                self._filtered_targets = tuple(
                    filter_.update(now, target)
                    for filter_, target in zip(self._one_euro_filters, deadbanded)
                )
        return tuple(target.copy() for target in self._filtered_targets)

    def _apply_deadband(self, target: np.ndarray, previous: np.ndarray) -> np.ndarray:
        filtered = previous.copy()

        translation_delta = target[:3, 3] - previous[:3, 3]
        translation_distance = float(np.linalg.norm(translation_delta))
        if translation_distance > self.config.translation_deadband:
            filtered[:3, 3] = target[:3, 3]

        previous_rotation = Rotation.from_matrix(previous[:3, :3])
        target_rotation = Rotation.from_matrix(target[:3, :3])
        rotation_delta = (target_rotation * previous_rotation.inv()).as_rotvec()
        rotation_distance = float(np.linalg.norm(rotation_delta))
        if rotation_distance > self.config.rotation_deadband:
            filtered[:3, :3] = target[:3, :3]

        return filtered

    def _ema_filter_target(self, target: np.ndarray, previous: np.ndarray) -> np.ndarray:
        filtered = previous.copy()
        filtered[:3, 3] = previous[:3, 3] + self.config.filter_alpha * (
            target[:3, 3] - previous[:3, 3]
        )
        previous_rotation = Rotation.from_matrix(previous[:3, :3])
        target_rotation = Rotation.from_matrix(target[:3, :3])
        rotation_delta = (target_rotation * previous_rotation.inv()).as_rotvec()
        filtered[:3, :3] = (
            Rotation.from_rotvec(self.config.filter_alpha * rotation_delta)
            * previous_rotation
        ).as_matrix()

        return filtered

    def _map_one(
        self, current: np.ndarray, xr_ref: np.ndarray, robot_ref: np.ndarray
    ) -> np.ndarray:
        axis_map = np.asarray(self.config.axis_map, dtype=np.float64)
        translation = (
            axis_map @ (current[:3, 3] - xr_ref[:3, 3])
        ) * self.config.translation_scale
        norm = float(np.linalg.norm(translation))
        if norm > self.config.max_translation:
            translation *= self.config.max_translation / norm

        xr_rotation_delta = current[:3, :3] @ xr_ref[:3, :3].T
        robot_rotation_delta = axis_map @ xr_rotation_delta @ axis_map.T
        if self.config.rotation_scale != 1.0:
            rotation_vector = Rotation.from_matrix(robot_rotation_delta).as_rotvec()
            robot_rotation_delta = Rotation.from_rotvec(
                rotation_vector * self.config.rotation_scale
            ).as_matrix()

        target = np.eye(4, dtype=np.float64)
        target[:3, :3] = robot_rotation_delta @ robot_ref[:3, :3]
        target[:3, 3] = robot_ref[:3, 3] + translation
        return target


def fixed_xr_neutral_poses(input_mode: str) -> tuple[np.ndarray, np.ndarray]:
    """Return TeleVuer's fixed neutral wrist poses for absolute mapping.

    The values mirror the current TeleVuerWrapper controller/hand coordinate
    conversion: the XR head translation is removed and the fixed waist
    offset is applied. They are intentionally constants, not sampled at B
    press time, so absolute mode does not become per-session incremental
    calibration.
    """
    if input_mode == "controller":
        left_rotation = np.eye(3, dtype=np.float64)
        right_rotation = np.eye(3, dtype=np.float64)
        left_translation = (0.30, 0.15, -0.12)
        right_translation = (0.30, -0.15, -0.12)
    elif input_mode == "hand":
        # T_TO_UNITREE_HUMANOID_LEFT/RIGHT_ARM in TeleVuerWrapper.
        left_rotation = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
        )
        right_rotation = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
        )
        left_translation = (0.25, 0.15, -0.12)
        right_translation = (0.25, -0.15, -0.12)
    else:
        raise ValueError(f"unsupported XR input mode: {input_mode}")

    def make_pose(
        rotation: np.ndarray, translation: tuple[float, float, float]
    ) -> np.ndarray:
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = rotation
        pose[:3, 3] = translation
        return pose

    return (
        make_pose(left_rotation, left_translation),
        make_pose(right_rotation, right_translation),
    )


class FixedAbsoluteWristMapper:
    """Map absolute XR wrist poses through fixed AMMR neutral references.

    This uses the same task-space conversion as ``RelativeWristMapper`` but
    its references are fixed at startup from constants and the AMMR zero FK.
    No reference is recalibrated when the operator presses B.
    """

    def __init__(
        self,
        xr_reference: tuple[
            Sequence[Sequence[float]], Sequence[Sequence[float]]
        ],
        robot_reference: tuple[
            Sequence[Sequence[float]], Sequence[Sequence[float]]
        ],
        config: PoseMapperConfig | None = None,
    ):
        self._mapper = RelativeWristMapper(config)
        self._mapper.calibrate(
            xr_reference[0],
            xr_reference[1],
            robot_reference[0],
            robot_reference[1],
        )

    @property
    def calibrated(self) -> bool:
        return self._mapper.calibrated

    @property
    def last_raw_targets(self) -> tuple[np.ndarray, np.ndarray] | None:
        return self._mapper.last_raw_targets

    def clear(self) -> None:
        """Reset filtering without discarding fixed absolute references."""
        self._mapper.reset_filter()

    def map(
        self,
        left_xr: Sequence[Sequence[float]],
        right_xr: Sequence[Sequence[float]],
        timestamp: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._mapper.map(left_xr, right_xr, timestamp)
