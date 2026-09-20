"""Stable 14-dimensional contract for AMMR real-robot recordings."""

from __future__ import annotations

from collections.abc import Sequence
import math

import numpy as np


CONTRACT_VERSION = "ammr_real_joint14_v1"
DRIVER_ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
LEFT_ARM_JOINTS = tuple(f"left_joint{i}" for i in range(1, 7))
RIGHT_ARM_JOINTS = tuple(f"right_joint{i}" for i in range(1, 7))
REAL_JOINTS = LEFT_ARM_JOINTS + ("left_gripper",) + RIGHT_ARM_JOINTS + (
    "right_gripper",
)

LEFT_ARM_SLICE = slice(0, 6)
LEFT_GRIPPER_INDEX = 6
RIGHT_ARM_SLICE = slice(7, 13)
RIGHT_GRIPPER_INDEX = 13

ARM_LOWER = np.array(
    [-2.6179938, 0.0, -2.9670597, -1.7453292, -1.2217304, -2.0943951],
    dtype=np.float64,
)
ARM_UPPER = np.array(
    [2.6179938, math.pi, 0.0, 1.7453292, 1.2217304, 2.0943951],
    dtype=np.float64,
)
POSITION_LOWER = np.r_[ARM_LOWER, 0.0, ARM_LOWER, 0.0]
POSITION_UPPER = np.r_[ARM_UPPER, 0.1, ARM_UPPER, 0.1]


def validate_vector(values: Sequence[float]) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (14,):
        raise ValueError(f"expected 14 values, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("real-robot vector contains NaN or infinity")
    return result


def arm_feedback(names: Sequence[str], positions: Sequence[float]) -> np.ndarray:
    """Extract six joints and the gripper width from one namespaced driver state."""
    if len(names) != len(positions):
        raise ValueError("joint names and positions must have equal lengths")
    if len(set(names)) != len(names):
        raise ValueError("joint names contain duplicates")
    by_name = dict(zip(names, positions))
    missing = [name for name in DRIVER_ARM_JOINTS if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing driver joints: {missing}")
    result = np.asarray(
        [by_name[name] for name in DRIVER_ARM_JOINTS]
        + [by_name.get("gripper", math.nan)],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(result[:6])):
        raise ValueError("arm feedback contains NaN or infinity")
    return result


def combine_feedback(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    left_array = np.asarray(left, dtype=np.float64)
    right_array = np.asarray(right, dtype=np.float64)
    if left_array.shape != (7,) or right_array.shape != (7,):
        raise ValueError("each arm feedback vector must contain 7 values")
    result = np.r_[left_array, right_array]
    return validate_vector(result)


def clamp_command(values: Sequence[float]) -> np.ndarray:
    return np.clip(validate_vector(values), POSITION_LOWER, POSITION_UPPER)


def driver_command(values: Sequence[float], side: str) -> tuple[list[str], list[float]]:
    command = validate_vector(values)
    if side == "left":
        selected = command[:7]
    elif side == "right":
        selected = command[7:]
    else:
        raise ValueError("side must be 'left' or 'right'")
    return list(DRIVER_ARM_JOINTS) + ["gripper"], selected.tolist()


def to_sim_contract(values: Sequence[float]) -> np.ndarray:
    """Embed real arm state into the existing 21-D IK contract."""
    from ammr_teleop.common.contract import (
        LEFT_ARM_SLICE as SIM_LEFT_ARM_SLICE,
        LEFT_GRIPPER_SLICE,
        RIGHT_ARM_SLICE as SIM_RIGHT_ARM_SLICE,
        RIGHT_GRIPPER_SLICE,
    )

    real = validate_vector(values)
    sim = np.zeros(21, dtype=np.float64)
    sim[SIM_LEFT_ARM_SLICE] = real[LEFT_ARM_SLICE]
    sim[SIM_RIGHT_ARM_SLICE] = real[RIGHT_ARM_SLICE]
    sim[LEFT_GRIPPER_SLICE] = [real[LEFT_GRIPPER_INDEX] / 2.0, -real[LEFT_GRIPPER_INDEX] / 2.0]
    sim[RIGHT_GRIPPER_SLICE] = [real[RIGHT_GRIPPER_INDEX] / 2.0, -real[RIGHT_GRIPPER_INDEX] / 2.0]
    return sim


def from_sim_contract(values: Sequence[float]) -> np.ndarray:
    """Project a 21-D simulated state onto the real 14-D arm/gripper contract."""
    from ammr_teleop.common.contract import (
        LEFT_ARM_SLICE as SIM_LEFT_ARM_SLICE,
        LEFT_GRIPPER_SLICE,
        RIGHT_ARM_SLICE as SIM_RIGHT_ARM_SLICE,
        RIGHT_GRIPPER_SLICE,
        validate_vector as validate_sim_vector,
    )

    sim = validate_sim_vector(values)
    left_gripper = sim[LEFT_GRIPPER_SLICE]
    right_gripper = sim[RIGHT_GRIPPER_SLICE]
    return validate_vector(
        np.r_[
            sim[SIM_LEFT_ARM_SLICE],
            left_gripper[0] - left_gripper[1],
            sim[SIM_RIGHT_ARM_SLICE],
            right_gripper[0] - right_gripper[1],
        ]
    )


def arms_from_sim_contract(values: Sequence[float], previous: Sequence[float]) -> np.ndarray:
    """Copy the solved arm joints from a 21-D IK result into the real contract."""
    from ammr_teleop.common.contract import (
        LEFT_ARM_SLICE as SIM_LEFT_ARM_SLICE,
        RIGHT_ARM_SLICE as SIM_RIGHT_ARM_SLICE,
        validate_vector as validate_sim_vector,
    )

    sim = validate_sim_vector(values)
    result = validate_vector(previous).copy()
    result[LEFT_ARM_SLICE] = sim[SIM_LEFT_ARM_SLICE]
    result[RIGHT_ARM_SLICE] = sim[SIM_RIGHT_ARM_SLICE]
    return result
