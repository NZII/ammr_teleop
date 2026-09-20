"""Pure safety helpers used by the real-robot control loop."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np

from ammr_teleop.common.trajectory import jerk_limited_step

from ammr_teleop.common.data_contract import (
    LEFT_ARM_SLICE,
    RIGHT_ARM_SLICE,
    clamp_command,
    validate_vector,
)


@dataclass(frozen=True)
class ChassisLimits:
    linear_x: float = 0.20
    linear_y: float = 0.15
    angular_z: float = 0.30
    deadband: float = 0.08


def apply_deadband(value: float, deadband: float) -> float:
    value = float(np.clip(value, -1.0, 1.0))
    if abs(value) <= deadband:
        return 0.0
    return math.copysign((abs(value) - deadband) / (1.0 - deadband), value)


def chassis_command(
    left_stick: Sequence[float], right_stick: Sequence[float], limits: ChassisLimits
) -> np.ndarray:
    left = np.asarray(left_stick, dtype=np.float64)
    right = np.asarray(right_stick, dtype=np.float64)
    if left.shape != (2,) or right.shape != (2,) or not np.all(np.isfinite(np.r_[left, right])):
        raise ValueError("thumbstick values must be finite 2-D vectors")
    return np.array(
        [
            -apply_deadband(left[1], limits.deadband) * limits.linear_x,
            -apply_deadband(left[0], limits.deadband) * limits.linear_y,
            -apply_deadband(right[0], limits.deadband) * limits.angular_z,
        ],
        dtype=np.float64,
    )


def rate_limit_command(
    desired: Sequence[float],
    previous: Sequence[float],
    dt: float,
    arm_velocity: float,
    gripper_velocity: float,
) -> np.ndarray:
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if arm_velocity <= 0.0 or gripper_velocity <= 0.0:
        raise ValueError("velocity limits must be positive")
    target = clamp_command(desired)
    prior = clamp_command(previous)
    limits = np.array([arm_velocity] * 6 + [gripper_velocity] + [arm_velocity] * 6 + [gripper_velocity])
    delta = np.clip(target - prior, -limits * dt, limits * dt)
    return prior + delta


def jerk_limit_command(
    desired: Sequence[float],
    previous: Sequence[float],
    previous_velocity: Sequence[float],
    previous_acceleration: Sequence[float],
    dt: float,
    arm_velocity: float,
    gripper_velocity: float,
    arm_acceleration: float,
    gripper_acceleration: float,
    arm_jerk: float,
    gripper_jerk: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply per-joint velocity, acceleration, and jerk limits."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    limits = (
        arm_velocity,
        gripper_velocity,
        arm_acceleration,
        gripper_acceleration,
        arm_jerk,
        gripper_jerk,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in limits):
        raise ValueError("velocity and acceleration limits must be finite and positive")

    target = clamp_command(desired)
    prior = clamp_command(previous)
    velocity = validate_vector(previous_velocity)
    acceleration = validate_vector(previous_acceleration)
    velocity_limits = np.array(
        [arm_velocity] * 6
        + [gripper_velocity]
        + [arm_velocity] * 6
        + [gripper_velocity],
        dtype=np.float64,
    )
    acceleration_limits = np.array(
        [arm_acceleration] * 6
        + [gripper_acceleration]
        + [arm_acceleration] * 6
        + [gripper_acceleration],
        dtype=np.float64,
    )
    jerk_limits = np.array(
        [arm_jerk] * 6
        + [gripper_jerk]
        + [arm_jerk] * 6
        + [gripper_jerk],
        dtype=np.float64,
    )
    command, next_velocity, next_acceleration = jerk_limited_step(
        target,
        prior,
        velocity,
        acceleration,
        dt,
        velocity_limits,
        acceleration_limits,
        jerk_limits,
    )
    # Jerk continuity can make the discrete trajectory cross a hard position
    # boundary while it brakes.  Hardware position limits take precedence:
    # clamp the published command and discard outward motion state on axes
    # that hit a boundary so the following cycle starts from a feasible state.
    bounded_command = clamp_command(command)
    saturated = np.abs(bounded_command - command) > 1e-12
    next_velocity[saturated] = 0.0
    next_acceleration[saturated] = 0.0
    return bounded_command, next_velocity, next_acceleration


def feedback_ik_seed(
    command: Sequence[float],
    feedback: Sequence[float],
    active_arm_sides: set[str],
    max_joint_error: float,
) -> np.ndarray:
    """Return a bounded feedback-corrected seed for the next arm IK solve.

    The physical robot can lag its last position command.  Seeding IK from
    that command alone makes the optimizer solve from a state the robot has
    not reached yet.  Use measured joints for enabled arms, but clip a large
    command-feedback gap to prevent a stale or discontinuous feedback sample
    from causing a discontinuous optimizer warm start.
    """
    if not active_arm_sides or not active_arm_sides.issubset({"left", "right"}):
        raise ValueError("active_arm_sides must be a non-empty subset of {'left', 'right'}")
    if not math.isfinite(max_joint_error) or max_joint_error <= 0.0:
        raise ValueError("max_joint_error must be finite and positive")
    result = validate_vector(command).copy()
    measured = validate_vector(feedback)
    for side, arm_slice in (("left", LEFT_ARM_SLICE), ("right", RIGHT_ARM_SLICE)):
        if side not in active_arm_sides:
            continue
        error = measured[arm_slice] - result[arm_slice]
        result[arm_slice] += np.clip(error, -max_joint_error, max_joint_error)
    return result


def feedback_is_fresh(now: float, timestamps: Sequence[float], timeout: float) -> bool:
    if timeout <= 0.0 or not math.isfinite(timeout):
        raise ValueError("timeout must be finite and positive")
    return bool(timestamps and all(ts > 0.0 and now - ts <= timeout for ts in timestamps))
