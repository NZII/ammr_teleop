"""Stable ROS/data contract for the 21 controllable AMMR joints."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math

import numpy as np

from ammr_teleop.common.trajectory import jerk_limited_step


ROBOT_JOINTS = (
    "chassis_x_joint",
    "chassis_y_joint",
    "chassis_yaw_joint",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_gripper_joint1",
    "left_gripper_joint2",
    "right_gripper_joint1",
    "right_gripper_joint2",
    "lifter_joint_1",
    "lifter_joint_2",
)

CHASSIS_SLICE = slice(0, 3)
LEFT_ARM_SLICE = slice(3, 9)
RIGHT_ARM_SLICE = slice(9, 15)
ARM_SLICE = slice(3, 15)
LEFT_GRIPPER_SLICE = slice(15, 17)
RIGHT_GRIPPER_SLICE = slice(17, 19)
LIFTER_SLICE = slice(19, 21)

LEFT_ARM_JOINTS = ROBOT_JOINTS[LEFT_ARM_SLICE]
RIGHT_ARM_JOINTS = ROBOT_JOINTS[RIGHT_ARM_SLICE]
ARM_JOINTS = ROBOT_JOINTS[ARM_SLICE]

# These limits are copied from the AMMR c2436 URDF. Keeping the control
# contract local means the XR process does not have to parse robot assets at
# runtime, while tests below guard the ordering and values.
POSITION_LOWER = np.array(
    [
        -10.0,
        -10.0,
        -math.pi,
        -2.6179938,
        0.0,
        -2.9670597,
        -1.7453292,
        -1.2217304,
        -2.0943951,
        -2.6179938,
        0.0,
        -2.9670597,
        -1.7453292,
        -1.2217304,
        -2.0943951,
        0.0,
        -0.05,
        0.0,
        -0.05,
        0.0,
        0.0,
    ],
    dtype=np.float64,
)
POSITION_UPPER = np.array(
    [
        10.0,
        10.0,
        math.pi,
        2.6179938,
        math.pi,
        0.0,
        1.7453292,
        1.2217304,
        2.0943951,
        2.6179938,
        math.pi,
        0.0,
        1.7453292,
        1.2217304,
        2.0943951,
        0.05,
        0.0,
        0.05,
        0.0,
        0.39,
        0.39,
    ],
    dtype=np.float64,
)
VELOCITY_LIMITS = np.array(
    [1.0, 1.0, 1.0] + [5.0] * 12 + [3.0] * 4 + [0.2] * 2,
    dtype=np.float64,
)

FLAT_RECORD_JOINTS = (
    ROBOT_JOINTS[LEFT_ARM_SLICE]
    + ROBOT_JOINTS[RIGHT_ARM_SLICE]
    + ROBOT_JOINTS[LEFT_GRIPPER_SLICE]
    + ROBOT_JOINTS[RIGHT_GRIPPER_SLICE]
    + ROBOT_JOINTS[CHASSIS_SLICE]
    + ROBOT_JOINTS[LIFTER_SLICE]
)
FLAT_RECORD_INDEXES = tuple(ROBOT_JOINTS.index(name) for name in FLAT_RECORD_JOINTS)


def ordered_positions(
    names: Sequence[str], positions: Sequence[float]
) -> np.ndarray:
    """Validate and reorder a named joint vector into the AMMR contract."""
    if len(names) != len(positions):
        raise ValueError("joint names and positions must have equal lengths")
    if len(set(names)) != len(names):
        raise ValueError("joint names contain duplicates")
    by_name = dict(zip(names, positions))
    missing = [name for name in ROBOT_JOINTS if name not in by_name]
    if missing:
        raise ValueError(f"joint state is missing AMMR joints: {missing}")
    result = np.asarray([by_name[name] for name in ROBOT_JOINTS], dtype=np.float64)
    if not np.all(np.isfinite(result)):
        raise ValueError("joint positions contain NaN or infinity")
    return result


def named_positions(positions: Sequence[float]) -> dict[str, float]:
    values = validate_vector(positions)
    return dict(zip(ROBOT_JOINTS, values.tolist()))


def validate_vector(positions: Sequence[float]) -> np.ndarray:
    result = np.asarray(positions, dtype=np.float64)
    if result.shape != (len(ROBOT_JOINTS),):
        raise ValueError(f"expected {len(ROBOT_JOINTS)} joint values, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("joint vector contains NaN or infinity")
    return result


def clamp_positions(positions: Sequence[float]) -> np.ndarray:
    return np.clip(validate_vector(positions), POSITION_LOWER, POSITION_UPPER)


def clamp_sim_positions(positions: Sequence[float]) -> np.ndarray:
    """Clamp non-arm joints while preserving equivalent USD arm angles."""
    result = validate_vector(positions).copy()
    indexes = np.r_[0:3, 15:21]
    result[indexes] = np.clip(
        result[indexes], POSITION_LOWER[indexes], POSITION_UPPER[indexes]
    )
    return result


def rate_limit_positions(
    desired: Sequence[float],
    previous: Sequence[float],
    dt: float,
    velocity_scale: float = 1.0,
    enforce_position_limits: bool = True,
) -> np.ndarray:
    """Apply URDF position and velocity limits to one command step."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if not math.isfinite(velocity_scale) or velocity_scale <= 0.0:
        raise ValueError("velocity_scale must be finite and positive")
    desired_array = (
        clamp_positions(desired)
        if enforce_position_limits
        else validate_vector(desired)
    )
    # Do not clamp the starting state. Isaac/USD can report a revolute joint
    # using an equivalent angle outside the URDF interval. Clamping it in one
    # step would create exactly the startup jump this limiter is meant to
    # prevent. Instead, move from the actual value toward the bounded target.
    previous_array = validate_vector(previous)
    max_delta = VELOCITY_LIMITS * velocity_scale * dt
    delta = np.clip(desired_array - previous_array, -max_delta, max_delta)
    result = previous_array + delta
    if not np.all(np.isfinite(result)):
        raise ValueError("rate-limited joint vector is not finite")
    return result


def jerk_limit_positions(
    desired: Sequence[float],
    previous: Sequence[float],
    previous_velocity: Sequence[float],
    previous_acceleration: Sequence[float],
    dt: float,
    velocity_scale: float,
    arm_acceleration: float,
    gripper_acceleration: float,
    arm_jerk: float,
    gripper_jerk: float,
    enforce_position_limits: bool = True,
    arm_velocity: float | None = None,
    gripper_velocity: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Limit simulated arm/gripper velocity, acceleration, and jerk.

    Chassis and lifter joints retain the existing velocity-only behavior so
    this arm-jitter optimization does not change their controller mapping.
    """
    scalar_limits = (
        dt,
        velocity_scale,
        arm_acceleration,
        gripper_acceleration,
        arm_jerk,
        gripper_jerk,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in scalar_limits):
        raise ValueError("dt and trajectory limits must be finite and positive")
    for name, value in (
        ("arm_velocity", arm_velocity),
        ("gripper_velocity", gripper_velocity),
    ):
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{name} must be finite and positive")

    target = (
        clamp_positions(desired)
        if enforce_position_limits
        else validate_vector(desired)
    )
    prior = validate_vector(previous)
    velocity = validate_vector(previous_velocity)
    acceleration = validate_vector(previous_acceleration)
    velocity_limits = VELOCITY_LIMITS * velocity_scale
    if arm_velocity is not None:
        velocity_limits[ARM_SLICE] = arm_velocity
    if gripper_velocity is not None:
        velocity_limits[15:19] = gripper_velocity
    error = target - prior

    # Preserve the established behavior for chassis and lifter joints.
    next_velocity = np.clip(error / dt, -velocity_limits, velocity_limits)
    next_acceleration = np.zeros_like(next_velocity)
    step = next_velocity * dt
    controlled = np.r_[3:19]
    acceleration_limits = np.r_[
        np.full(12, arm_acceleration, dtype=np.float64),
        np.full(4, gripper_acceleration, dtype=np.float64),
    ]
    jerk_limits = np.r_[
        np.full(12, arm_jerk, dtype=np.float64),
        np.full(4, gripper_jerk, dtype=np.float64),
    ]
    controlled_position, controlled_velocity, controlled_acceleration = (
        jerk_limited_step(
            target[controlled],
            prior[controlled],
            velocity[controlled],
            acceleration[controlled],
            dt,
            velocity_limits[controlled],
            acceleration_limits,
            jerk_limits,
        )
    )
    step[controlled] = controlled_position - prior[controlled]
    next_velocity[controlled] = controlled_velocity
    next_acceleration[controlled] = controlled_acceleration

    # Chassis and lifter remain velocity-only and may land exactly on their
    # targets without carrying acceleration state.
    uncontrolled = np.r_[0:3, 19:21]
    uncontrolled_overshoot = np.abs(step[uncontrolled]) > np.abs(error[uncontrolled])
    step[uncontrolled[uncontrolled_overshoot]] = error[
        uncontrolled[uncontrolled_overshoot]
    ]
    next_velocity[uncontrolled] = step[uncontrolled] / dt
    result = prior + step
    if not np.all(np.isfinite(result)):
        raise ValueError("acceleration-limited joint vector is not finite")
    return result, next_velocity, next_acceleration


def gripper_pair(open_fraction: float) -> tuple[float, float]:
    """Map 0=closed, 1=open to the two mirrored AMMR finger joints."""
    value = float(np.clip(open_fraction, 0.0, 1.0))
    return 0.05 * value, -0.05 * value


def grouped_record_data(
    positions: Sequence[float], velocities: Sequence[float] | None = None
) -> dict[str, dict[str, list[float]]]:
    qpos = validate_vector(positions)
    qvel = None if velocities is None else validate_vector(velocities)

    def group(indexes: Sequence[int]) -> dict[str, list[float]]:
        return {
            "qpos": qpos[list(indexes)].tolist(),
            "qvel": [] if qvel is None else qvel[list(indexes)].tolist(),
            "torque": [],
        }

    return {
        "left_arm": group(range(3, 9)),
        "right_arm": group(range(9, 15)),
        "left_ee": group(range(15, 17)),
        "right_ee": group(range(17, 19)),
        "body": group((0, 1, 2, 19, 20)),
    }


def flatten_record_mapping(
    grouped: Mapping[str, Mapping[str, Sequence[float]]],
) -> np.ndarray:
    values: list[float] = []
    for key in ("left_arm", "right_arm", "left_ee", "right_ee", "body"):
        try:
            values.extend(float(value) for value in grouped[key]["qpos"])
        except KeyError as exc:
            raise ValueError(f"record is missing {key}.qpos") from exc
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (len(ROBOT_JOINTS),) or not np.all(np.isfinite(result)):
        raise ValueError("record must contain 21 finite qpos values")
    # The recorded layout is arms, end effectors, then body. Convert back to
    # Isaac's internal command contract; it is not a public ROS topic format.
    command = np.empty(len(ROBOT_JOINTS), dtype=np.float64)
    command[list(FLAT_RECORD_INDEXES)] = result
    return command
