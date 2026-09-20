"""Discrete jerk-limited position tracking shared by sim and hardware."""

from __future__ import annotations

from collections.abc import Sequence
import math

import numpy as np


def jerk_limited_step(
    target: Sequence[float],
    previous: Sequence[float],
    previous_velocity: Sequence[float],
    previous_acceleration: Sequence[float],
    dt: float,
    velocity_limits: Sequence[float],
    acceleration_limits: Sequence[float],
    jerk_limits: Sequence[float],
    *,
    position_gain: float = 4.0,
    velocity_gain: float = 5.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance a bounded position trajectory by one control period.

    The velocity-bound envelope reserves enough room to ramp acceleration to
    zero at the configured jerk limit.  This avoids replacing a jerk spike
    with a hard velocity clip when a joint reaches its speed boundary.
    """
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be finite and positive")
    if (
        not math.isfinite(position_gain)
        or position_gain <= 0.0
        or not math.isfinite(velocity_gain)
        or velocity_gain <= 0.0
    ):
        raise ValueError("trajectory gains must be finite and positive")

    arrays = tuple(
        np.asarray(values, dtype=np.float64)
        for values in (
            target,
            previous,
            previous_velocity,
            previous_acceleration,
            velocity_limits,
            acceleration_limits,
            jerk_limits,
        )
    )
    shape = arrays[0].shape
    if len(shape) != 1 or any(values.shape != shape for values in arrays):
        raise ValueError("trajectory vectors must be equally sized 1-D arrays")
    if any(not np.all(np.isfinite(values)) for values in arrays):
        raise ValueError("trajectory vectors must contain only finite values")
    target_array, prior, velocity, acceleration, vmax, amax, jmax = arrays
    if np.any(vmax <= 0.0) or np.any(amax <= 0.0) or np.any(jmax <= 0.0):
        raise ValueError("trajectory limits must be positive")
    if np.any(np.abs(velocity) > vmax + 1e-9):
        raise ValueError("previous velocity exceeds its configured limit")
    if np.any(np.abs(acceleration) > amax + 1e-9):
        raise ValueError("previous acceleration exceeds its configured limit")

    error = target_array - prior
    requested_velocity = np.clip(position_gain * error, -vmax, vmax)
    requested_acceleration = np.clip(
        velocity_gain * (requested_velocity - velocity), -amax, amax
    )

    jerk_step = jmax * dt
    lower_acceleration = np.maximum(-amax, acceleration - jerk_step)
    upper_acceleration = np.minimum(amax, acceleration + jerk_step)

    # If acceleration points toward a velocity limit, it must start ramping
    # down early enough that bounded jerk can bring it to zero at that limit.
    positive_margin = np.maximum(vmax - velocity, 0.0)
    negative_margin = np.maximum(vmax + velocity, 0.0)
    safe_positive = -jerk_step + np.sqrt(
        jerk_step * jerk_step + 2.0 * jmax * positive_margin
    )
    safe_negative = -jerk_step + np.sqrt(
        jerk_step * jerk_step + 2.0 * jmax * negative_margin
    )
    lower_acceleration = np.maximum(lower_acceleration, -safe_negative)
    upper_acceleration = np.minimum(upper_acceleration, safe_positive)
    if np.any(lower_acceleration > upper_acceleration + 1e-9):
        raise ValueError("trajectory state cannot satisfy jerk and velocity limits")

    next_acceleration = np.clip(
        requested_acceleration, lower_acceleration, upper_acceleration
    )
    next_velocity = velocity + next_acceleration * dt
    step = next_velocity * dt

    # Exact arrival is allowed only when velocity, acceleration, jerk, and the
    # next-cycle velocity braking envelope all remain feasible.
    arrival_velocity = error / dt
    arrival_acceleration = (arrival_velocity - velocity) / dt
    crosses_target = (step * error > 0.0) & (np.abs(step) > np.abs(error))
    feasible_arrival = (
        (np.abs(arrival_velocity) <= vmax + 1e-12)
        & (np.abs(arrival_acceleration) <= amax + 1e-12)
        & (np.abs(arrival_acceleration - acceleration) <= jerk_step + 1e-12)
    )
    arrival_safe = (
        (
            (arrival_acceleration <= 0.0)
            | (
                arrival_velocity
                + arrival_acceleration * arrival_acceleration / (2.0 * jmax)
                <= vmax + 1e-12
            )
        )
        & (
            (arrival_acceleration >= 0.0)
            | (
                arrival_velocity
                - arrival_acceleration * arrival_acceleration / (2.0 * jmax)
                >= -vmax - 1e-12
            )
        )
    )
    land_on_target = crosses_target & feasible_arrival & arrival_safe
    step[land_on_target] = error[land_on_target]
    next_velocity[land_on_target] = arrival_velocity[land_on_target]
    next_acceleration[land_on_target] = arrival_acceleration[land_on_target]
    return prior + step, next_velocity, next_acceleration
