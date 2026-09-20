"""Pinocchio/CasADi adapter around the AMMR dual-arm IK implementation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

from ammr_teleop.common.contract import ARM_JOINTS, ROBOT_JOINTS, validate_vector
from ammr_teleop.common.pose_mapper import validate_pose


@dataclass(frozen=True)
class IKRegularizationConfig:
    translation_weight: float = 50.0
    rotation_weight: float = 1.0
    posture_weight: float = 0.01
    motion_weight: float = 0.20

    def __post_init__(self) -> None:
        for name in ("translation_weight", "rotation_weight"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("posture_weight", "motion_weight"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


class _PassThroughSolutionFilter:
    """ArmIK-compatible filter that returns the latest solution unchanged."""

    def __init__(self, data_size: int):
        self._data_size = int(data_size)
        self._data_queue: list[np.ndarray] = []
        self._filtered_data = np.zeros(self._data_size, dtype=np.float64)

    def add_data(self, new_data: Sequence[float]) -> None:
        value = np.asarray(new_data, dtype=np.float64)
        if value.shape != (self._data_size,):
            raise ValueError("IK solution has unexpected shape")
        self._data_queue[:] = [value.copy()]
        self._filtered_data = value.copy()

    @property
    def filtered_data(self) -> np.ndarray:
        return self._filtered_data


class DualArmKinematics:
    def __init__(
        self,
        regularization: IKRegularizationConfig | None = None,
    ):
        import pinocchio as pin
        from .robot_ik import ArmIK

        self.pin = pin
        self.ik = ArmIK()
        self.regularization = regularization or IKRegularizationConfig()
        config = self.regularization
        # Replace ArmIK's fixed scalar objective with explicit, configurable
        # all-joint regularization. smooth_cost penalizes every one of the 12
        # arm joints relative to the bounded feedback warm-start seed.
        self.ik.opti.minimize(
            config.translation_weight * self.ik.translational_cost
            + config.rotation_weight * self.ik.rotation_cost
            + config.posture_weight * self.ik.regularization_cost
            + config.motion_weight * self.ik.smooth_cost
        )
        # ArmIK installs a fixed four-frame moving-average filter internally.
        # XR teleoperation already filters task-space input and constrains the
        # output trajectory, so that extra history only adds joint-space lag.
        # Replace it for both simulation and hardware control paths.
        self.ik.smooth_filter = _PassThroughSolutionFilter(
            self.ik.reduced_robot.model.nq
        )
        self.last_failure_reason: str | None = None

    def reduced_q(self, positions: Sequence[float]) -> np.ndarray:
        state = validate_vector(positions)
        by_name = dict(zip(ROBOT_JOINTS, state))
        q = np.zeros(self.ik.reduced_robot.model.nq, dtype=np.float64)
        for name in ARM_JOINTS:
            joint_id = self.ik.reduced_robot.model.getJointId(name)
            q_index = self.ik.reduced_robot.model.joints[joint_id].idx_q
            q[q_index] = by_name[name]
        return q

    def forward(
        self, positions: Sequence[float]
    ) -> tuple[np.ndarray, np.ndarray]:
        q = self.reduced_q(positions)
        return self._forward_reduced_q(q)

    def reset_solver_history(self, positions: Sequence[float]) -> None:
        """Forget IK solutions from before a control re-calibration.

        Reset the configured solution filter and the solver warm-start seed
        to the command that will be held during calibration.
        """
        q = self.reduced_q(positions)
        smooth_filter = getattr(self.ik, "smooth_filter", None)
        if smooth_filter is not None:
            data_queue = getattr(smooth_filter, "_data_queue", None)
            if data_queue is not None:
                data_queue.clear()
            if hasattr(smooth_filter, "_filtered_data"):
                smooth_filter._filtered_data = q.copy()
        if hasattr(self.ik, "init_data"):
            self.ik.init_data = q.copy()
        self.ik.last_failure_reason = None
        self.last_failure_reason = None

    def _forward_reduced_q(
        self, q: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        data = self.ik.reduced_robot.model.createData()
        self.pin.forwardKinematics(self.ik.reduced_robot.model, data, q)
        self.pin.updateFramePlacements(self.ik.reduced_robot.model, data)
        return (
            data.oMf[self.ik.L_hand_id].homogeneous.copy(),
            data.oMf[self.ik.R_hand_id].homogeneous.copy(),
        )

    def solve(
        self,
        left_target: Sequence[Sequence[float]],
        right_target: Sequence[Sequence[float]],
        positions: Sequence[float],
    ) -> np.ndarray | None:
        state = validate_vector(positions)
        current_q = self.reduced_q(state)
        left_target = validate_pose(left_target, "left_target")
        right_target = validate_pose(right_target, "right_target")

        # Calibration maps the first XR sample exactly to the current robot
        # pose. Avoid an unnecessary solve at that fixed point, which is also
        # the singular input that exposed Pinocchio log3's NaN derivative.
        current_left, current_right = self._forward_reduced_q(current_q)
        if self._pose_is_close(left_target, current_left) and self._pose_is_close(
            right_target, current_right
        ):
            self.last_failure_reason = None
            return state.copy()

        try:
            solution, _ = self.ik.solve_ik(
                left_target,
                right_target,
                current_lr_arm_motor_q=current_q,
            )
        except Exception as error:
            self.last_failure_reason = f"{type(error).__name__}: {error}"
            return None
        try:
            succeeded = bool(self.ik.opti.stats().get("success", False))
        except (RuntimeError, TypeError):
            succeeded = False
        if not succeeded or solution is None:
            self.last_failure_reason = (
                getattr(self.ik, "last_failure_reason", None)
                or "solver did not converge"
            )
            return None
        solution = np.asarray(solution, dtype=np.float64)
        if solution.shape != current_q.shape or not np.all(np.isfinite(solution)):
            self.last_failure_reason = "solver returned a non-finite or malformed solution"
            return None

        target = state.copy()
        for name in ARM_JOINTS:
            joint_id = self.ik.reduced_robot.model.getJointId(name)
            q_index = self.ik.reduced_robot.model.joints[joint_id].idx_q
            state_index = ROBOT_JOINTS.index(name)
            value = float(solution[q_index])
            # Isaac/USD may expose a revolute coordinate with a 2π offset
            # from the URDF/Pinocchio interval. Choose the equivalent angle
            # nearest the measured state so enabling XR never commands a
            # full-turn jump.
            value += 2.0 * np.pi * round((state[state_index] - value) / (2.0 * np.pi))
            target[state_index] = value
        self.last_failure_reason = None
        return target

    @staticmethod
    def _pose_is_close(
        first: np.ndarray,
        second: np.ndarray,
        translation_tolerance: float = 1e-8,
        rotation_tolerance: float = 1e-8,
    ) -> bool:
        return bool(
            np.linalg.norm(first[:3, 3] - second[:3, 3])
            <= translation_tolerance
            and np.linalg.norm(first[:3, :3] - second[:3, :3])
            <= rotation_tolerance
        )
