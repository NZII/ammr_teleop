import casadi                                                                       

import numpy as np
import pinocchio as pin                             
import math
from pinocchio import casadi as cpin 
import numpy as np
from scipy.spatial.transform import Rotation as R     
import os
from pathlib import Path
import shutil
import sys
import tempfile
from zipfile import ZipFile

class WeightedMovingFilter:
    def __init__(self, weights, data_size = 14):
        self._window_size = len(weights)
        self._weights = np.array(weights)
        assert np.isclose(np.sum(self._weights), 1.0), "[WeightedMovingFilter] the sum of weights list must be 1.0!"
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1]

        data_array = np.array(self._data_queue)
        temp_filtered_data = np.zeros(self._data_size)
        for i in range(self._data_size):
            temp_filtered_data[i] = np.convolve(data_array[:, i], self._weights, mode='valid')[-1]
        
        return temp_filtered_data

    def add_data(self, new_data):
        assert len(new_data) == self._data_size

        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return  # skip duplicate data
        
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)

        self._data_queue.append(new_data)
        self._filtered_data = self._apply_filter()

    @property
    def filtered_data(self):
        return self._filtered_data


def _resolve_robot_description():
    from .paths import configured_path
    urdf = Path(configured_path("kinematics_urdf"))
    if not urdf.is_file():
        raise FileNotFoundError(f"Kinematics URDF missing: {urdf}")
    return urdf, [item for item in os.environ.get("AMMR_URDF_PACKAGE_DIRS", "").split(os.pathsep) if item]


class ArmIK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        urdf_path, package_dirs = _resolve_robot_description()
        self.urdf_path = urdf_path
        self.robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path), package_dirs)
        self._rename_ambiguous_fixed_joint_frames(self.robot.model)
        self.mixed_jointsToLockIDs = [
                                        "chassis_x_joint", "chassis_y_joint", "chassis_yaw_joint",
                                        "lifter_joint_1", "lifter_joint_2",
                                        "fl_steering_joint", "fr_steering_joint", "rl_steering_joint", "rr_steering_joint",
                                        "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint",
                                        "left_gripper_joint1", "left_gripper_joint2",
                                        "right_gripper_joint1", "right_gripper_joint2",
                                        "top_camera_stand_joint",
                                    ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )
        self._add_ee_frame('L_ee', 'left_gripper_base')
        self._add_ee_frame('R_ee', 'right_gripper_base')
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        # Pinocchio's log3 has an undefined CasADi derivative at the identity
        # rotation. XR calibration intentionally starts with target == current
        # wrist pose, so log3(I) makes IPOPT report NaN in grad_f. Use the
        # smooth Frobenius residual of the relative rotation instead. Its
        # small-angle norm matches the angle residual up to first order and it
        # remains finite at both identity and 180-degree rotations.
        def smooth_rotation_error(current_rotation, target_rotation):
            relative_rotation = current_rotation @ target_rotation.T
            return (1.0 / math.sqrt(2.0)) * casadi.reshape(
                relative_rotation - casadi.DM.eye(3), 9, 1
            )

        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    smooth_rotation_error(
                        self.cdata.oMf[self.L_hand_id].rotation,
                        self.cTf_l[:3, :3],
                    ),
                    smooth_rotation_error(
                        self.cdata.oMf[self.R_hand_id].rotation,
                        self.cTf_r[:3, :3],
                    ),
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        # Match the stable part of Unitree's runtime configuration. A looser
        # acceptable tolerance and warm start are preferable for 30 Hz XR
        # tracking, where the target changes every cycle and exact convergence
        # is unnecessary.
        opts = {
            "expand": True,
            "detect_simple_bounds": True,
            "calc_lam_p": False,
            "print_time": False,
            "ipopt.sb": "yes",
            "ipopt.print_level": 0,
            "ipopt.max_iter": 30,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 5e-4,
            "ipopt.acceptable_iter": 5,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.derivative_test": "none",
            "ipopt.jacobian_approximation": "exact",
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1]), self.reduced_robot.model.nq
        )
        self.last_failure_reason = None
        self.vis = None

    @staticmethod
    def _rename_ambiguous_fixed_joint_frames(model):
        """Avoid Pinocchio 3.8 ambiguity when a fixed joint frame and body share a name."""
        body_names = {
            frame.name for frame in model.frames if frame.type == pin.FrameType.BODY
        }
        for frame in model.frames:
            if frame.type == pin.FrameType.FIXED_JOINT and frame.name in body_names:
                frame.name = f"{frame.name}__fixed_joint_frame"

    def _add_ee_frame(self, ee_name, parent_body_name):
        parent_frame_id = self.reduced_robot.model.getFrameId(
            parent_body_name, pin.FrameType.BODY
        )
        parent_frame = self.reduced_robot.model.frames[parent_frame_id]
        self.reduced_robot.model.addFrame(
            pin.Frame(
                ee_name,
                parent_frame.parentJoint,
                parent_frame_id,
                pin.SE3(np.eye(3), np.array([0.05, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

    def _bounded_initial_q(self, q):
        """Return a finite interior IPOPT seed without changing the output q."""
        result = np.asarray(q, dtype=np.float64).reshape(-1)
        expected = self.reduced_robot.model.nq
        if result.shape != (expected,) or not np.all(np.isfinite(result)):
            raise ValueError(f"IK initial q must contain {expected} finite values")
        lower = np.asarray(self.reduced_robot.model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(self.reduced_robot.model.upperPositionLimit, dtype=np.float64)
        if (
            lower.shape != result.shape
            or upper.shape != result.shape
            or not np.all(np.isfinite(lower))
            or not np.all(np.isfinite(upper))
            or np.any(lower >= upper)
        ):
            raise ValueError("IK joint limits must be finite and strictly ordered")
        margin = np.minimum(1e-6, 0.5 * (upper - lower))
        return np.clip(result, lower + margin, upper - margin)

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        """
        输入:
            left_wrist: 左腕部的目标位置和姿态(4x4齐次变换矩阵)。
            right_wrist: 右腕部的目标位置和姿态(4x4齐次变换矩阵)。
            current_lr_arm_motor_q: 当前左右臂关节的角度(可选, 默认为None)。
            current_lr_arm_motor_dq: 当前左右臂关节的速度(可选, 默认为None)。
        返回: 
            sol_q: 机器人关节角度。
            sol_tauff: 对应的关节力矩。
        """
        if current_lr_arm_motor_q is None:
            requested_q = np.asarray(self.init_data, dtype=np.float64).copy()
        else:
            requested_q = np.asarray(current_lr_arm_motor_q, dtype=np.float64).reshape(-1).copy()
        seed_q = self._bounded_initial_q(requested_q)
        self.opti.set_initial(self.var_q, seed_q)

        self.opti.set_value(self.param_tf_l, np.asarray(left_wrist, dtype=np.float64))
        self.opti.set_value(self.param_tf_r, np.asarray(right_wrist, dtype=np.float64))
        self.opti.set_value(self.var_q_last, seed_q) # for smooth

        try:
            sol = self.opti.solve()
            stats = self.opti.stats()
            if not bool(stats.get("success", False)):
                raise RuntimeError(f"IPOPT did not converge: {stats.get('return_status', 'unknown')}")

            sol_q = np.asarray(self.opti.value(self.var_q), dtype=np.float64).reshape(-1)
            if sol_q.shape != seed_q.shape or not np.all(np.isfinite(sol_q)):
                raise RuntimeError("IPOPT returned a non-finite joint solution")
            self.smooth_filter.add_data(sol_q)
            sol_q = np.asarray(self.smooth_filter.filtered_data, dtype=np.float64).copy()
            if not np.all(np.isfinite(sol_q)):
                raise RuntimeError("IK smoothing produced a non-finite joint solution")

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - seed_q) * 0.0

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))
            self.init_data = sol_q.copy()
            self.last_failure_reason = None

            return sol_q, sol_tauff
        
        except Exception as e:
            self.last_failure_reason = str(e)
            # Never consume opti.debug.value() here: it can contain an
            # infeasible or NaN iterate and poison the next warm start. The
            # caller already has a valid measured state to hold.
            fallback_q = requested_q.copy()
            if not np.all(np.isfinite(fallback_q)):
                fallback_q = seed_q.copy()
            return fallback_q, np.zeros(self.reduced_robot.model.nv)
        
    def api(self, left_pose, right_pose):
        rotation_L = R.from_euler('xyz', [0., 0, -math.pi/2])
        rotation_R = R.from_euler('xyz', [0., math.pi/4 * 3, math.pi/2])
        quaternion_L = rotation_L.as_quat()
        quaternion_R = rotation_R.as_quat()
        L_tf_target = pin.SE3(pin.Quaternion(quaternion_L), np.array(left_pose))
        R_tf_target = pin.SE3(pin.Quaternion(quaternion_R), np.array(right_pose))
        a, _ = self.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)
        return list(a)
    
