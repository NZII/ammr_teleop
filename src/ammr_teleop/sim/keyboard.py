#!/usr/bin/env python3
"""Keyboard teleoperation publisher for the AMMR Isaac Sim bridge."""

from __future__ import annotations

import argparse
import math
import os
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from sshkeyboard import listen_keyboard, stop_listening
from std_msgs.msg import Bool

from ammr_teleop.common.contract import (
    ARM_JOINTS as CONTRACT_ARM_JOINTS,
    RIGHT_ARM_JOINTS as CONTRACT_RIGHT_ARM_JOINTS,
    ROBOT_JOINTS as CONTRACT_ROBOT_JOINTS,
)

ROBOT_JOINTS = list(CONTRACT_ROBOT_JOINTS)
ARM_JOINTS = list(CONTRACT_ARM_JOINTS)
RIGHT_ARM_JOINTS = list(CONTRACT_RIGHT_ARM_JOINTS)


class RightArmIKController:
    """Keep both wrist poses fixed while applying XYZ increments to the right wrist."""

    def __init__(self, initial_joint_positions: list[float]):
        import pinocchio as pin
        from ammr_teleop.common.robot_ik import ArmIK

        self.pin = pin
        self.ik = ArmIK()
        self.q = np.zeros(self.ik.reduced_robot.model.nq)
        initial_by_name = dict(zip(ROBOT_JOINTS, initial_joint_positions))
        for joint_name in ARM_JOINTS:
            joint_id = self.ik.reduced_robot.model.getJointId(joint_name)
            q_index = self.ik.reduced_robot.model.joints[joint_id].idx_q
            self.q[q_index] = initial_by_name[joint_name]

        data = self.ik.reduced_robot.model.createData()
        pin.forwardKinematics(self.ik.reduced_robot.model, data, self.q)
        pin.updateFramePlacements(self.ik.reduced_robot.model, data)
        self.left_target = data.oMf[self.ik.L_hand_id].homogeneous.copy()
        self.right_target = data.oMf[self.ik.R_hand_id].homogeneous.copy()
        xyz = self.right_target[:3, 3]
        print(
            "[IK] Right wrist initialized at "
            f"x={xyz[0]:.5f}, y={xyz[1]:.5f}, z={xyz[2]:.5f}"
        )

    def apply_delta(self, target: list[float], delta_xyz: np.ndarray) -> bool:
        candidate = self.right_target.copy()
        candidate[:3, 3] += delta_xyz
        solution, _ = self.ik.solve_ik(
            left_wrist=self.left_target,
            right_wrist=candidate,
            current_lr_arm_motor_q=self.q,
        )
        try:
            solve_succeeded = bool(self.ik.opti.stats().get("success", False))
        except (RuntimeError, TypeError):
            solve_succeeded = False
        if not solve_succeeded:
            # ArmIK catches solver exceptions internally and returns the input
            # joint vector. Treat that fallback as a rejected command so the
            # Cartesian target does not keep moving beyond the reachable pose.
            return False
        if solution is None:
            return False
        solution = np.asarray(solution, dtype=float)
        if solution.shape != self.q.shape or not np.all(np.isfinite(solution)):
            return False

        self.q = solution
        self.right_target = candidate
        for target_name in RIGHT_ARM_JOINTS:
            joint_id = self.ik.reduced_robot.model.getJointId(target_name)
            q_index = self.ik.reduced_robot.model.joints[joint_id].idx_q
            target[ROBOT_JOINTS.index(target_name)] = float(solution[q_index])
        return True


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class AmmrKeyboardNode(Node):
    """Maintain a full joint target while keyboard input changes the chassis."""

    MOVEMENT_KEYS = frozenset(
        {"up", "down", "left", "right", "[", "]", "w", "s", "a", "d", "h", "l", "j", "k"}
    )

    def __init__(
        self,
        frequency: float,
        linear_speed: float,
        yaw_speed: float,
        arm_speed: float,
        gripper_speed: float,
    ):
        super().__init__("ammr_keyboard_teleop")
        self.frequency = frequency
        self.linear_speed = linear_speed
        self.yaw_speed = yaw_speed
        self.arm_speed = arm_speed
        self.gripper_speed = gripper_speed
        self.command_publisher = self.create_publisher(
            JointState, "/ammr/joint_command", reliable_qos()
        )
        self.record_publisher = self.create_publisher(
            Bool, "/ammr/record_enable", reliable_qos()
        )
        self.create_subscription(
            JointState, "/ammr/joint_states", self._on_joint_state, reliable_qos()
        )
        self.create_subscription(
            Bool, "/ammr/record_status", self._on_record_status, reliable_qos()
        )

        self.lock = threading.RLock()
        self.running = threading.Event()
        self.running.set()
        self.initialized = threading.Event()
        self.pressed_keys: set[str] = set()
        self.latest_state: list[float] | None = None
        self.target: list[float] | None = None
        self.stop_requested = False
        self.record_enabled = False
        self.last_record_press = 0.0
        self.arm_ik: RightArmIKController | None = None
        self.arm_motion_blocked = False

    def initialize_arm_ik(self) -> None:
        with self.lock:
            if self.target is None:
                raise RuntimeError("Cannot initialize IK before receiving robot state")
            initial = self.target.copy()
        self.arm_ik = RightArmIKController(initial)

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.get_logger().warning("Ignoring malformed /ammr/joint_states message")
            return
        positions = dict(zip(msg.name, msg.position))
        if any(name not in positions for name in ROBOT_JOINTS):
            self.get_logger().warning("Joint state does not contain all 21 AMMR joints")
            return
        ordered = [float(positions[name]) for name in ROBOT_JOINTS]
        if not all(math.isfinite(value) for value in ordered):
            self.get_logger().warning("Ignoring non-finite joint state")
            return
        with self.lock:
            self.latest_state = ordered
            if self.target is None:
                self.target = ordered.copy()
                self.initialized.set()
                self.get_logger().info("Initialized command target from simulator state")

    def _on_record_status(self, msg: Bool) -> None:
        with self.lock:
            changed = self.record_enabled != bool(msg.data)
            self.record_enabled = bool(msg.data)
        if changed:
            state = "RECORDING" if msg.data else "IDLE"
            self.get_logger().info(f"Recorder status: {state}")

    def on_press(self, key: str) -> None:
        key = key.lower()
        now = time.monotonic()
        with self.lock:
            if key in self.MOVEMENT_KEYS:
                self.pressed_keys.add(key)
            elif key == "x":
                self.pressed_keys.clear()
                self.stop_requested = True
            elif key == "r" and now - self.last_record_press >= 0.5:
                self.last_record_press = now
                desired = not self.record_enabled
                self.record_publisher.publish(Bool(data=desired))
                self.get_logger().info(
                    "Requested recording START" if desired else "Requested recording STOP"
                )
            elif key == "esc":
                self.running.clear()

    def on_release(self, key: str) -> None:
        with self.lock:
            self.pressed_keys.discard(key.lower())

    def update_and_publish(self, dt: float) -> None:
        with self.lock:
            if self.target is None:
                return
            if self.stop_requested:
                if self.latest_state is not None:
                    self.target[0:3] = self.latest_state[0:3]
                self.stop_requested = False

            forward = int("up" in self.pressed_keys) - int("down" in self.pressed_keys)
            lateral = int("left" in self.pressed_keys) - int("right" in self.pressed_keys)
            yaw_direction = int("[" in self.pressed_keys) - int("]" in self.pressed_keys)
            current_yaw = (
                self.latest_state[2] if self.latest_state is not None else self.target[2]
            )
            local_x = forward * self.linear_speed * dt
            local_y = lateral * self.linear_speed * dt
            self.target[0] += local_x * math.cos(current_yaw) - local_y * math.sin(current_yaw)
            self.target[1] += local_x * math.sin(current_yaw) + local_y * math.cos(current_yaw)
            self.target[2] += yaw_direction * self.yaw_speed * dt

            arm_x = int("w" in self.pressed_keys) - int("s" in self.pressed_keys)
            arm_y = int("a" in self.pressed_keys) - int("d" in self.pressed_keys)
            arm_z = int("h" in self.pressed_keys) - int("l" in self.pressed_keys)
            arm_motion_requested = bool(arm_x or arm_y or arm_z)
            if not arm_motion_requested:
                self.arm_motion_blocked = False
            elif self.arm_ik is not None and not self.arm_motion_blocked:
                delta = np.array([arm_x, arm_y, arm_z], dtype=float)
                delta *= self.arm_speed * dt
                if not self.arm_ik.apply_delta(self.target, delta):
                    self.arm_motion_blocked = True
                    self.get_logger().warning(
                        "Right-arm IK rejected the increment; release the arm key before retrying"
                    )

            gripper_direction = int("k" in self.pressed_keys) - int("j" in self.pressed_keys)
            if gripper_direction:
                gripper_delta = gripper_direction * self.gripper_speed * dt
                self.target[17] = min(0.05, max(0.0, self.target[17] + gripper_delta))
                self.target[18] = min(0.0, max(-0.05, self.target[18] - gripper_delta))
            target = self.target.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ROBOT_JOINTS
        msg.position = target
        self.command_publisher.publish(msg)

    def request_record_stop(self) -> None:
        with self.lock:
            recording = self.record_enabled
        if recording:
            self.record_publisher.publish(Bool(data=False))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frequency", type=float, default=30.0)
    parser.add_argument("--linear-speed", type=float, default=0.30, help="Planar joint units per second")
    parser.add_argument("--yaw-speed", type=float, default=0.60, help="Yaw radians per second")
    parser.add_argument("--arm-speed", type=float, default=0.10, help="Right wrist metres per second")
    parser.add_argument("--gripper-speed", type=float, default=0.10, help="Gripper joint units per second")
    parser.add_argument("--state-timeout", type=float, default=30.0, help="Seconds to wait for initial simulator state")
    parser.add_argument(
        "--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0"))
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if any(
        value <= 0
        for value in (
            args.frequency,
            args.linear_speed,
            args.yaw_speed,
            args.arm_speed,
            args.gripper_speed,
        )
    ):
        raise ValueError("frequency and speeds must be positive")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    rclpy.init()
    node = AmmrKeyboardNode(
        args.frequency,
        args.linear_speed,
        args.yaw_speed,
        args.arm_speed,
        args.gripper_speed,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_executor() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()

    keyboard_thread = threading.Thread(
        target=listen_keyboard,
        kwargs={
            "on_press": node.on_press,
            "on_release": node.on_release,
            "until": "esc",
            "sequential": True,
            "delay_second_char": 0.20,
            "delay_other_chars": 0.03,
        },
        daemon=True,
    )

    def stop(_signum=None, _frame=None) -> None:
        node.running.clear()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        node.get_logger().info("Waiting for /ammr/joint_states before accepting commands...")
        deadline = time.monotonic() + args.state_timeout
        while node.running.is_set() and not node.initialized.wait(timeout=0.1):
            if time.monotonic() >= deadline:
                break
        if not node.initialized.is_set():
            node.get_logger().error("Timed out waiting for the simulator joint state")
            return 2
        node.initialize_arm_ik()

        print("\nAMMR keyboard control")
        print("  Arrow keys: chassis translation    [ / ]: chassis yaw")
        print("  W/S: right wrist X    A/D: right wrist Y    H/L: right wrist Z")
        print("  J: close right gripper    K: open right gripper")
        print("  X: hold current chassis pose    R: record toggle    Esc: quit\n")
        keyboard_thread.start()
        period = 1.0 / args.frequency
        previous = time.monotonic()
        while node.running.is_set() and rclpy.ok():
            started = time.monotonic()
            dt = min(max(started - previous, 0.0), 0.1)
            previous = started
            node.update_and_publish(dt)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        node.request_record_stop()
        node.running.clear()
        try:
            stop_listening()
        except Exception:
            pass
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
