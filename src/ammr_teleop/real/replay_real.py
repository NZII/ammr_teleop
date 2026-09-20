#!/usr/bin/env python3
"""Safely replay a 14-D AMMR/Piper LeRobot episode on real or virtual hardware."""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time
from typing import Sequence

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

from ammr_teleop.common.data_contract import arm_feedback, combine_feedback, driver_command
from ammr_teleop.common.control_profiles import available_profiles, load_profile
from ammr_teleop.real.episode import (
    RealEpisodeFrame,
    episode_summary,
    interpolate_action,
    read_real_episode,
    select_frames,
)
from ammr_teleop.real.safety import feedback_is_fresh, jerk_limit_command


def reliable_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class PiperReplayNode(Node):
    def __init__(
        self,
        hardware_output: bool,
        gripper_effort: float,
        target_mode: str = "real",
    ) -> None:
        if target_mode not in {"real", "sim"}:
            raise ValueError("target_mode must be 'real' or 'sim'")
        super().__init__(
            "ammr_piper_replay_real"
            if target_mode == "real"
            else "ammr_episode_replay_sim"
        )
        self.hardware_output = bool(hardware_output)
        self.gripper_effort = float(gripper_effort)
        self.target_mode = target_mode
        self.lock = threading.RLock()
        self.left_state: np.ndarray | None = None
        self.right_state: np.ndarray | None = None
        self.left_received_at = 0.0
        self.right_received_at = 0.0
        self.left_publisher = self.create_publisher(
            JointState, "/left/control/joint_states", reliable_qos()
        )
        self.right_publisher = self.create_publisher(
            JointState, "/right/control/joint_states", reliable_qos()
        )
        self.create_subscription(
            JointState,
            "/left/feedback/joint_states",
            lambda msg: self._on_feedback("left", msg),
            reliable_qos(),
        )
        self.create_subscription(
            JointState,
            "/right/feedback/joint_states",
            lambda msg: self._on_feedback("right", msg),
            reliable_qos(),
        )
        self.gate_clients = {
            "left": self.create_client(SetBool, "/left/control_enable"),
            "right": self.create_client(SetBool, "/right/control_enable"),
        }
        self.hold_clients = {
            "left": self.create_client(Empty, "/left/emergency_stop"),
            "right": self.create_client(Empty, "/right/emergency_stop"),
        }
        target = "REAL PIPER HARDWARE" if target_mode == "real" else "ISAAC SIMULATION"
        output = "ROS output enabled" if self.hardware_output else "dry-run"
        self.get_logger().warning(f"AMMR replay target: {target} ({output})")

    def _on_feedback(self, side: str, msg: JointState) -> None:
        try:
            value = arm_feedback(msg.name, msg.position)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid {side} feedback: {error}")
            return
        if not np.isfinite(value[6]):
            self.get_logger().warning(f"Ignoring {side} feedback without gripper")
            return
        with self.lock:
            if side == "left":
                self.left_state = value
                self.left_received_at = time.monotonic()
            else:
                self.right_state = value
                self.right_received_at = time.monotonic()

    def state(self) -> tuple[np.ndarray | None, tuple[float, float]]:
        with self.lock:
            if self.left_state is None or self.right_state is None:
                return None, (self.left_received_at, self.right_received_at)
            return (
                combine_feedback(self.left_state, self.right_state),
                (self.left_received_at, self.right_received_at),
            )

    def wait_for_safety_services(self, timeout: float) -> bool:
        if not self.hardware_output:
            return True
        for client in (*self.gate_clients.values(), *self.hold_clients.values()):
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Required safety service unavailable: {client.srv_name}")
                return False
        return True

    def set_gate(self, enabled: bool, timeout: float | None = None) -> bool:
        if not self.hardware_output:
            return True
        if any(not client.service_is_ready() for client in self.gate_clients.values()):
            return False
        futures = [
            client.call_async(SetBool.Request(data=bool(enabled)))
            for client in self.gate_clients.values()
        ]
        if timeout is None:
            return True
        deadline = time.monotonic() + timeout
        for future in futures:
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                return False
            try:
                response = future.result()
            except Exception:
                return False
            if response is None or not response.success:
                return False
        return True

    def emergency_hold(self) -> None:
        if not self.hardware_output:
            return
        self.set_gate(False)
        for client in self.hold_clients.values():
            if client.service_is_ready():
                client.call_async(Empty.Request())

    def publish(self, command: np.ndarray) -> None:
        if not self.hardware_output:
            return
        for side, publisher in (
            ("left", self.left_publisher),
            ("right", self.right_publisher),
        ):
            names, positions = driver_command(command, side)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            msg.effort = [0.0] * 6 + [self.gripper_effort]
            publisher.publish(msg)


class ReplayControls:
    def __init__(self, keyboard: bool, initially_armed: bool) -> None:
        self.keyboard = keyboard
        self.armed = threading.Event()
        if initially_armed:
            self.armed.set()
        self.paused = threading.Event()
        self.stopped = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self, logger) -> None:
        if not self.keyboard:
            return
        from sshkeyboard import listen_keyboard

        def on_press(key: str) -> None:
            if key == "space":
                if not self.armed.is_set():
                    self.armed.set()
                    logger.info("Replay ARMED; starting smooth entry")
                elif self.paused.is_set():
                    self.paused.clear()
                    logger.info("Replay resumed")
                else:
                    self.paused.set()
                    logger.info("Replay paused; holding current command")
            elif key == "q":
                self.stopped.set()
                self.paused.clear()
                logger.warning("Replay stop requested")

        self.thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": on_press, "until": None, "sequential": False},
            daemon=True,
        )
        self.thread.start()

    def close(self) -> None:
        self.stopped.set()
        if self.keyboard:
            try:
                from sshkeyboard import stop_listening

                stop_listening()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    profiles = available_profiles()
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument(
        "--profile", choices=profiles, default="piper_absolute_smooth_v1"
    )
    selected, _ = probe.parse_known_args(argv)
    parameters = load_profile(selected.profile)["parameters"]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target-mode",
        choices=("real", "sim"),
        default="real",
        help="Select real Piper hardware or the Isaac virtual-hardware ROS interface.",
    )
    parser.add_argument(
        "--profile", choices=profiles, default=selected.profile
    )
    parser.add_argument(
        "--smoothing",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Apply profile velocity/acceleration/jerk limiting during episode "
            "playback (default: --no-smoothing)."
        ),
    )
    parser.add_argument("--data-path", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--clamp-limits",
        action="store_true",
        help="Clamp legacy out-of-range action values to Piper limits",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Episode time scale; 1.0 preserves the recorded timestamps.",
    )
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--end-index", type=int, default=None)
    parser.add_argument(
        "--command-frequency",
        type=float,
        default=float(parameters["command_frequency"]),
    )
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--settle-timeout", type=float, default=30.0)
    parser.add_argument("--hold-final-time", type=float, default=1.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--state-timeout", type=float, default=0.25)
    parser.add_argument("--service-timeout", type=float, default=2.0)
    parser.add_argument("--control-stall-timeout", type=float, default=0.10)
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument(
        "--arm-velocity", type=float, default=float(parameters["arm_velocity"])
    )
    parser.add_argument(
        "--arm-acceleration",
        type=float,
        default=float(parameters["arm_acceleration"]),
    )
    parser.add_argument("--arm-jerk", type=float, default=float(parameters["arm_jerk"]))
    parser.add_argument(
        "--gripper-velocity",
        type=float,
        default=float(parameters["gripper_velocity"]),
    )
    parser.add_argument(
        "--gripper-acceleration",
        type=float,
        default=float(parameters["gripper_acceleration"]),
    )
    parser.add_argument(
        "--gripper-jerk",
        type=float,
        default=float(parameters["gripper_jerk"]),
    )
    parser.add_argument("--gripper-effort", type=float, default=1.0)
    parser.add_argument("--enable-hardware-output", action="store_true")
    parser.add_argument(
        "--hardware-components",
        nargs="+",
        choices=("left_arm", "right_arm"),
        default=[],
    )
    parser.add_argument("--start-immediately", action="store_true")
    parser.add_argument(
        "--keyboard",
        action=argparse.BooleanOptionalAction,
        default=sys.stdin.isatty(),
    )
    parser.add_argument(
        "--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "101"))
    )
    args = parser.parse_args(argv)

    positive = (
        "speed",
        "command_frequency",
        "settle_time",
        "settle_timeout",
        "hold_final_time",
        "startup_timeout",
        "state_timeout",
        "service_timeout",
        "control_stall_timeout",
        "position_tolerance",
        "arm_velocity",
        "arm_acceleration",
        "arm_jerk",
        "gripper_velocity",
        "gripper_acceleration",
        "gripper_jerk",
        "gripper_effort",
    )
    for name in positive:
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.enable_hardware_output and not args.validate_only:
        if set(args.hardware_components) != {"left_arm", "right_arm"}:
            parser.error(
                f"{args.target_mode} replay output requires "
                "--hardware-components left_arm right_arm"
            )
        if not args.keyboard and not args.start_immediately:
            parser.error(
                f"non-interactive {args.target_mode} replay requires --start-immediately"
            )
    elif args.hardware_components and not args.validate_only:
        parser.error("--hardware-components requires --enable-hardware-output")
    return args


def _sleep_to_rate(next_tick: float, period: float) -> float:
    now = time.monotonic()
    if next_tick > now:
        time.sleep(next_tick - now)
        return next_tick + period
    if now - next_tick > period:
        return now + period
    return next_tick + period


def _fresh_feedback(node: PiperReplayNode, timeout: float) -> np.ndarray:
    state, timestamps = node.state()
    now = time.monotonic()
    if state is None or not feedback_is_fresh(now, timestamps, timeout):
        source = "Piper" if node.target_mode == "real" else "Isaac simulator"
        raise RuntimeError(f"{source} feedback is incomplete or stale")
    return state


def run_replay(args: argparse.Namespace, frames: Sequence[RealEpisodeFrame]) -> int:
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = PiperReplayNode(
        args.enable_hardware_output,
        args.gripper_effort,
        target_mode=args.target_mode,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin, daemon=True)
    executor_thread.start()
    controls = ReplayControls(
        args.keyboard,
        initially_armed=(not args.enable_hardware_output or args.start_immediately),
    )
    signal.signal(signal.SIGINT, lambda *_: controls.stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: controls.stopped.set())
    controls.start(node.get_logger())
    gate_enabled = False
    result = 0

    try:
        deadline = time.monotonic() + args.startup_timeout
        state = None
        while time.monotonic() < deadline and not controls.stopped.is_set():
            state, _ = node.state()
            if state is not None:
                break
            time.sleep(0.05)
        if state is None:
            source = "Piper" if args.target_mode == "real" else "Isaac simulator"
            raise RuntimeError(
                f"timed out waiting for complete {source} dual-arm/gripper feedback; "
                f"verify the target is running on ROS_DOMAIN_ID={args.domain_id}"
            )
        _fresh_feedback(node, args.state_timeout)
        if not node.wait_for_safety_services(args.service_timeout):
            source = "Piper" if args.target_mode == "real" else "simulation"
            raise RuntimeError(f"required {source} safety services are unavailable")

        first_target = frames[0].action
        initial_error = np.abs(first_target - state)
        node.get_logger().warning(
            "Replay initial difference: "
            f"max={initial_error.max():.4f}, mean={initial_error.mean():.4f}; "
            f"first_frame={frames[0].index}"
        )
        if args.enable_hardware_output and not controls.armed.is_set():
            target = "real robot" if args.target_mode == "real" else "simulator"
            node.get_logger().warning(
                f"Replay ready for {target}; press Space to ARM, q to abort"
            )
        while not controls.armed.is_set() and not controls.stopped.is_set():
            _fresh_feedback(node, args.state_timeout)
            time.sleep(0.05)
        if controls.stopped.is_set():
            return 0

        if not node.set_gate(True, args.service_timeout):
            raise RuntimeError("Piper control gates did not acknowledge replay enable")
        gate_enabled = args.enable_hardware_output
        time.sleep(0.1)
        period = 1.0 / args.command_frequency
        command = state.copy()
        velocity = np.zeros(14, dtype=np.float64)
        acceleration = np.zeros(14, dtype=np.float64)
        previous_tick = time.monotonic()
        next_tick = previous_tick + period
        settle_started = previous_tick
        node.get_logger().info("Moving smoothly from feedback to the first replay frame")
        while not controls.stopped.is_set():
            now = time.monotonic()
            dt = now - previous_tick
            previous_tick = now
            if dt > args.control_stall_timeout:
                raise RuntimeError(f"replay control loop stalled for {dt * 1000.0:.1f}ms")
            _fresh_feedback(node, args.state_timeout)
            settle_target = command.copy() if controls.paused.is_set() else first_target
            command, velocity, acceleration = jerk_limit_command(
                settle_target,
                command,
                velocity,
                acceleration,
                max(dt, 1e-4),
                args.arm_velocity,
                args.gripper_velocity,
                args.arm_acceleration,
                args.gripper_acceleration,
                args.arm_jerk,
                args.gripper_jerk,
            )
            node.publish(command)
            settled = (
                np.max(np.abs(first_target - command)) <= args.position_tolerance
                and np.max(np.abs(velocity)) <= args.position_tolerance
                and now - settle_started >= args.settle_time
            )
            if settled:
                break
            if now - settle_started > args.settle_timeout:
                raise RuntimeError("failed to settle safely at the first replay frame")
            next_tick = _sleep_to_rate(next_tick, period)
        if controls.stopped.is_set():
            return 0

        node.get_logger().info(
            f"Replay started: frames={len(frames)}, speed={args.speed:.3f}, "
            f"command_rate={args.command_frequency:.1f}Hz, "
            f"smoothing={'on' if args.smoothing else 'off'}"
        )
        duration = frames[-1].timestamp - frames[0].timestamp
        playback_elapsed = 0.0
        previous_tick = time.monotonic()
        next_tick = previous_tick + period
        squared_error = np.zeros(14, dtype=np.float64)
        max_error = np.zeros(14, dtype=np.float64)
        samples = 0
        while not controls.stopped.is_set() and playback_elapsed < duration:
            now = time.monotonic()
            dt = now - previous_tick
            previous_tick = now
            if dt > args.control_stall_timeout:
                raise RuntimeError(f"replay control loop stalled for {dt * 1000.0:.1f}ms")
            feedback = _fresh_feedback(node, args.state_timeout)
            if controls.paused.is_set():
                target = command.copy()
            else:
                playback_elapsed = min(duration, playback_elapsed + dt * args.speed)
                target = interpolate_action(frames, playback_elapsed)
            if args.smoothing:
                command, velocity, acceleration = jerk_limit_command(
                    target,
                    command,
                    velocity,
                    acceleration,
                    max(dt, 1e-4),
                    args.arm_velocity,
                    args.gripper_velocity,
                    args.arm_acceleration,
                    args.gripper_acceleration,
                    args.arm_jerk,
                    args.gripper_jerk,
                )
            else:
                command = target.copy()
                velocity.fill(0.0)
                acceleration.fill(0.0)
            node.publish(command)
            error = np.abs(command - feedback)
            squared_error += error * error
            max_error = np.maximum(max_error, error)
            samples += 1
            next_tick = _sleep_to_rate(next_tick, period)

        final_target = frames[-1].action
        final_started = time.monotonic()
        while not controls.stopped.is_set():
            now = time.monotonic()
            dt = now - previous_tick
            previous_tick = now
            if dt > args.control_stall_timeout:
                raise RuntimeError(f"replay control loop stalled for {dt * 1000.0:.1f}ms")
            _fresh_feedback(node, args.state_timeout)
            if args.smoothing:
                command, velocity, acceleration = jerk_limit_command(
                    final_target,
                    command,
                    velocity,
                    acceleration,
                    max(dt, 1e-4),
                    args.arm_velocity,
                    args.gripper_velocity,
                    args.arm_acceleration,
                    args.gripper_acceleration,
                    args.arm_jerk,
                    args.gripper_jerk,
                )
            else:
                command = final_target.copy()
                velocity.fill(0.0)
                acceleration.fill(0.0)
            node.publish(command)
            settled = (
                np.max(np.abs(final_target - command)) <= args.position_tolerance
                and np.max(np.abs(velocity)) <= args.position_tolerance
            )
            if settled and now - final_started >= args.hold_final_time:
                break
            if now - final_started > args.settle_timeout:
                raise RuntimeError("failed to settle safely at the final replay frame")
            next_tick = _sleep_to_rate(next_tick, period)

        if samples:
            rmse = np.sqrt(squared_error / samples)
            node.get_logger().info(
                f"Replay tracking: mean_rmse={rmse.mean():.5f}, "
                f"max_error={max_error.max():.5f}"
            )
        node.get_logger().info("Replay complete; entering emergency hold")
    except Exception as error:
        node.get_logger().error(f"Replay aborted: {type(error).__name__}: {error}")
        result = 3
    finally:
        if gate_enabled or args.enable_hardware_output:
            node.emergency_hold()
        controls.close()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    episode = read_real_episode(args.data_path, clamp_limits=args.clamp_limits)
    frames = select_frames(episode, args.start_index, args.end_index)
    print(episode_summary(episode))
    print(
        f"selected frames={len(frames)}, indices={frames[0].index}..{frames[-1].index}, "
        f"duration={frames[-1].timestamp - frames[0].timestamp:.3f}s"
    )
    smoothing = f"on ({args.profile})" if args.smoothing else "off"
    print(f"replay smoothing={smoothing}")
    if episode.limit_clamped_frames:
        print(
            f"WARNING: clamped limit violations in {episode.limit_clamped_frames} "
            "frame(s); original parquet was not modified."
        )
    if args.validate_only:
        target = "real-robot" if args.target_mode == "real" else "simulation"
        print(
            f"{target.capitalize()} episode validation passed; "
            "no ROS commands were published."
        )
        return 0
    return run_replay(args, frames)


if __name__ == "__main__":
    raise SystemExit(main())
