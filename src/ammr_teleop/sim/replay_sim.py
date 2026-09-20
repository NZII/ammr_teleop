#!/usr/bin/env python3
"""Replay an AMMR JSON or LeRobot episode through the ROS 2 simulator bridge."""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time

import numpy as np

from ammr_teleop.common.contract import (
    ROBOT_JOINTS,
    VELOCITY_LIMITS,
    ordered_positions,
    rate_limit_positions,
)
from ammr_teleop.sim.episode import Episode, episode_summary, read_episode


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-path", required=True)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument(
        "--strict-limits",
        action="store_true",
        help="Reject actions outside URDF limits instead of reporting warnings.",
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--frequency", type=float, default=None)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--end-index", type=int, default=None)
    parser.add_argument("--settle-time", type=float, default=2.0)
    parser.add_argument("--state-timeout", type=float, default=30.0)
    parser.add_argument("--velocity-scale", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--keyboard",
        action=argparse.BooleanOptionalAction,
        default=sys.stdin.isatty(),
        help="Enable Space/q replay controls (defaults to interactive terminals only).",
    )
    parser.add_argument(
        "--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0"))
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace, episode: Episode) -> None:
    for name in ("speed", "settle_time", "state_timeout", "velocity_scale"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if args.frequency is not None and (
        not math.isfinite(args.frequency) or args.frequency <= 0.0
    ):
        raise ValueError("frequency must be finite and positive")
    if args.start_index < 0:
        raise ValueError("start-index cannot be negative")
    end = len(episode.frames) if args.end_index is None else args.end_index
    if end <= args.start_index or end > len(episode.frames):
        raise ValueError("invalid replay frame range")


def run_ros_replay(args: argparse.Namespace, episode: Episode) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from sshkeyboard import listen_keyboard, stop_listening

    def qos() -> QoSProfile:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

    class ReplayNode(Node):
        def __init__(self):
            super().__init__("ammr_episode_replay_sim")
            self.publisher = self.create_publisher(
                JointState, "/ammr/joint_command", qos()
            )
            self.create_subscription(
                JointState, "/ammr/joint_states", self.on_state, qos()
            )
            self.lock = threading.RLock()
            self.initialized = threading.Event()
            self.latest_state: np.ndarray | None = None

        def on_state(self, msg: JointState) -> None:
            try:
                state = ordered_positions(msg.name, msg.position)
            except ValueError as error:
                self.get_logger().warning(f"Ignoring invalid state: {error}")
                return
            with self.lock:
                self.latest_state = state
                self.initialized.set()

        def state(self) -> np.ndarray | None:
            with self.lock:
                return None if self.latest_state is None else self.latest_state.copy()

        def publish(self, command: np.ndarray) -> None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(ROBOT_JOINTS)
            msg.position = command.tolist()
            self.publisher.publish(msg)

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = ReplayNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin, daemon=True)
    executor_thread.start()
    running = threading.Event()
    running.set()
    paused = threading.Event()

    def on_press(key: str) -> None:
        if key == "space":
            if paused.is_set():
                paused.clear()
                node.get_logger().info("Replay resumed")
            else:
                paused.set()
                node.get_logger().info("Replay paused")
        elif key == "q":
            running.clear()
            paused.clear()
            stop_listening()

    keyboard_thread = threading.Thread(
        target=listen_keyboard,
        kwargs={"on_press": on_press, "until": None, "sequential": False},
        daemon=True,
    )

    def stop(_signum=None, _frame=None) -> None:
        running.clear()
        paused.clear()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    start = args.start_index
    end = len(episode.frames) if args.end_index is None else args.end_index
    frames = episode.frames[start:end]
    fixed_dt = None if args.frequency is None else 1.0 / args.frequency
    squared_error = np.zeros(21, dtype=np.float64)
    max_error = np.zeros(21, dtype=np.float64)
    error_samples = 0

    try:
        node.get_logger().info("Waiting for /ammr/joint_states...")
        if not node.initialized.wait(args.state_timeout):
            node.get_logger().error("Timed out waiting for AMMR simulator state")
            return 2
        if args.keyboard:
            keyboard_thread.start()

        current = node.state()
        assert current is not None
        first = frames[0].action
        settle_frequency = args.frequency or episode.fps
        required_settle_time = float(
            np.max(np.abs(first - current) / (VELOCITY_LIMITS * args.velocity_scale))
        )
        settle_duration = max(args.settle_time, required_settle_time)
        settle_steps = max(1, int(math.ceil(settle_duration * settle_frequency)))
        settle_dt = settle_duration / settle_steps
        command = current.copy()
        node.get_logger().info(
            f"Moving smoothly to replay frame {frames[0].index} over {settle_duration:.3f}s"
        )
        for step in range(settle_steps):
            if not running.is_set():
                break
            desired = current + (first - current) * ((step + 1) / settle_steps)
            command = rate_limit_positions(
                desired,
                command,
                settle_dt,
                velocity_scale=args.velocity_scale,
                enforce_position_limits=False,
            )
            node.publish(command)
            time.sleep(settle_dt)

        while running.is_set():
            replay_start = time.monotonic()
            first_timestamp = frames[0].timestamp
            for offset, frame in enumerate(frames):
                if not running.is_set():
                    break
                while paused.is_set() and running.is_set():
                    node.publish(command)
                    time.sleep(0.1)
                    replay_start += 0.1
                if fixed_dt is None:
                    target_elapsed = (frame.timestamp - first_timestamp) / args.speed
                else:
                    target_elapsed = offset * fixed_dt / args.speed
                while running.is_set():
                    remaining = target_elapsed - (time.monotonic() - replay_start)
                    if remaining <= 0.0:
                        break
                    time.sleep(min(remaining, 0.01))

                if offset == 0:
                    frame_dt = fixed_dt or (1.0 / episode.fps)
                elif fixed_dt is not None:
                    frame_dt = fixed_dt / args.speed
                else:
                    frame_dt = max(
                        (frame.timestamp - frames[offset - 1].timestamp) / args.speed,
                        1e-4,
                    )
                command = rate_limit_positions(
                    frame.action,
                    command,
                    frame_dt,
                    velocity_scale=args.velocity_scale,
                    enforce_position_limits=False,
                )
                node.publish(command)
                state = node.state()
                if state is not None and frame.observation is not None:
                    error = np.abs(state - frame.observation)
                    squared_error += error * error
                    max_error = np.maximum(max_error, error)
                    error_samples += 1
            if not args.loop:
                break
            node.get_logger().info("Looping replay")
            loop_target = frames[0].action
            loop_duration = float(
                np.max(
                    np.abs(loop_target - command)
                    / (VELOCITY_LIMITS * args.velocity_scale)
                )
            )
            loop_steps = max(1, int(math.ceil(loop_duration * settle_frequency)))
            loop_dt = max(loop_duration / loop_steps, 1e-4)
            for _ in range(loop_steps):
                if not running.is_set():
                    break
                command = rate_limit_positions(
                    loop_target,
                    command,
                    loop_dt,
                    velocity_scale=args.velocity_scale,
                    enforce_position_limits=False,
                )
                node.publish(command)
                time.sleep(loop_dt)

        if error_samples:
            rmse = np.sqrt(squared_error / error_samples)
            node.get_logger().info(
                "Replay state error: "
                f"mean RMSE={rmse.mean():.6f}, max={max_error.max():.6f}"
            )
        node.get_logger().info("Replay complete; holding final target")
    finally:
        running.clear()
        if args.keyboard:
            try:
                stop_listening()
            except Exception:
                pass
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)
        if args.keyboard and keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
    return 0


def main() -> int:
    args = parse_args()
    episode = read_episode(args.data_path, strict_limits=args.strict_limits)
    validate_args(args, episode)
    print(episode_summary(episode))
    if args.validate_only:
        print("Episode validation passed.")
        return 0
    return run_ros_replay(args, episode)


if __name__ == "__main__":
    raise SystemExit(main())
