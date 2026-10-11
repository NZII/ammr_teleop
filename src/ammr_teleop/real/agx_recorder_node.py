"""Standalone AGX-side recorder for AMMR real-robot XR episodes.

The node keeps image encoding and dataset finalization out of the XR/IK
process.  It intentionally uses only standard ROS 2 interfaces so it can be
commissioned before a custom recording message package exists. RGB images are
received directly from the three SDK-backed ZMQ publishers; ROS remains only
for arm state/action and recording control.
"""

from __future__ import annotations

import argparse
from ammr_teleop.common.paths import configured_path
from dataclasses import dataclass
import json
import math
from pathlib import Path
import signal
import threading
import time
from typing import Sequence

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from ammr_teleop.common.camera_zmq import CameraFrame, CameraSubscriberSet
from ammr_teleop.common.data_contract import arm_feedback, combine_feedback
from ammr_teleop.real.quality import (
    QualityThresholds,
    assess_episode_quality,
    episode_quality_metrics,
)
from ammr_teleop.real.recorder import CAMERAS, ProcessEpisodeRecorder


def sensor_data_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@dataclass(frozen=True)
class TimedVector:
    value: np.ndarray
    received_at: float


@dataclass(frozen=True)
class TimedImage:
    image: np.ndarray
    received_at: float
    source_stamp: float
    sequence: int


class AmmrAgxRecorderNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ammr_agx_recorder")
        self.args = args
        self.cache_lock = threading.RLock()
        self.recording_lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()

        self.feedback: dict[str, TimedVector | None] = {side: None for side in ("left", "right")}
        self.actions: dict[str, TimedVector | None] = {side: None for side in ("left", "right")}
        self.images: dict[str, TimedImage | None] = {name: None for name in CAMERAS}
        self.last_recorded_image_sequences: tuple[int, int, int] | None = None
        self.camera_reuse_counts = {name: 0 for name in CAMERAS}

        self.recorder = ProcessEpisodeRecorder(
            args.data_dir,
            args.recording_frequency,
            args.task,
            queue_size=args.writer_queue_size,
            startup_timeout=args.writer_startup_timeout,
            stop_timeout=args.writer_stop_timeout,
        )
        self.missing_sample_ticks = 0
        self.last_error: str | None = None
        self.last_saved_path: str | None = None
        self.last_quality: dict | None = None
        self.last_source_diagnostics: dict[str, float] = {}
        self.started_at: float | None = None

        self.status_publisher = self.create_publisher(
            String, args.status_topic, status_qos()
        )
        self.recording_service = self.create_service(
            SetBool,
            args.service_name,
            self._on_set_enabled,
            callback_group=self.callback_group,
        )

        for side, topic in (
            ("left", args.left_feedback_topic),
            ("right", args.right_feedback_topic),
        ):
            self.create_subscription(
                JointState,
                topic,
                lambda msg, selected=side: self._on_joint_state(
                    "feedback", selected, msg
                ),
                sensor_data_qos(),
                callback_group=self.callback_group,
            )
        for side, topic in (
            ("left", args.left_action_topic),
            ("right", args.right_action_topic),
        ):
            self.create_subscription(
                JointState,
                topic,
                lambda msg, selected=side: self._on_joint_state(
                    "action", selected, msg
                ),
                sensor_data_qos(),
                callback_group=self.callback_group,
            )
        self.camera_subscribers = CameraSubscriberSet(
            {
                "head": args.head_zmq_endpoint,
                "left_wrist": args.left_wrist_zmq_endpoint,
                "right_wrist": args.right_wrist_zmq_endpoint,
            },
            self._on_zmq_image,
            hwm=args.camera_zmq_hwm,
        )
        self.camera_subscribers.start()

        self.sample_timer = self.create_timer(
            1.0 / args.recording_frequency,
            self._sample,
            callback_group=self.callback_group,
        )
        self.status_timer = self.create_timer(
            1.0 / args.status_frequency,
            self._publish_status,
            callback_group=self.callback_group,
        )
        self._publish_status()
        self.get_logger().info(
            "AGX recorder ready: "
            f"service={args.service_name}, status={args.status_topic}, "
            f"fps={args.recording_frequency:.1f}, data_dir={self.recorder.root}"
        )

    def _on_joint_state(self, kind: str, side: str, msg: JointState) -> None:
        try:
            value = arm_feedback(msg.name, msg.position)
        except ValueError as error:
            self.get_logger().warning(
                f"Ignoring invalid {side} {kind} JointState: {error}"
            )
            return
        if not np.isfinite(value[6]):
            self.get_logger().warning(
                f"Ignoring {side} {kind} JointState without gripper"
            )
            return
        sample = TimedVector(value.copy(), time.monotonic())
        with self.cache_lock:
            target = self.feedback if kind == "feedback" else self.actions
            target[side] = sample

    def _on_zmq_image(self, frame: CameraFrame) -> None:
        with self.cache_lock:
            sample = TimedImage(
                frame.image,
                frame.received_at,
                frame.source_stamp,
                frame.sequence,
            )
            self.images[frame.camera_name] = sample

    def _latest_snapshot(self, sampled_at: float | None = None):
        sampled_at = time.monotonic() if sampled_at is None else sampled_at
        with self.cache_lock:
            missing = [
                *(f"{side}_feedback" for side, sample in self.feedback.items() if sample is None),
                *(f"{side}_action" for side, sample in self.actions.items() if sample is None),
                *(f"{name}_camera" for name, sample in self.images.items() if sample is None),
            ]
            if missing:
                raise ValueError("waiting for first samples: " + ", ".join(missing))
            feedback = dict(self.feedback)
            actions = dict(self.actions)
            images = {
                name: TimedImage(
                    sample.image.copy(),  # type: ignore[union-attr]
                    sample.received_at,
                    sample.source_stamp,
                    sample.sequence,
                )
                for name, sample in self.images.items()
            }
        diagnostics: dict[str, float] = {}
        for kind, samples in (("feedback", feedback), ("action", actions)):
            for side, sample in samples.items():
                assert sample is not None
                prefix = f"ammr.capture.{kind}_{side}"
                diagnostics[f"{prefix}_received_monotonic_s"] = sample.received_at
                diagnostics[f"{prefix}_age_ms"] = max(0.0, sampled_at - sample.received_at) * 1000.0
        for name, sample in images.items():
            prefix = f"ammr.capture.camera_{name}"
            diagnostics[f"{prefix}_received_monotonic_s"] = sample.received_at
            diagnostics[f"{prefix}_age_ms"] = max(0.0, sampled_at - sample.received_at) * 1000.0
            diagnostics[f"{prefix}_source_timestamp_s"] = sample.source_stamp
            diagnostics[f"{prefix}_sequence"] = float(sample.sequence)
        return feedback, actions, images, diagnostics

    def _status(self, state: str | None = None) -> dict:
        if state is None:
            state = "recording" if self.recorder.active else "idle"
            if self.last_error is not None and not self.recorder.active:
                state = "error"
        return {
            "state": state,
            "episode": self.recorder.episode_index,
            "frames": (
                self.recorder.frames_accepted
                if self.recorder.active
                else self.recorder.frames_written
            ),
            "missing_sample_ticks": self.missing_sample_ticks,
            "camera_reuse_counts": dict(self.camera_reuse_counts),
            "camera_frames_received": dict(self.camera_subscribers.frames_received),
            "camera_sequence_drops": dict(self.camera_subscribers.sequence_drops),
            "camera_errors": dict(self.camera_subscribers.errors),
            "producer_drops": self.recorder.producer_drops,
            "writer_queue_depth": self.recorder.queue_depth,
            "writer_mode": "process",
            "source_diagnostics": self.last_source_diagnostics,
            "actual_fps": (
                self.recorder.frames_accepted / (time.monotonic() - self.started_at)
                if self.recorder.active
                and self.started_at is not None
                and time.monotonic() > self.started_at
                else 0.0
            ),
            "path": self.last_saved_path,
            "quality": self.last_quality,
            "error": self.last_error,
            "fps": self.args.recording_frequency,
        }

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(self._status(), ensure_ascii=False)
        self.status_publisher.publish(message)

    @staticmethod
    def _response(response: SetBool.Response, success: bool, payload: dict):
        response.success = success
        response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _on_set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        with self.recording_lock:
            if request.data:
                if self.recorder.active:
                    return self._response(response, True, self._status())
                try:
                    self._latest_snapshot()
                except ValueError as error:
                    problem = str(error)
                    self.last_error = problem
                    self._publish_status()
                    self.get_logger().warning(f"Recording start rejected: {problem}")
                    return self._response(response, False, self._status("rejected"))
                try:
                    episode = self.recorder.start()
                except Exception as error:
                    self.last_error = str(error)
                    self._publish_status()
                    return self._response(response, False, self._status("error"))
                started_at = time.monotonic()
                self.missing_sample_ticks = 0
                self.camera_reuse_counts = {name: 0 for name in CAMERAS}
                self.last_recorded_image_sequences = None
                self.last_error = None
                self.last_saved_path = None
                self.last_quality = None
                self.last_source_diagnostics = {}
                self.started_at = started_at
                self.get_logger().info(
                    f"Recording episode_{episode:06d} at {self.args.recording_frequency:.1f} Hz"
                )
                self._publish_status()
                return self._response(response, True, self._status())

            if not self.recorder.active:
                return self._response(response, True, self._status())
            try:
                quality = self._episode_quality(time.monotonic())
                path = self.recorder.stop(quality)
                self.last_saved_path = str(path)
                self.last_quality = quality
                self.last_error = None
                self.get_logger().info(
                    f"Saved episode_{self.recorder.episode_index:06d}: {path} "
                    f"frames={self.recorder.frames_written}, "
                    f"missing_ticks={self.missing_sample_ticks}, "
                    f"producer_drops={self.recorder.producer_drops}, "
                    f"quality={'valid' if quality['valid'] else 'invalid'}"
                )
                if not quality["valid"]:
                    self.get_logger().warning(
                        "Episode quality INVALID: " + "; ".join(quality["issues"])
                    )
                self._publish_status()
                return self._response(response, True, self._status())
            except Exception as error:
                self.last_error = str(error)
                self.get_logger().error(f"Failed to save recording: {error}")
                self._publish_status()
                return self._response(response, False, self._status("error"))

    def _sample(self) -> None:
        with self.recording_lock:
            if not self.recorder.active:
                return
            now = time.monotonic()
            try:
                feedback, actions, images, diagnostics = self._latest_snapshot(now)
            except ValueError as error:
                problem = str(error)
                self.missing_sample_ticks += 1
                self.last_error = problem
                if self.missing_sample_ticks == 1 or self.missing_sample_ticks % 30 == 0:
                    self.get_logger().warning(
                        f"Recording waiting for source data (tick {self.missing_sample_ticks}): {problem}"
                    )
                return

            sequences = tuple(images[name].sequence for name in CAMERAS)
            reused = {}
            if self.last_recorded_image_sequences is None:
                reused = {name: False for name in CAMERAS}
            else:
                reused = {
                    name: current == previous
                    for name, current, previous in zip(
                        CAMERAS, sequences, self.last_recorded_image_sequences
                    )
                }
            for name, is_reused in reused.items():
                if is_reused:
                    self.camera_reuse_counts[name] += 1
                diagnostics[f"ammr.capture.camera_{name}_reused"] = float(is_reused)
            self.last_source_diagnostics = dict(diagnostics)
            try:
                assert feedback["left"] is not None and feedback["right"] is not None
                assert actions["left"] is not None and actions["right"] is not None
                state = combine_feedback(
                    feedback["left"].value, feedback["right"].value
                )
                action = combine_feedback(
                    actions["left"].value, actions["right"].value
                )
                queued = self.recorder.try_add_frame(
                    state,
                    action,
                    {name: images[name].image for name in CAMERAS},
                    diagnostics,
                )
            except Exception as error:
                self.last_error = str(error)
                self.get_logger().error(f"Recording sample failed: {error}")
                return
            if queued:
                self.last_recorded_image_sequences = sequences
            self.last_error = None

    def _episode_quality(self, now: float) -> dict:
        duration = max(
            now - self.started_at if self.started_at is not None else 0.0,
            1e-9,
        )
        frames = self.recorder.frames_accepted
        metrics = episode_quality_metrics(
            frames=frames,
            duration_s=duration,
            producer_drops=self.recorder.producer_drops,
        )
        thresholds = QualityThresholds(
            min_frames=self.args.quality_min_frames,
            min_actual_fps=self.args.quality_min_fps,
            max_producer_drops=self.args.quality_max_producer_drops,
        )
        return assess_episode_quality(metrics, thresholds)

    def stop_before_shutdown(self) -> None:
        self.camera_subscribers.close()
        with self.recording_lock:
            if not self.recorder.active:
                return
            try:
                quality = self._episode_quality(time.monotonic())
                path = self.recorder.stop(quality)
                self.last_saved_path = str(path)
                self.last_quality = quality
                self.get_logger().info(f"Saved active recording during shutdown: {path}")
            except Exception as error:
                self.get_logger().error(
                    f"Failed to save active recording during shutdown: {error}"
                )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording-frequency", type=float, default=30.0)
    parser.add_argument("--status-frequency", type=float, default=1.0)
    parser.add_argument("--writer-queue-size", type=int, default=16)
    parser.add_argument("--writer-startup-timeout", type=float, default=10.0)
    parser.add_argument("--writer-stop-timeout", type=float, default=30.0)
    parser.add_argument("--quality-min-frames", type=int, default=30)
    parser.add_argument("--quality-min-fps", type=float, default=20.0)
    parser.add_argument("--quality-max-producer-drops", type=int, default=0)
    parser.add_argument("--data-dir", default=str(Path(configured_path("data_root")) / "ammr_real_xr"))
    parser.add_argument("--task", default="Operate AMMR dual arms with XR.")
    parser.add_argument("--service-name", default="/ammr/recording/set_enabled")
    parser.add_argument("--status-topic", default="/ammr/recording/status")
    parser.add_argument("--left-feedback-topic", default="/left/feedback/joint_states")
    parser.add_argument("--right-feedback-topic", default="/right/feedback/joint_states")
    parser.add_argument("--left-action-topic", default="/left/control/joint_states")
    parser.add_argument("--right-action-topic", default="/right/control/joint_states")
    parser.add_argument("--head-zmq-endpoint", default="tcp://127.0.0.1:55555")
    parser.add_argument("--left-wrist-zmq-endpoint", default="tcp://127.0.0.1:55556")
    parser.add_argument("--right-wrist-zmq-endpoint", default="tcp://127.0.0.1:55557")
    parser.add_argument("--camera-zmq-hwm", type=int, default=2)
    args = parser.parse_args(argv)
    for name in (
        "recording_frequency",
        "status_frequency",
        "writer_startup_timeout",
        "writer_stop_timeout",
        "quality_min_fps",
    ):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.writer_queue_size <= 0:
        parser.error("--writer-queue-size must be positive")
    if args.camera_zmq_hwm <= 0:
        parser.error("--camera-zmq-hwm must be positive")
    if args.quality_min_frames <= 0:
        parser.error("--quality-min-frames must be positive")
    if args.quality_max_producer_drops < 0:
        parser.error("--quality-max-producer-drops cannot be negative")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = AmmrAgxRecorderNode(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def request_stop(_signum=None, _frame=None) -> None:
        executor.shutdown(timeout_sec=2.0)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        executor.spin()
    finally:
        node.stop_before_shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
