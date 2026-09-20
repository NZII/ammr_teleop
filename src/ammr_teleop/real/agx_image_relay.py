#!/usr/bin/env python3
"""AGX-side latest-frame JPEG relay for the optional XR image stream.

The camera drivers and the recorder remain local to the AGX.  This node is
started only when the control-center image-stream button is enabled.  It
subscribes to the three local raw RGB topics and publishes
``sensor_msgs/CompressedImage`` on the conventional ``/compressed`` topics.
Each camera has a one-frame pending slot, so a slow encoder never creates an
unbounded queue or delays the robot-control processes.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def compressed_topic(topic: str) -> str:
    topic = str(topic).rstrip("/")
    return topic if topic.endswith("/compressed") else f"{topic}/compressed"


@dataclass(frozen=True)
class PendingFrame:
    image: np.ndarray
    stamp_sec: int
    stamp_nanosec: int
    frame_id: str


def _raw_to_bgr(msg: Image) -> np.ndarray | None:
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    if width <= 0 or height <= 0:
        return None
    encoding = str(msg.encoding).lower()
    channels = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }.get(encoding)
    if channels is None:
        return None
    row_bytes = width * channels
    if step < row_bytes:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected_size = step * height
    if raw.size < expected_size:
        return None
    rows = raw[:expected_size].reshape(height, step)
    image = rows[:, :row_bytes].reshape(height, width, channels)
    if encoding == "rgb8":
        image = image[:, :, ::-1]
    elif encoding == "rgba8":
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif encoding == "mono8":
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(image)


class AmmrAgxImageRelay(Node):
    def __init__(
        self,
        *,
        topics: dict[str, str],
        jpeg_quality: int,
        max_fps: float,
    ) -> None:
        super().__init__("ammr_image_relay")
        self._stop = threading.Event()
        self._condition = threading.Condition()
        self._pending: dict[str, PendingFrame | None] = {
            name: None for name in topics
        }
        self._received = {name: 0 for name in topics}
        self._published = {name: 0 for name in topics}
        self._dropped = {name: 0 for name in topics}
        self._jpeg_quality = int(jpeg_quality)
        self._period = 1.0 / float(max_fps)
        self._last_publish = {name: 0.0 for name in topics}
        self._publishers: dict[str, object] = {}
        self._workers: list[threading.Thread] = []
        callback_group = ReentrantCallbackGroup()
        qos = sensor_qos()
        for name, topic in topics.items():
            output = compressed_topic(topic)
            self._publishers[name] = self.create_publisher(
                CompressedImage, output, qos
            )
            self.create_subscription(
                Image,
                topic,
                lambda msg, selected=name: self._on_image(selected, msg),
                qos,
                callback_group=callback_group,
            )
            worker = threading.Thread(
                target=self._encode_loop,
                args=(name,),
                daemon=True,
                name=f"ammr-jpeg-{name}",
            )
            worker.start()
            self._workers.append(worker)
        self._stats_timer = self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            "JPEG image relay ready: "
            + ", ".join(
                f"{name}={topic} -> {compressed_topic(topic)}"
                for name, topic in topics.items()
            )
            + f" quality={self._jpeg_quality} max_fps={max_fps:.1f}"
        )

    def _on_image(self, name: str, msg: Image) -> None:
        image = _raw_to_bgr(msg)
        if image is None:
            self.get_logger().warning(
                f"Ignoring unsupported or malformed {name} image "
                f"encoding={msg.encoding!r}"
            )
            return
        frame = PendingFrame(
            image=image,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            frame_id=str(msg.header.frame_id),
        )
        with self._condition:
            self._received[name] += 1
            if self._pending[name] is not None:
                self._dropped[name] += 1
            self._pending[name] = frame
            self._condition.notify_all()

    def _encode_loop(self, name: str) -> None:
        publisher = self._publishers[name]
        while not self._stop.is_set():
            with self._condition:
                while self._pending[name] is None and not self._stop.is_set():
                    self._condition.wait(timeout=0.5)
                if self._stop.is_set():
                    return
                # A callback can wake the condition before the rate period
                # has elapsed. Re-check the deadline after every wake-up so
                # the configured max_fps remains a real upper bound.
                while True:
                    wait_for = self._period - (
                        time.monotonic() - self._last_publish[name]
                    )
                    if wait_for <= 0.0:
                        break
                    self._condition.wait(timeout=wait_for)
                    if self._stop.is_set():
                        return
                frame = self._pending[name]
                self._pending[name] = None
            if frame is None:
                continue
            success, encoded = cv2.imencode(
                ".jpg",
                frame.image,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not success:
                self.get_logger().warning(f"JPEG encoding failed for {name}")
                continue
            message = CompressedImage()
            message.header.stamp.sec = frame.stamp_sec
            message.header.stamp.nanosec = frame.stamp_nanosec
            message.header.frame_id = frame.frame_id
            message.format = "jpeg"
            message.data = encoded.tobytes()
            publisher.publish(message)
            self._last_publish[name] = time.monotonic()
            with self._condition:
                self._published[name] += 1

    def _log_stats(self) -> None:
        with self._condition:
            stats = ", ".join(
                f"{name}: in={self._received[name]} out={self._published[name]} "
                f"drop={self._dropped[name]}"
                for name in self._pending
            )
        self.get_logger().info(f"JPEG image relay stats: {stats}")

    def destroy_node(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        for worker in self._workers:
            worker.join(timeout=1.0)
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--head-topic", default="/head/color/image_raw")
    parser.add_argument("--left-wrist-topic", default="/left/color/image_raw")
    parser.add_argument("--right-wrist-topic", default="/right/color/image_raw")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--max-fps", type=float, default=30.0)
    args = parser.parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")
    if not np.isfinite(args.max_fps) or args.max_fps <= 0.0:
        parser.error("--max-fps must be finite and positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = AmmrAgxImageRelay(
        topics={
            "head": args.head_topic,
            "left_wrist": args.left_wrist_topic,
            "right_wrist": args.right_wrist_topic,
        },
        jpeg_quality=args.jpeg_quality,
        max_fps=args.max_fps,
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
