"""ROS 2 adapter for the AMMR real robot.

The adapter never publishes motion unless ``hardware_output`` is explicitly
enabled by the caller.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
import threading
import time

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

from ammr_teleop.common.camera_zmq import CameraFrame, CameraSubscriberSet
from ammr_teleop.common.data_contract import (
    arm_feedback,
    clamp_command,
    combine_feedback,
    driver_command,
    validate_vector,
)


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def image_qos() -> QoSProfile:
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
class ImageSample:
    image: np.ndarray
    received_at: float
    stamp: float


@dataclass(frozen=True)
class PendingCompressedImage:
    data: bytes
    stamp: float


class AmmrRealHardwareNode(Node):
    def __init__(
        self,
        *,
        hardware_output: bool,
        enabled_components: set[str],
        required_arm_sides: set[str],
        head_topic: str,
        left_wrist_topic: str,
        right_wrist_topic: str,
        head_zmq_endpoint: str = "tcp://192.168.3.99:55555",
        left_wrist_zmq_endpoint: str = "tcp://192.168.3.99:55556",
        right_wrist_zmq_endpoint: str = "tcp://192.168.3.99:55557",
        subscribe_head_camera: bool = True,
        subscribe_wrist_cameras: bool = True,
        image_transport: str = "raw",
        recorder_service: str = "/ammr/recording/set_enabled",
        recorder_status_topic: str = "/ammr/recording/status",
    ) -> None:
        target_mode = os.environ.get("AMMR_TARGET_MODE", "real").strip().lower()
        if target_mode not in {"real", "sim"}:
            raise ValueError("AMMR_TARGET_MODE must be 'real' or 'sim'")
        super().__init__(f"ammr_xr_teleop_{target_mode}")
        self.target_mode = target_mode
        self.hardware_output = bool(hardware_output)
        self.enabled_components = set(enabled_components)
        self.required_arm_sides = set(required_arm_sides)
        if not self.required_arm_sides.issubset({"left", "right"}):
            raise ValueError("required_arm_sides must be a subset of {'left', 'right'}")
        self.lock = threading.RLock()
        self.left_state: np.ndarray | None = None
        self.right_state: np.ndarray | None = None
        self.left_received_at = 0.0
        self.right_received_at = 0.0
        self.images: dict[str, ImageSample] = {}
        self.remote_recorder_status: dict = {}
        if image_transport not in {"raw", "compressed", "zmq"}:
            raise ValueError("image_transport must be 'raw', 'compressed', or 'zmq'")
        self.image_transport = image_transport
        self._cv2 = None
        if image_transport == "compressed":
            try:
                import cv2
            except ImportError as error:
                raise RuntimeError(
                    "compressed image streaming requires OpenCV (cv2)"
                ) from error
            self._cv2 = cv2
        # JPEG decoding is deliberately kept out of the ROS executor callback.
        # The control loop and arm feedback continue to run while each camera
        # worker decodes only its newest pending frame.
        self._image_stop = threading.Event()
        self._image_condition = threading.Condition()
        self._compressed_pending: dict[str, PendingCompressedImage | None] = {}
        self._image_workers: list[threading.Thread] = []
        self._zmq_cameras: CameraSubscriberSet | None = None

        self.left_command_pub = self.create_publisher(
            JointState, "/left/control/joint_states", reliable_qos(1)
        )
        self.right_command_pub = self.create_publisher(
            JointState, "/right/control/joint_states", reliable_qos(1)
        )
        self.chassis_pub = self.create_publisher(Twist, "/cmd_vel", reliable_qos(1))
        self.create_subscription(
            JointState,
            "/left/feedback/joint_states",
            lambda msg: self._on_arm_state("left", msg),
            reliable_qos(1),
        )
        self.create_subscription(
            JointState,
            "/right/feedback/joint_states",
            lambda msg: self._on_arm_state("right", msg),
            reliable_qos(1),
        )
        camera_topics = []
        if subscribe_head_camera:
            camera_topics.append(("head", head_topic))
        if subscribe_wrist_cameras:
            camera_topics.extend(
                [
                    ("left_wrist", left_wrist_topic),
                    ("right_wrist", right_wrist_topic),
                ]
            )
        if image_transport == "zmq":
            endpoints = {}
            if subscribe_head_camera:
                endpoints["head"] = head_zmq_endpoint
            if subscribe_wrist_cameras:
                endpoints.update(
                    left_wrist=left_wrist_zmq_endpoint,
                    right_wrist=right_wrist_zmq_endpoint,
                )
            self._zmq_cameras = CameraSubscriberSet(endpoints, self._on_zmq_image)
            self._zmq_cameras.start()
            camera_topics = []
        for camera, topic in camera_topics:
            if image_transport == "compressed":
                self._compressed_pending[camera] = None
                self.create_subscription(
                    CompressedImage,
                    self._compressed_topic(topic),
                    lambda msg, name=camera: self._on_compressed_image(name, msg),
                    image_qos(),
                )
            else:
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=camera: self._on_image(name, msg),
                    image_qos(),
                )
        for camera in self._compressed_pending:
            worker = threading.Thread(
                target=self._decode_compressed_loop,
                args=(camera,),
                daemon=True,
                name=f"ammr-image-decode-{camera}",
            )
            worker.start()
            self._image_workers.append(worker)

        self.remote_recorder_client = self.create_client(SetBool, recorder_service)
        self.create_subscription(
            String,
            recorder_status_topic,
            self._on_recorder_status,
            status_qos(),
        )

        self.left_gate = self.create_client(SetBool, "/left/control_enable")
        self.right_gate = self.create_client(SetBool, "/right/control_enable")
        self.left_hold = self.create_client(Empty, "/left/emergency_stop")
        self.right_hold = self.create_client(Empty, "/right/emergency_stop")

        self.lift_client = None
        self._lift_request_type = None
        try:
            from lifting_msg_pkg.srv import LiftMotorSrv

            self._lift_request_type = LiftMotorSrv.Request
            self.lift_client = self.create_client(LiftMotorSrv, "/LiftingMotorService")
        except ImportError:
            self.get_logger().warning(
                "lifting_msg_pkg is unavailable; lift output remains disabled"
            )

        mode = (
            f"ENABLED components={sorted(self.enabled_components)}"
            if self.hardware_output
            else "DRY-RUN"
        )
        target = "Isaac virtual hardware" if target_mode == "sim" else "Real hardware"
        self.get_logger().warning(f"{target} output mode: {mode}")

    def _on_recorder_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning("Ignoring invalid AGX recorder status JSON")
            return
        if not isinstance(status, dict):
            return
        with self.lock:
            self.remote_recorder_status = status

    def recorder_status(self) -> dict:
        with self.lock:
            return dict(self.remote_recorder_status)

    def recorder_service_ready(self) -> bool:
        return self.remote_recorder_client.service_is_ready()

    def request_recording(self, enabled: bool):
        if not self.remote_recorder_client.service_is_ready():
            raise RuntimeError(
                f"AGX recorder service unavailable: {self.remote_recorder_client.srv_name}"
            )
        return self.remote_recorder_client.call_async(
            SetBool.Request(data=bool(enabled))
        )

    def _on_arm_state(self, side: str, msg: JointState) -> None:
        try:
            state = arm_feedback(msg.name, msg.position)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid {side} arm feedback: {error}")
            return
        # A configured gripper is expected to appear in feedback/joint_states.
        if not np.isfinite(state[6]):
            self.get_logger().warning(
                f"Ignoring {side} feedback without the gripper joint"
            )
            return
        with self.lock:
            if side == "left":
                self.left_state = state
                self.left_received_at = time.monotonic()
            else:
                self.right_state = state
                self.right_received_at = time.monotonic()

    @staticmethod
    def _compressed_topic(topic: str) -> str:
        topic = str(topic).rstrip("/")
        return topic if topic.endswith("/compressed") else f"{topic}/compressed"

    @staticmethod
    def _message_stamp(msg: Image | CompressedImage) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_image(self, name: str, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning(
                f"Ignoring {name} image encoding {msg.encoding!r}; expected rgb8/bgr8"
            )
            return
        expected_row = int(msg.width) * 3
        expected_size = int(msg.step) * int(msg.height)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.width <= 0 or msg.height <= 0 or msg.step < expected_row or raw.size < expected_size:
            return
        rows = raw[:expected_size].reshape(int(msg.height), int(msg.step))
        image = rows[:, :expected_row].reshape(int(msg.height), int(msg.width), 3)
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        sample = ImageSample(
            np.ascontiguousarray(image), time.monotonic(), self._message_stamp(msg)
        )
        with self.lock:
            self.images[name] = sample

    def _on_compressed_image(self, name: str, msg: CompressedImage) -> None:
        """Queue the newest compressed frame without blocking ROS callbacks."""
        if self._cv2 is None or name not in self._compressed_pending:
            return
        encoded = bytes(msg.data)
        if not encoded:
            return
        pending = PendingCompressedImage(encoded, self._message_stamp(msg))
        with self._image_condition:
            self._compressed_pending[name] = pending
            self._image_condition.notify_all()

    def _on_zmq_image(self, frame: CameraFrame) -> None:
        sample = ImageSample(frame.image, time.monotonic(), frame.source_stamp)
        with self.lock:
            self.images[frame.camera_name] = sample

    def _decode_compressed_loop(self, name: str) -> None:
        cv2 = self._cv2
        if cv2 is None:
            return
        while not self._image_stop.is_set():
            with self._image_condition:
                while (
                    self._compressed_pending.get(name) is None
                    and not self._image_stop.is_set()
                ):
                    self._image_condition.wait(timeout=0.5)
                if self._image_stop.is_set():
                    return
                pending = self._compressed_pending.get(name)
                self._compressed_pending[name] = None
            if pending is None:
                continue
            image = cv2.imdecode(
                np.frombuffer(pending.data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if image is None or image.ndim != 3 or image.shape[2] != 3:
                self.get_logger().warning(
                    f"Ignoring undecodable {name} compressed image"
                )
                continue
            sample = ImageSample(
                np.ascontiguousarray(image), time.monotonic(), pending.stamp
            )
            with self.lock:
                self.images[name] = sample

    def state(self) -> tuple[np.ndarray | None, tuple[float, ...]]:
        with self.lock:
            if "left" in self.required_arm_sides and self.left_state is None:
                return None, (self.left_received_at,)
            if "right" in self.required_arm_sides and self.right_state is None:
                return None, (self.right_received_at,)
            # A disabled arm may not have a driver running during staged
            # commissioning.  Use a virtual zero state only as the inactive
            # side of the dual-arm IK seed; no commands are published to it.
            left = self.left_state if self.left_state is not None else np.zeros(7)
            right = self.right_state if self.right_state is not None else np.zeros(7)
            timestamps = tuple(
                timestamp
                for side, timestamp in (
                    ("left", self.left_received_at),
                    ("right", self.right_received_at),
                )
                if side in self.required_arm_sides
            )
            return (
                combine_feedback(left, right),
                timestamps,
            )

    def camera_samples(self) -> dict[str, ImageSample]:
        with self.lock:
            return {
                name: ImageSample(sample.image.copy(), sample.received_at, sample.stamp)
                for name, sample in self.images.items()
            }

    def camera_sample(self, name: str) -> ImageSample | None:
        """Return one copied frame without copying the other camera streams."""
        with self.lock:
            sample = self.images.get(name)
            if sample is None:
                return None
            return ImageSample(sample.image.copy(), sample.received_at, sample.stamp)

    def publish_arm_command(self, command: np.ndarray, effort: float) -> None:
        # Final defense at the ROS publishing boundary. This also guarantees
        # the action topic observed by the recorder stays inside the contract.
        command = clamp_command(command)
        if not self.hardware_output:
            return
        for side, publisher in (
            ("left", self.left_command_pub),
            ("right", self.right_command_pub),
        ):
            if f"{side}_arm" not in self.enabled_components:
                continue
            names, positions = driver_command(command, side)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            msg.effort = [0.0] * 6 + [float(effort)]
            publisher.publish(msg)

    def publish_chassis(self, velocity: np.ndarray) -> None:
        velocity = np.asarray(velocity, dtype=np.float64)
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            raise ValueError("chassis velocity must contain 3 finite values")
        if not self.hardware_output or "chassis" not in self.enabled_components:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity.tolist()
        self.chassis_pub.publish(msg)

    def set_control_gate(self, enabled: bool) -> None:
        if not self.hardware_output:
            return
        for side, client in (("left", self.left_gate), ("right", self.right_gate)):
            if f"{side}_arm" not in self.enabled_components:
                continue
            if client.service_is_ready():
                client.call_async(SetBool.Request(data=bool(enabled)))
            else:
                self.get_logger().warning(f"Control gate service unavailable: {client.srv_name}")

    def emergency_hold(self) -> None:
        if not self.hardware_output:
            return
        for side, client in (("left", self.left_hold), ("right", self.right_hold)):
            if f"{side}_arm" not in self.enabled_components:
                continue
            if client.service_is_ready():
                client.call_async(Empty.Request())
        self.publish_chassis(np.zeros(3))
        self.stop_lift()

    def command_lift_speed(self, millimeters_per_second: int) -> None:
        if not self.hardware_output or "lift" not in self.enabled_components or self.lift_client is None or self._lift_request_type is None:
            return
        if not self.lift_client.service_is_ready():
            return
        request = self._lift_request_type()
        request.mode = -4
        request.val = int(millimeters_per_second)
        self.lift_client.call_async(request)

    def stop_lift(self) -> None:
        if not self.hardware_output or "lift" not in self.enabled_components or self.lift_client is None or self._lift_request_type is None:
            return
        if not self.lift_client.service_is_ready():
            return
        request = self._lift_request_type()
        request.mode = -2
        request.val = 0
        self.lift_client.call_async(request)

    def safe_shutdown(self) -> None:
        if not self.hardware_output:
            return
        self.set_control_gate(False)
        self.emergency_hold()
        # Repeat zero velocity so a best-effort shutdown is not one packet.
        for _ in range(3):
            self.publish_chassis(np.zeros(3))
            time.sleep(0.03)

    def destroy_node(self):
        self._image_stop.set()
        with self._image_condition:
            self._image_condition.notify_all()
        for worker in self._image_workers:
            worker.join(timeout=1.0)
        if self._zmq_cameras is not None:
            self._zmq_cameras.close()
        return super().destroy_node()
