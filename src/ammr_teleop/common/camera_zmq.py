"""Versioned latest-frame ZMQ transport for AMMR RGB cameras."""

from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time
from typing import Callable, Mapping

import cv2
import numpy as np


PROTOCOL_VERSION = 1
CAMERA_NAMES = ("head", "left_wrist", "right_wrist")


@dataclass(frozen=True)
class CameraFrame:
    camera_name: str
    image: np.ndarray
    sequence: int
    source_stamp: float
    received_at: float
    capture_monotonic: float
    bundle_sequence: int | None = None


def encode_metadata(
    camera_name: str,
    *,
    sequence: int,
    source_timestamp_ns: int,
    capture_monotonic_ns: int,
    width: int,
    height: int,
    jpeg_quality: int,
    bundle_sequence: int | None = None,
) -> bytes:
    if camera_name not in CAMERA_NAMES:
        raise ValueError(f"unknown camera name: {camera_name}")
    metadata = {
        "schema_version": PROTOCOL_VERSION,
        "camera_name": camera_name,
        "sequence": int(sequence),
        "source_timestamp_ns": int(source_timestamp_ns),
        "capture_monotonic_ns": int(capture_monotonic_ns),
        "bundle_sequence": None if bundle_sequence is None else int(bundle_sequence),
        "width": int(width),
        "height": int(height),
        "encoding": "bgr8",
        "codec": "jpeg",
        "jpeg_quality": int(jpeg_quality),
    }
    return json.dumps(metadata, separators=(",", ":")).encode("utf-8")


def decode_message(metadata_bytes: bytes, jpeg: bytes, received_at: float) -> CameraFrame:
    metadata = json.loads(metadata_bytes.decode("utf-8"))
    if int(metadata.get("schema_version", -1)) != PROTOCOL_VERSION:
        raise ValueError(f"unsupported camera schema: {metadata.get('schema_version')}")
    name = str(metadata["camera_name"])
    if name not in CAMERA_NAMES:
        raise ValueError(f"unknown camera name: {name}")
    if metadata.get("codec") != "jpeg" or metadata.get("encoding") != "bgr8":
        raise ValueError("camera packet must contain a bgr8 JPEG")
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode camera JPEG")
    height, width = image.shape[:2]
    if (width, height) != (int(metadata["width"]), int(metadata["height"])):
        raise ValueError("decoded image dimensions do not match metadata")
    bundle = metadata.get("bundle_sequence")
    return CameraFrame(
        camera_name=name,
        image=image,
        sequence=int(metadata["sequence"]),
        source_stamp=int(metadata["source_timestamp_ns"]) * 1e-9,
        received_at=float(received_at),
        capture_monotonic=int(metadata["capture_monotonic_ns"]) * 1e-9,
        bundle_sequence=None if bundle is None else int(bundle),
    )


class CameraPublisher:
    """Python publisher used by Isaac's shared-memory ZMQ relay."""

    def __init__(self, camera_name: str, endpoint: str, *, hwm: int = 2) -> None:
        import zmq

        if camera_name not in CAMERA_NAMES:
            raise ValueError(f"unknown camera name: {camera_name}")
        self.camera_name = camera_name
        self._zmq = zmq
        self._socket = zmq.Context.instance().socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, int(hwm))
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(endpoint)

    def publish(
        self,
        image: np.ndarray,
        *,
        sequence: int,
        source_timestamp_ns: int,
        capture_monotonic_ns: int,
        bundle_sequence: int | None = None,
        jpeg_quality: int = 92,
    ) -> bool:
        frame = np.asarray(image)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("camera image must be HxWx3 uint8")
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        height, width = frame.shape[:2]
        metadata = encode_metadata(
            self.camera_name,
            sequence=sequence,
            source_timestamp_ns=source_timestamp_ns,
            capture_monotonic_ns=capture_monotonic_ns,
            width=width,
            height=height,
            jpeg_quality=jpeg_quality,
            bundle_sequence=bundle_sequence,
        )
        try:
            self._socket.send_multipart(
                [metadata, memoryview(encoded)], flags=self._zmq.DONTWAIT, copy=False
            )
            return True
        except self._zmq.Again:
            return False

    def close(self) -> None:
        self._socket.close(linger=0)


class CameraSubscriberSet:
    """One independent latest-frame subscriber thread per enabled camera."""

    def __init__(
        self,
        endpoints: Mapping[str, str],
        on_frame: Callable[[CameraFrame], None],
        *,
        hwm: int = 2,
    ) -> None:
        unknown = set(endpoints) - set(CAMERA_NAMES)
        if unknown:
            raise ValueError(f"unknown cameras: {sorted(unknown)}")
        self.endpoints = dict(endpoints)
        self.on_frame = on_frame
        self.hwm = int(hwm)
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []
        self.errors: dict[str, str] = {}
        self.frames_received = {name: 0 for name in endpoints}
        self.sequence_drops = {name: 0 for name in endpoints}
        self._last_sequence: dict[str, int] = {}

    def start(self) -> None:
        if self.threads:
            return
        for name, endpoint in self.endpoints.items():
            thread = threading.Thread(
                target=self._run,
                args=(name, endpoint),
                name=f"ammr-zmq-{name}",
                daemon=True,
            )
            thread.start()
            self.threads.append(thread)

    def _run(self, expected_name: str, endpoint: str) -> None:
        import zmq

        socket = zmq.Context.instance().socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"")
        socket.setsockopt(zmq.RCVHWM, self.hwm)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(endpoint)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        try:
            while not self.stop_event.is_set():
                if not poller.poll(100):
                    continue
                latest = socket.recv_multipart()
                while socket.getsockopt(zmq.EVENTS) & zmq.POLLIN:
                    latest = socket.recv_multipart()
                if len(latest) != 2:
                    self.errors[expected_name] = "expected two-part camera packet"
                    continue
                try:
                    frame = decode_message(latest[0], latest[1], time.monotonic())
                    if frame.camera_name != expected_name:
                        raise ValueError(
                            f"endpoint for {expected_name} sent {frame.camera_name}"
                        )
                except Exception as error:
                    self.errors[expected_name] = str(error)
                    continue
                previous = self._last_sequence.get(expected_name)
                if previous is not None and frame.sequence > previous + 1:
                    self.sequence_drops[expected_name] += frame.sequence - previous - 1
                self._last_sequence[expected_name] = frame.sequence
                self.frames_received[expected_name] += 1
                self.errors.pop(expected_name, None)
                self.on_frame(frame)
        finally:
            poller.unregister(socket)
            socket.close(linger=0)

    def close(self, timeout: float = 2.0) -> None:
        self.stop_event.set()
        deadline = time.monotonic() + timeout
        for thread in self.threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        self.threads.clear()

