import time

import cv2
import numpy as np
import pytest

from ammr_teleop.common.camera_zmq import decode_message, encode_metadata


def test_camera_message_round_trip():
    image = np.zeros((24, 32, 3), dtype=np.uint8)
    image[:, :, 1] = 190
    ok, jpeg = cv2.imencode(".jpg", image)
    assert ok
    metadata = encode_metadata(
        "head", sequence=17, source_timestamp_ns=123_000_000,
        capture_monotonic_ns=456_000_000, width=32, height=24,
        jpeg_quality=92, bundle_sequence=9,
    )
    frame = decode_message(metadata, jpeg.tobytes(), time.monotonic())
    assert frame.camera_name == "head"
    assert frame.sequence == 17
    assert frame.bundle_sequence == 9
    assert frame.image.shape == image.shape
    assert frame.source_stamp == pytest.approx(0.123)
    assert frame.capture_monotonic == pytest.approx(0.456)


def test_camera_message_rejects_dimension_mismatch():
    image = np.zeros((8, 8, 3), dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", image)
    assert ok
    metadata = encode_metadata(
        "left_wrist", sequence=1, source_timestamp_ns=1,
        capture_monotonic_ns=1, width=9, height=8, jpeg_quality=90,
    )
    with pytest.raises(ValueError, match="dimensions"):
        decode_message(metadata, jpeg.tobytes(), time.monotonic())

