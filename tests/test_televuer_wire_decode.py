import asyncio
from multiprocessing import Array, Value
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import msgpack
import numpy as np


TELEVUER_SRC = Path(__file__).resolve().parents[1] / "third_party" / "televuer" / "src"
sys.path.insert(0, str(TELEVUER_SRC))

from televuer.televuer import TeleVuer, decode_vuer_wire_value


class TeleVuerWireDecodeTest(unittest.TestCase):
    def test_nested_messagepack_scalars_are_decoded(self):
        payload = {
            "zero": msgpack.packb(0),
            "float": msgpack.packb(0.25),
            "false": msgpack.packb(False),
            "vector": [msgpack.packb(-0.5), msgpack.packb(0.75)],
        }

        decoded = decode_vuer_wire_value(payload)

        self.assertEqual(decoded["zero"], 0)
        self.assertEqual(decoded["float"], 0.25)
        self.assertIs(decoded["false"], False)
        self.assertEqual(decoded["vector"], [-0.5, 0.75])

    def test_genuine_binary_payload_is_preserved(self):
        payload = b"raw-binary-data"
        self.assertIs(decode_vuer_wire_value(payload), payload)

    def test_controller_event_accepts_binary_numeric_fields(self):
        tvuer = TeleVuer.__new__(TeleVuer)
        tvuer.left_arm_pose_shared = Array("d", 16, lock=True)
        tvuer.right_arm_pose_shared = Array("d", 16, lock=True)

        for prefix in ("left", "right"):
            setattr(tvuer, f"{prefix}_ctrl_trigger_shared", Value("b", False, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_triggerValue_shared", Value("d", 0.0, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_squeeze_shared", Value("b", False, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_squeezeValue_shared", Value("d", 0.0, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_thumbstick_shared", Value("b", False, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_thumbstickValue_shared", Array("d", 2, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_aButton_shared", Value("b", False, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_bButton_shared", Value("b", False, lock=True))

        pose = np.eye(4, dtype=np.float64).reshape(-1, order="F").tolist()
        encoded_pose = [msgpack.packb(value) for value in pose]
        controller_state = {
            "trigger": msgpack.packb(False),
            "triggerValue": msgpack.packb(0.25),
            "squeeze": msgpack.packb(True),
            "squeezeValue": msgpack.packb(0.75),
            "thumbstick": msgpack.packb(False),
            "thumbstickValue": [msgpack.packb(-0.5), msgpack.packb(0.5)],
            "aButton": msgpack.packb(False),
            "bButton": msgpack.packb(True),
        }
        event = SimpleNamespace(
            value={
                "left": encoded_pose,
                "leftState": controller_state,
                "right": encoded_pose,
                "rightState": controller_state,
            }
        )

        asyncio.run(tvuer.on_controller_move(event, None))

        self.assertFalse(tvuer.left_ctrl_trigger_shared.value)
        self.assertAlmostEqual(tvuer.left_ctrl_triggerValue_shared.value, 0.25)
        self.assertTrue(tvuer.left_ctrl_squeeze_shared.value)
        self.assertEqual(list(tvuer.left_ctrl_thumbstickValue_shared[:]), [-0.5, 0.5])
        self.assertFalse(tvuer.left_ctrl_aButton_shared.value)
        self.assertTrue(tvuer.left_ctrl_bButton_shared.value)

    def test_partial_controller_event_keeps_previous_state(self):
        tvuer = TeleVuer.__new__(TeleVuer)
        tvuer.left_arm_pose_shared = Array("d", [1.0] * 16, lock=True)
        tvuer.right_arm_pose_shared = Array("d", [2.0] * 16, lock=True)
        for prefix in ("left", "right"):
            setattr(tvuer, f"{prefix}_ctrl_trigger_shared", Value("b", True, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_triggerValue_shared", Value("d", 0.5, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_squeeze_shared", Value("b", True, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_squeezeValue_shared", Value("d", 0.5, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_thumbstick_shared", Value("b", True, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_thumbstickValue_shared", Array("d", [0.5, 0.5], lock=True))
            setattr(tvuer, f"{prefix}_ctrl_aButton_shared", Value("b", True, lock=True))
            setattr(tvuer, f"{prefix}_ctrl_bButton_shared", Value("b", True, lock=True))

        asyncio.run(tvuer.on_controller_move(SimpleNamespace(value={"left": [3.0] * 16}), None))

        self.assertEqual(list(tvuer.left_arm_pose_shared[:]), [1.0] * 16)
        self.assertEqual(list(tvuer.right_arm_pose_shared[:]), [2.0] * 16)
        self.assertTrue(tvuer.left_ctrl_trigger_shared.value)
        self.assertTrue(tvuer.right_ctrl_bButton_shared.value)


if __name__ == "__main__":
    unittest.main()
