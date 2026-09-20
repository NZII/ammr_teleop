from collections import deque
from dataclasses import dataclass
import unittest

from ammr_teleop.real.time_sync import (
    SyncMatchError,
    append_bounded,
    match_histories,
)


@dataclass(frozen=True)
class Sample:
    value: float
    received_at: float
    source_stamp: float = 0.0


class AmmrRealTimeSyncTest(unittest.TestCase):
    def test_action_never_uses_future_sample(self):
        feedback = {
            side: [Sample(1.0, 9.99)] for side in ("left", "right")
        }
        actions = {
            side: [Sample(1.0, 9.98), Sample(2.0, 10.01)]
            for side in ("left", "right")
        }
        images = {
            name: [Sample(1.0, 10.0, 100.0)]
            for name in ("head", "left_wrist", "right_wrist")
        }
        matched = match_histories(
            feedback,
            actions,
            images,
            target=10.0,
            state_max_age=0.25,
            action_max_age=0.25,
            camera_max_age=0.20,
            camera_max_skew=0.08,
        )
        self.assertEqual(matched.actions["left"].value, 1.0)
        self.assertAlmostEqual(
            matched.diagnostics["ammr.sync.left_action_age_ms"], 20.0
        )

    def test_nearest_feedback_and_images_are_selected(self):
        feedback = {
            side: [Sample(1.0, 9.90), Sample(2.0, 10.01)]
            for side in ("left", "right")
        }
        actions = {side: [Sample(1.0, 9.99)] for side in ("left", "right")}
        images = {
            name: [Sample(1.0, 9.95, 1.0), Sample(2.0, 10.01, 2.0)]
            for name in ("head", "left_wrist", "right_wrist")
        }
        matched = match_histories(
            feedback,
            actions,
            images,
            target=10.0,
            state_max_age=0.25,
            action_max_age=0.25,
            camera_max_age=0.20,
            camera_max_skew=0.08,
        )
        self.assertEqual(matched.feedback["right"].value, 2.0)
        self.assertEqual(matched.images["head"].value, 2.0)

    def test_rejects_camera_skew(self):
        feedback = {side: [Sample(1.0, 10.0)] for side in ("left", "right")}
        actions = {side: [Sample(1.0, 10.0)] for side in ("left", "right")}
        images = {
            "head": [Sample(1.0, 9.95)],
            "left_wrist": [Sample(1.0, 10.0)],
            "right_wrist": [Sample(1.0, 10.05)],
        }
        with self.assertRaisesRegex(SyncMatchError, "camera_skew"):
            match_histories(
                feedback,
                actions,
                images,
                target=10.0,
                state_max_age=0.25,
                action_max_age=0.25,
                camera_max_age=0.20,
                camera_max_skew=0.08,
            )

    def test_bounded_history_drops_old_entries(self):
        history = deque()
        append_bounded(history, Sample(1.0, 1.0), 0.5)
        append_bounded(history, Sample(2.0, 1.4), 0.5)
        append_bounded(history, Sample(3.0, 1.6), 0.5)
        self.assertEqual([sample.value for sample in history], [2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
