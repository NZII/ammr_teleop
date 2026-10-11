import unittest

from ammr_teleop.real.quality import (
    QualityThresholds,
    assess_episode_quality,
    episode_quality_metrics,
)


class AmmrRealQualityTest(unittest.TestCase):
    def test_quality_counts_only_writer_attempts_not_camera_reuse(self):
        metrics = episode_quality_metrics(
            frames=80,
            duration_s=3.0,
            producer_drops=5,
        )
        self.assertEqual(metrics["attempted_samples"], 85)
        self.assertEqual(metrics["producer_drop_ratio"], 5 / 85)

    def test_valid_episode(self):
        result = assess_episode_quality(
            {
                "frames": 300,
                "actual_fps": 27.5,
                "producer_drops": 0,
            },
            QualityThresholds(),
        )
        self.assertTrue(result["valid"])
        self.assertEqual(result["issues"], [])

    def test_invalid_episode_reports_all_failed_thresholds(self):
        result = assess_episode_quality(
            {
                "frames": 10,
                "actual_fps": 9.0,
                "rejected_ratio": 0.4,
                "duplicate_ratio": 0.5,
                "target_skip_ratio": 0.2,
                "producer_drops": 2,
            },
            QualityThresholds(),
        )
        self.assertFalse(result["valid"])
        self.assertEqual(len(result["issues"]), 3)


if __name__ == "__main__":
    unittest.main()
