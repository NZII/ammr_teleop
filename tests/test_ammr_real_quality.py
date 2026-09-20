import unittest

from ammr_teleop.real.quality import QualityThresholds, assess_episode_quality


class AmmrRealQualityTest(unittest.TestCase):
    def test_valid_episode(self):
        result = assess_episode_quality(
            {
                "frames": 300,
                "actual_fps": 27.5,
                "rejected_ratio": 0.05,
                "duplicate_ratio": 0.08,
                "target_skip_ratio": 0.0,
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
        self.assertEqual(len(result["issues"]), 6)


if __name__ == "__main__":
    unittest.main()
