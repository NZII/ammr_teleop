import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ammr_teleop.real.jitter_diagnostics import JitterCsvLogger


class JitterCsvLoggerTest(unittest.TestCase):
    def test_writes_one_complete_control_chain_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "jitter.csv"
            logger = JitterCsvLogger(path)
            metadata_path = logger.write_metadata(
                {
                    "profile": {"name": "piper_absolute_smooth_v1"},
                    "effective_translation_gain": 1.3,
                }
            )
            pose = np.eye(4)
            logger.write(
                timestamp=1.5,
                enabled=True,
                reset_active=False,
                ik_success=True,
                ik_solve_ms=2.0,
                xr_left=pose,
                xr_right=pose,
                target_raw=(pose, pose),
                target_filtered=(pose, pose),
                ik_seed=np.full(14, 0.5),
                ik_solution=np.zeros(14),
                command=np.ones(14),
                command_velocity=np.full(14, 0.2),
                command_acceleration=np.full(14, 0.8),
                feedback=np.full(14, 2.0),
            )
            logger.close()
            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.reader(file))
            with metadata_path.open(encoding="utf-8") as file:
                metadata = json.load(file)
        self.assertEqual(len(rows), 2)
        self.assertIn("ik_solve_ms", rows[0])
        self.assertIn("feedback_q_13", rows[0])
        self.assertEqual(len(rows[0]), len(rows[1]))
        self.assertEqual(metadata_path, Path(f"{path}.meta.json"))
        self.assertEqual(metadata["profile"]["name"], "piper_absolute_smooth_v1")
        self.assertAlmostEqual(metadata["effective_translation_gain"], 1.3)
