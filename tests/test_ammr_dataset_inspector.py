import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd

from ammr_teleop.analysis.dataset_inspector import (
    calculate_metrics,
    episode_paths,
    scan_dataset,
    write_summary,
)
from ammr_teleop.common.data_contract import CONTRACT_VERSION, REAL_JOINTS
from ammr_teleop.real.episode import read_real_episode


class AmmrDatasetInspectorTest(unittest.TestCase):
    def _dataset(self, directory: str) -> Path:
        root = Path(directory)
        data = root / "data" / "chunk-000"
        meta = root / "meta"
        data.mkdir(parents=True)
        meta.mkdir()
        info = {
            "contract_version": CONTRACT_VERSION,
            "fps": 10.0,
            "features": {
                "action": {"shape": [14], "names": list(REAL_JOINTS)},
                "observation.state": {
                    "shape": [14],
                    "names": list(REAL_JOINTS),
                },
            },
        }
        (meta / "info.json").write_text(json.dumps(info), encoding="utf-8")
        for episode_index in range(2):
            final = np.zeros(14)
            final[0] = 0.01 * (episode_index + 1)
            actions = [np.zeros(14), final]
            states = [np.zeros(14), np.zeros(14)]
            rows = [
                {
                    "frame_index": index,
                    "timestamp": index / 10.0,
                    "ammr.sync.target_timestamp": index / 5.0,
                    "action": action.tolist(),
                    "observation.state": state.tolist(),
                }
                for index, (action, state) in enumerate(zip(actions, states))
            ]
            pd.DataFrame(rows).to_parquet(
                data / f"episode_{episode_index:06d}.parquet", index=False
            )
        quality_rows = [
            {
                "episode_index": 0,
                "quality": {
                    "valid": False,
                    "issues": ["actual_fps=7.50 < 8.00"],
                    "metrics": {
                        "actual_fps": 7.5,
                        "rejected_ratio": 0.1,
                        "duplicate_ratio": 0.2,
                        "target_skip_ratio": 0.05,
                    },
                },
            },
            {
                "episode_index": 1,
                "quality": {
                    "valid": True,
                    "issues": [],
                    "metrics": {"actual_fps": 9.5},
                },
            },
        ]
        (meta / "episodes.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in quality_rows),
            encoding="utf-8",
        )
        return root

    def test_scans_multiple_episodes_and_computes_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self._dataset(directory)
            self.assertEqual(len(episode_paths(root)), 2)
            metrics = scan_dataset(root)
            self.assertEqual(len(metrics), 2)
            self.assertTrue(all(item.valid for item in metrics))
            self.assertAlmostEqual(metrics[0].nominal_hz, 10.0)
            self.assertAlmostEqual(metrics[0].timeline_hz, 10.0)
            self.assertAlmostEqual(metrics[0].actual_hz, 7.5)
            self.assertAlmostEqual(metrics[0].capture_duration_s, 0.2)
            self.assertEqual(metrics[0].actual_hz_source, "episodes.jsonl quality")
            self.assertFalse(metrics[0].quality_valid)
            self.assertAlmostEqual(metrics[0].duplicate_ratio, 0.2)
            self.assertAlmostEqual(metrics[1].max_action_step, 0.02)

    def test_uses_target_timestamp_when_quality_metadata_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self._dataset(directory)
            (root / "meta" / "episodes.jsonl").unlink()
            metrics = scan_dataset(root)
            self.assertAlmostEqual(metrics[0].timeline_hz, 10.0)
            self.assertAlmostEqual(metrics[0].actual_hz, 5.0)
            self.assertAlmostEqual(metrics[0].capture_duration_s, 0.2)
            self.assertEqual(
                metrics[0].actual_hz_source, "ammr.sync.target_timestamp"
            )
            self.assertIsNone(metrics[0].quality_valid)

    def test_calculates_per_joint_tracking_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self._dataset(directory)
            episode = read_real_episode(episode_paths(root)[0])
            metrics = calculate_metrics(episode)
            self.assertEqual(len(metrics.joint_rmse), 14)
            self.assertGreater(metrics.mean_rmse, 0.0)

    def test_writes_one_dataset_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self._dataset(directory)
            output = Path(directory) / "summary.json"
            write_summary(scan_dataset(root), output)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(len(payload["episodes"]), 2)
            self.assertEqual(payload["joint_names"], list(REAL_JOINTS))


if __name__ == "__main__":
    unittest.main()
