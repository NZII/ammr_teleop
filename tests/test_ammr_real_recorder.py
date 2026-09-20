import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

from ammr_teleop.common.data_contract import CONTRACT_VERSION, REAL_JOINTS
from ammr_teleop.real.recorder import (
    AsyncEpisodeRecorder,
    ProcessEpisodeRecorder,
    RealEpisodeRecorder,
)


class AmmrRealRecorderTest(unittest.TestCase):
    def test_lerobot_episode_has_14d_contract_and_three_videos(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = RealEpisodeRecorder(directory, 30.0, "test task")
            self.assertEqual(recorder.start(), 0)
            recorder.add_frame(
                np.zeros(14),
                np.ones(14) * 0.01,
                {"head": image, "left_wrist": image, "right_wrist": image},
                {"ammr.sync.camera_skew_ms": 2.5},
            )
            quality = {"valid": True, "issues": [], "metrics": {"frames": 1}}
            parquet = recorder.stop(quality)
            table = pd.read_parquet(parquet)
            info = json.loads((Path(directory) / "meta" / "info.json").read_text())
            episode_stats = json.loads(
                (Path(directory) / "meta" / "episodes_stats.jsonl")
                .read_text()
                .strip()
            )
            episode_meta = json.loads(
                (Path(directory) / "meta" / "episodes.jsonl")
                .read_text()
                .strip()
            )
            self.assertEqual(len(table.iloc[0]["action"]), 14)
            self.assertEqual(info["contract_version"], CONTRACT_VERSION)
            self.assertEqual(info["features"]["action"]["names"], list(REAL_JOINTS))
            self.assertTrue(info["base_fixed_during_recording"])
            self.assertEqual(table.iloc[0]["ammr.sync.camera_skew_ms"], 2.5)
            self.assertIn(
                "ammr.sync.camera_skew_ms", info["sync_diagnostics"]
            )
            self.assertEqual(
                episode_stats["stats"]["ammr.sync.camera_skew_ms"]["mean"],
                2.5,
            )
            self.assertTrue(episode_meta["quality"]["valid"])
            self.assertFalse((Path(directory) / ".in_progress" / "episode_000000").exists())
            for name in ("head", "left_wrist", "right_wrist"):
                path = Path(directory) / "videos" / "chunk-000" / f"observation.images.{name}" / "episode_000000.mp4"
                self.assertTrue(path.is_file())

    def test_async_recorder_drains_queue_before_saving(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = AsyncEpisodeRecorder(
                directory, 30.0, "async test", queue_size=4
            )
            self.assertEqual(recorder.start(), 0)
            for index in range(3):
                while not recorder.try_add_frame(
                    np.ones(14) * index,
                    np.ones(14) * (index + 1),
                    {"head": image, "left_wrist": image, "right_wrist": image},
                ):
                    pass
            parquet = recorder.stop()
            table = pd.read_parquet(parquet)
            self.assertEqual(len(table), 3)
            self.assertEqual(recorder.frames_written, 3)
            self.assertEqual(recorder.producer_drops, 0)
            self.assertFalse(recorder.active)

    def test_failed_validation_preserves_in_progress_episode(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = RealEpisodeRecorder(directory, 30.0, "failure test")
            recorder.start()
            recorder.add_frame(
                np.zeros(14),
                np.zeros(14),
                {"head": image, "left_wrist": image, "right_wrist": image},
            )
            with patch.object(
                recorder,
                "_validate_staged_episode",
                side_effect=RuntimeError("validation failed"),
            ):
                with self.assertRaisesRegex(RuntimeError, "validation failed"):
                    recorder.stop()
            self.assertTrue(
                (Path(directory) / ".in_progress" / "episode_000000").is_dir()
            )
            self.assertFalse(
                (Path(directory) / "data" / "chunk-000" / "episode_000000.parquet").exists()
            )
            recorder.preserve_partial()

    def test_process_recorder_drains_queue_before_saving(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = ProcessEpisodeRecorder(
                directory, 30.0, "process test", queue_size=4
            )
            self.assertEqual(recorder.start(), 0)
            for index in range(3):
                self.assertTrue(
                    recorder.try_add_frame(
                        np.ones(14) * index,
                        np.ones(14) * (index + 1),
                        {
                            "head": image.copy(),
                            "left_wrist": image.copy(),
                            "right_wrist": image.copy(),
                        },
                    )
                )
            parquet = recorder.stop()
            table = pd.read_parquet(parquet)
            self.assertEqual(len(table), 3)
            self.assertEqual(recorder.frames_written, 3)
            self.assertEqual(recorder.producer_drops, 0)
            self.assertFalse(recorder.active)


if __name__ == "__main__":
    unittest.main()
