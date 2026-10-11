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
                {
                    "ammr.capture.camera_head_age_ms": 2.5,
                    "ammr.capture.camera_head_reused": 0.0,
                },
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
            self.assertEqual(table.iloc[0]["timestamp"], 0.0)
            self.assertEqual(info["contract_version"], CONTRACT_VERSION)
            self.assertEqual(info["chunks_size"], 1000)
            self.assertEqual(info["total_chunks"], 1)
            self.assertEqual(info["features"]["action"]["names"], list(REAL_JOINTS))
            self.assertTrue(info["base_fixed_during_recording"])
            self.assertEqual(table.iloc[0]["ammr.capture.camera_head_age_ms"], 2.5)
            self.assertIn(
                "ammr.capture.camera_head_age_ms", info["capture_diagnostics"]
            )
            self.assertEqual(
                episode_stats["stats"]["ammr.capture.camera_head_age_ms"]["mean"],
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
                    {"ammr.capture.camera_head_age_ms": float(index)},
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
                {"ammr.capture.camera_head_age_ms": 0.0},
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
                        {"ammr.capture.camera_head_age_ms": float(index)},
                    )
                )
            parquet = recorder.stop()
            table = pd.read_parquet(parquet)
            self.assertEqual(len(table), 3)
            self.assertEqual(recorder.frames_written, 3)
            self.assertEqual(recorder.producer_drops, 0)
            self.assertFalse(recorder.active)

    def test_lerobot_timeline_is_dense_without_common_capture_timestamp(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = RealEpisodeRecorder(directory, 30.0, "timeline test")
            recorder.start()
            for capture_timestamp in (12.0, 12.2, 14.0):
                recorder.add_frame(
                    np.zeros(14),
                    np.zeros(14),
                    {"head": image, "left_wrist": image, "right_wrist": image},
                    {"ammr.capture.camera_head_received_monotonic_s": capture_timestamp},
                )
            parquet = recorder.stop()
            table = pd.read_parquet(parquet)
            expected = np.arange(3, dtype=np.float64) / 30.0
            np.testing.assert_array_equal(table["timestamp"].to_numpy(), expected)
            self.assertNotIn("ammr.sync.target_timestamp", table.columns)
            for name in ("head", "left_wrist", "right_wrist"):
                image_key = f"observation.images.{name}"
                image_timestamps = np.asarray(
                    [value["timestamp"] for value in table[image_key]],
                    dtype=np.float64,
                )
                np.testing.assert_array_equal(image_timestamps, expected)

    def test_common_target_timestamp_is_not_required_or_validated(self):
        image = np.zeros((24, 32, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            recorder = RealEpisodeRecorder(directory, 30.0, "timeline failure")
            recorder.start()
            for timestamp in (2.0, 1.0):
                recorder.add_frame(
                    np.zeros(14),
                    np.zeros(14),
                    {"head": image, "left_wrist": image, "right_wrist": image},
                    {"ammr.legacy.target_timestamp": timestamp},
                )
            parquet = recorder.stop()
            self.assertEqual(len(pd.read_parquet(parquet)), 2)


if __name__ == "__main__":
    unittest.main()
