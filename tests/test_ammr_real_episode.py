import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd

from ammr_teleop.common.data_contract import CONTRACT_VERSION, REAL_JOINTS
from ammr_teleop.real.episode import (
    interpolate_action,
    read_real_episode,
    select_frames,
)


class AmmrRealEpisodeTest(unittest.TestCase):
    def _dataset(self, directory: str, actions: list[np.ndarray]) -> Path:
        root = Path(directory)
        data_dir = root / "data" / "chunk-000"
        meta_dir = root / "meta"
        data_dir.mkdir(parents=True)
        meta_dir.mkdir(parents=True)
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
        (meta_dir / "info.json").write_text(json.dumps(info), encoding="utf-8")
        path = data_dir / "episode_000000.parquet"
        rows = []
        for index, action in enumerate(actions):
            rows.append(
                {
                    "frame_index": index,
                    "timestamp": index / 10.0,
                    "action": np.asarray(action).tolist(),
                    "observation.state": np.asarray(action).tolist(),
                }
            )
        pd.DataFrame(rows).to_parquet(path, index=False)
        return path

    def test_reads_strict_14d_episode_and_interpolates(self):
        first = np.zeros(14)
        second = np.zeros(14)
        second[0] = 0.2
        with tempfile.TemporaryDirectory() as directory:
            episode = read_real_episode(self._dataset(directory, [first, second]))
            self.assertEqual(len(episode.frames), 2)
            selected = select_frames(episode, 0, None)
            midpoint = interpolate_action(selected, 0.05)
            self.assertAlmostEqual(midpoint[0], 0.1)

    def test_rejects_action_outside_piper_limits(self):
        action = np.zeros(14)
        action[6] = 0.2
        with tempfile.TemporaryDirectory() as directory:
            path = self._dataset(directory, [action])
            with self.assertRaisesRegex(ValueError, "exceeds Piper limits"):
                read_real_episode(path)

    def test_requires_matching_contract_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self._dataset(directory, [np.zeros(14)])
            info_path = Path(directory) / "meta" / "info.json"
            info = json.loads(info_path.read_text())
            info["contract_version"] = "wrong"
            info_path.write_text(json.dumps(info))
            with self.assertRaisesRegex(ValueError, "contract mismatch"):
                read_real_episode(path)

    def test_can_explicitly_clamp_legacy_limit_violation(self):
        action = np.zeros(14)
        action[6] = 0.1049
        with tempfile.TemporaryDirectory() as directory:
            episode = read_real_episode(
                self._dataset(directory, [action]), clamp_limits=True
            )
            self.assertEqual(episode.limit_clamped_frames, 1)
            self.assertLessEqual(episode.frames[0].action[6], 0.1)


if __name__ == "__main__":
    unittest.main()
