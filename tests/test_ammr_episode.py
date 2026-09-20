import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from ammr_teleop.sim.episode import read_episode
from ammr_teleop.common.data_contract import REAL_JOINTS, to_sim_contract


class AmmrEpisodeTest(unittest.TestCase):
    def test_json_episode_round_trip(self):
        action = np.zeros(14)
        action[0] = 0.2
        action[[6, 13]] = [0.08, 0.06]
        observation = action * 0.9
        payload = {
            "info": {
                "robot_type": "ammr",
                "control_source": "xr",
                "image": {"fps": 30},
            },
            "data": [
                {
                    "idx": 0,
                    "actions": action.tolist(),
                    "states": observation.tolist(),
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            episode = read_episode(path)
        self.assertEqual(len(episode.frames), 1)
        np.testing.assert_allclose(episode.frames[0].action, to_sim_contract(action))
        np.testing.assert_allclose(
            episode.frames[0].observation, to_sim_contract(observation)
        )
        self.assertEqual(episode.metadata["control_source"], "xr")

    def test_joint_limit_violation_is_rejected(self):
        action = np.zeros(14)
        action[0] = 10.0
        payload = {
            "info": {"image": {"fps": 30}},
            "data": [{"idx": 0, "actions": action.tolist()}],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "joint limits"):
                read_episode(path, strict_limits=True)

    def test_lerobot_21d_episode_is_rejected(self):
        import pandas as pd

        command = np.zeros(21)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data_dir = root / "data" / "chunk-000"
            meta_dir = root / "meta"
            data_dir.mkdir(parents=True)
            meta_dir.mkdir()
            path = data_dir / "episode_000000.parquet"
            pd.DataFrame(
                [
                    {
                        "observation.state": command.tolist(),
                        "action": command.tolist(),
                        "timestamp": 0.0,
                        "frame_index": 0,
                    }
                ]
            ).to_parquet(path, index=False)
            info = {
                "fps": 30,
                "robot_type": "ammr",
                "features": {
                    "observation.state": {"names": [f"joint_{i}" for i in range(21)]},
                    "action": {"names": [f"joint_{i}" for i in range(21)]},
                },
            }
            (meta_dir / "info.json").write_text(
                json.dumps(info), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "expected 14 finite values"):
                read_episode(path)

    def test_lerobot_arm14_episode_embeds_into_sim_contract(self):
        import pandas as pd

        arm14 = np.array(
            [0.1, 0.2, -0.3, 0.4, -0.5, 0.6, 0.08,
             -0.1, 0.3, -0.2, 0.5, -0.4, 0.2, 0.02]
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data_dir = root / "data" / "chunk-000"
            meta_dir = root / "meta"
            data_dir.mkdir(parents=True)
            meta_dir.mkdir()
            path = data_dir / "episode_000000.parquet"
            pd.DataFrame(
                [{
                    "observation.state": arm14.tolist(),
                    "action": arm14.tolist(),
                    "timestamp": 0.0,
                    "frame_index": 0,
                }]
            ).to_parquet(path, index=False)
            (meta_dir / "info.json").write_text(
                json.dumps({
                    "fps": 30,
                    "features": {
                        "observation.state": {"names": list(REAL_JOINTS)},
                        "action": {"names": list(REAL_JOINTS)},
                    },
                }),
                encoding="utf-8",
            )
            episode = read_episode(path)

        command = episode.frames[0].action
        np.testing.assert_allclose(command[0:3], 0.0)
        np.testing.assert_allclose(command[3:9], arm14[0:6])
        np.testing.assert_allclose(command[9:15], arm14[7:13])
        np.testing.assert_allclose(command[15:17], [0.04, -0.04])
        np.testing.assert_allclose(command[17:19], [0.01, -0.01])
        np.testing.assert_allclose(command[19:21], 0.0)


if __name__ == "__main__":
    unittest.main()
