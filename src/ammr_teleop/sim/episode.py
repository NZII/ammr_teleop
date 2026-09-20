"""Read and validate AMMR episodes independently of their storage format."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from ammr_teleop.common.contract import (
    POSITION_LOWER,
    POSITION_UPPER,
    ROBOT_JOINTS,
)


@dataclass(frozen=True)
class EpisodeFrame:
    index: int
    timestamp: float
    action: np.ndarray
    observation: np.ndarray | None


@dataclass(frozen=True)
class Episode:
    path: Path
    frames: tuple[EpisodeFrame, ...]
    fps: float
    metadata: dict[str, Any]

    @property
    def duration(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        return self.frames[-1].timestamp - self.frames[0].timestamp


def _flat_record_to_contract(values: Any, names: Any = None) -> np.ndarray:
    if isinstance(values, dict):
        raise ValueError("grouped 21-D recordings are no longer supported")
    vector = np.asarray(values, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"expected finite values, got {vector.shape}")
    if vector.shape != (14,):
        raise ValueError(f"expected 14 finite values, got {vector.shape}")

    from ammr_teleop.common.data_contract import REAL_JOINTS, to_sim_contract

    if isinstance(names, list):
        if len(names) != 14:
            raise ValueError(f"expected 14 joint names, got {len(names)}")
        by_name = dict(zip(names, vector))
        missing = [name for name in REAL_JOINTS if name not in by_name]
        if missing:
            raise ValueError(f"14-D recording is missing joints: {missing}")
        vector = np.asarray([by_name[name] for name in REAL_JOINTS])
    return to_sim_contract(vector)


def _infer_lerobot_root(parquet_path: Path) -> Path:
    # <dataset>/data/chunk-000/episode_XXXXXX.parquet
    if parquet_path.parent.name.startswith("chunk-") and parquet_path.parent.parent.name == "data":
        return parquet_path.parent.parent.parent
    return parquet_path.parent


def read_lerobot(path: Path) -> Episode:
    try:
        import pandas as pd
    except ImportError as exc:
        raise RuntimeError("pandas and pyarrow are required to read LeRobot data") from exc

    dataset_root = _infer_lerobot_root(path)
    info_path = dataset_root / "meta" / "info.json"
    metadata: dict[str, Any] = {}
    if info_path.is_file():
        with info_path.open("r", encoding="utf-8") as stream:
            metadata = json.load(stream)
    features = metadata.get("features", {})
    action_names = features.get("action", {}).get("names")
    state_names = features.get("observation.state", {}).get("names")
    fps = float(metadata.get("fps", 30.0))
    table = pd.read_parquet(path)
    if "action" not in table:
        raise ValueError("LeRobot parquet does not contain an action column")

    frames: list[EpisodeFrame] = []
    for row_number, row in table.iterrows():
        index = int(row.get("frame_index", row_number))
        timestamp = float(row.get("timestamp", index / fps))
        action = _flat_record_to_contract(row["action"], action_names)
        observation = None
        if "observation.state" in row and row["observation.state"] is not None:
            observation = _flat_record_to_contract(
                row["observation.state"], state_names
            )
        frames.append(EpisodeFrame(index, timestamp, action, observation))
    return Episode(path, tuple(frames), fps, metadata)


def read_json_episode(path: Path) -> Episode:
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    metadata = payload.get("info", {})
    fps = float(metadata.get("image", {}).get("fps", 30.0))
    frames: list[EpisodeFrame] = []
    for row_number, item in enumerate(payload.get("data", [])):
        index = int(item.get("idx", row_number))
        timestamp = float(item.get("timestamp", index / fps))
        action = _flat_record_to_contract(item["actions"])
        states = item.get("states")
        observation = (
            None
            if states is None
            else _flat_record_to_contract(states)
        )
        frames.append(EpisodeFrame(index, timestamp, action, observation))
    return Episode(path, tuple(frames), fps, metadata)


def read_episode(path: str | Path, strict_limits: bool = False) -> Episode:
    episode_path = Path(path).expanduser().resolve()
    if not episode_path.is_file():
        raise FileNotFoundError(f"episode does not exist: {episode_path}")
    if episode_path.suffix.lower() == ".parquet":
        episode = read_lerobot(episode_path)
    elif episode_path.suffix.lower() == ".json":
        episode = read_json_episode(episode_path)
    else:
        raise ValueError("episode path must end in .parquet or .json")
    validate_episode(episode, strict_limits=strict_limits)
    return episode


def joint_limit_violation_frames(episode: Episode) -> int:
    return sum(
        bool(
            np.any(frame.action < POSITION_LOWER - 1e-6)
            or np.any(frame.action > POSITION_UPPER + 1e-6)
        )
        for frame in episode.frames
    )


def validate_episode(episode: Episode, strict_limits: bool = False) -> None:
    if not episode.frames:
        raise ValueError("episode contains no frames")
    if not np.isfinite(episode.fps) or episode.fps <= 0.0:
        raise ValueError("episode fps must be finite and positive")
    previous_timestamp: float | None = None
    for frame in episode.frames:
        if not np.all(np.isfinite(frame.action)):
            raise ValueError(f"frame {frame.index} action is not finite")
        if strict_limits and (
            np.any(frame.action < POSITION_LOWER - 1e-6) or np.any(
            frame.action > POSITION_UPPER + 1e-6
            )
        ):
            bad = [
                ROBOT_JOINTS[index]
                for index in np.flatnonzero(
                    (frame.action < POSITION_LOWER - 1e-6)
                    | (frame.action > POSITION_UPPER + 1e-6)
                )
            ]
            raise ValueError(f"frame {frame.index} exceeds joint limits: {bad}")
        if frame.observation is not None and not np.all(
            np.isfinite(frame.observation)
        ):
            raise ValueError(f"frame {frame.index} observation is not finite")
        if previous_timestamp is not None and frame.timestamp < previous_timestamp:
            raise ValueError("episode timestamps are not monotonic")
        previous_timestamp = frame.timestamp


def episode_summary(episode: Episode) -> str:
    source = episode.metadata.get("control_source", "unknown")
    violations = joint_limit_violation_frames(episode)
    return (
        f"{episode.path}: {len(episode.frames)} frames, "
        f"{episode.duration:.3f}s, {episode.fps:g}Hz, source={source}, "
        f"limit_warning_frames={violations}"
    )
