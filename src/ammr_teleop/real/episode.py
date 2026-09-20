"""Read and strictly validate AMMR/Piper 14-D real-robot episodes."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ammr_teleop.common.data_contract import (
    CONTRACT_VERSION,
    POSITION_LOWER,
    POSITION_UPPER,
    REAL_JOINTS,
    validate_vector,
)


@dataclass(frozen=True)
class RealEpisodeFrame:
    index: int
    timestamp: float
    action: np.ndarray
    observation: np.ndarray | None


@dataclass(frozen=True)
class RealEpisode:
    path: Path
    frames: tuple[RealEpisodeFrame, ...]
    fps: float
    metadata: dict[str, Any]
    limit_clamped_frames: int = 0

    @property
    def duration(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        return self.frames[-1].timestamp - self.frames[0].timestamp


def _dataset_root(parquet_path: Path) -> Path:
    if (
        parquet_path.parent.name.startswith("chunk-")
        and parquet_path.parent.parent.name == "data"
    ):
        return parquet_path.parent.parent.parent
    return parquet_path.parent


def _ordered_vector(values: Any, names: Sequence[str] | None, field: str) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64).reshape(-1)
    if names is not None:
        if len(names) != len(vector) or len(set(names)) != len(names):
            raise ValueError(f"{field} names do not match its vector")
        by_name = dict(zip(names, vector))
        missing = [name for name in REAL_JOINTS if name not in by_name]
        if missing:
            raise ValueError(f"{field} is missing joints: {missing}")
        vector = np.asarray([by_name[name] for name in REAL_JOINTS])
    try:
        return validate_vector(vector)
    except ValueError as error:
        raise ValueError(f"invalid {field}: {error}") from error


def read_real_episode(path: str | Path, *, clamp_limits: bool = False) -> RealEpisode:
    episode_path = Path(path).expanduser().resolve()
    if not episode_path.is_file():
        raise FileNotFoundError(f"episode does not exist: {episode_path}")
    if episode_path.suffix.lower() != ".parquet":
        raise ValueError("real-robot replay requires a LeRobot .parquet episode")

    info_path = _dataset_root(episode_path) / "meta" / "info.json"
    if not info_path.is_file():
        raise ValueError(f"real-robot dataset metadata is missing: {info_path}")
    metadata = json.loads(info_path.read_text(encoding="utf-8"))
    if metadata.get("contract_version") != CONTRACT_VERSION:
        raise ValueError(
            "real-robot contract mismatch: "
            f"expected {CONTRACT_VERSION}, got {metadata.get('contract_version')!r}"
        )
    features = metadata.get("features")
    if not isinstance(features, dict):
        raise ValueError("dataset metadata does not contain features")
    action_feature = features.get("action", {})
    state_feature = features.get("observation.state", {})
    action_names = action_feature.get("names")
    state_names = state_feature.get("names")
    if list(action_names or ()) != list(REAL_JOINTS):
        raise ValueError("dataset action names do not match the Piper 14-D contract")
    if list(state_names or ()) != list(REAL_JOINTS):
        raise ValueError("dataset observation names do not match the Piper 14-D contract")

    fps = float(metadata.get("fps", 0.0))
    if not np.isfinite(fps) or fps <= 0.0:
        raise ValueError("dataset fps must be finite and positive")
    try:
        import pandas as pd
    except ImportError as error:
        raise RuntimeError("pandas and pyarrow are required for replay") from error
    table = pd.read_parquet(episode_path)
    if "action" not in table.columns:
        raise ValueError("episode does not contain action")
    if "observation.state" not in table.columns:
        raise ValueError("episode does not contain observation.state")

    frames: list[RealEpisodeFrame] = []
    limit_clamped_frames = 0
    prior_timestamp: float | None = None
    for row_number, row in table.iterrows():
        index = int(row.get("frame_index", row_number))
        timestamp = float(row.get("timestamp", row_number / fps))
        if not np.isfinite(timestamp):
            raise ValueError(f"frame {index} timestamp is not finite")
        if prior_timestamp is not None and timestamp <= prior_timestamp:
            raise ValueError("episode timestamps must be strictly increasing")
        action = _ordered_vector(row["action"], action_names, f"frame {index} action")
        observation = _ordered_vector(
            row["observation.state"],
            state_names,
            f"frame {index} observation.state",
        )
        violations = np.flatnonzero(
            (action < POSITION_LOWER - 1e-6) | (action > POSITION_UPPER + 1e-6)
        )
        if len(violations):
            joints = [REAL_JOINTS[position] for position in violations]
            if not clamp_limits:
                raise ValueError(f"frame {index} action exceeds Piper limits: {joints}")
            action = np.clip(action, POSITION_LOWER, POSITION_UPPER)
            limit_clamped_frames += 1
        frames.append(RealEpisodeFrame(index, timestamp, action, observation))
        prior_timestamp = timestamp
    if not frames:
        raise ValueError("episode contains no frames")
    return RealEpisode(episode_path, tuple(frames), fps, metadata, limit_clamped_frames)


def select_frames(
    episode: RealEpisode, start_index: int, end_index: int | None
) -> tuple[RealEpisodeFrame, ...]:
    if start_index < 0:
        raise ValueError("start-index cannot be negative")
    end = len(episode.frames) if end_index is None else end_index
    if end <= start_index or end > len(episode.frames):
        raise ValueError("invalid replay frame range")
    return episode.frames[start_index:end]


def interpolate_action(
    frames: Sequence[RealEpisodeFrame], elapsed: float
) -> np.ndarray:
    if not frames:
        raise ValueError("cannot interpolate an empty episode")
    relative = float(elapsed)
    if relative <= 0.0:
        return frames[0].action.copy()
    duration = frames[-1].timestamp - frames[0].timestamp
    if relative >= duration:
        return frames[-1].action.copy()
    absolute = frames[0].timestamp + relative
    timestamps = np.fromiter((frame.timestamp for frame in frames), dtype=np.float64)
    upper = int(np.searchsorted(timestamps, absolute, side="right"))
    lower = upper - 1
    interval = frames[upper].timestamp - frames[lower].timestamp
    fraction = (absolute - frames[lower].timestamp) / interval
    return frames[lower].action * (1.0 - fraction) + frames[upper].action * fraction


def episode_summary(episode: RealEpisode) -> str:
    return (
        f"{episode.path}: {len(episode.frames)} frames, "
        f"{episode.duration:.3f}s, {episode.fps:g}Hz, "
        f"contract={episode.metadata.get('contract_version')}"
    )
