"""Episode-level quality assessment for AMMR real-robot recordings."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping


@dataclass(frozen=True)
class QualityThresholds:
    min_frames: int = 30
    min_actual_fps: float = 20.0
    max_producer_drops: int = 0


def episode_quality_metrics(
    *, frames: int, duration_s: float, producer_drops: int
) -> dict[str, float | int]:
    """Summarize stored frames and writer backpressure without sync penalties."""
    attempted_samples = frames + producer_drops
    denominator = max(attempted_samples, 1)
    return {
        "frames": frames,
        "duration_s": duration_s,
        "actual_fps": frames / max(duration_s, 1e-9),
        "attempted_samples": attempted_samples,
        "producer_drops": producer_drops,
        "producer_drop_ratio": producer_drops / denominator,
    }


def assess_episode_quality(
    metrics: Mapping[str, float | int], thresholds: QualityThresholds
) -> dict:
    """Return a serializable quality result without deleting bad episodes."""
    frames = int(metrics.get("frames", 0))
    actual_fps = float(metrics.get("actual_fps", 0.0))
    producer_drops = int(metrics.get("producer_drops", 0))
    if not math.isfinite(actual_fps):
        raise ValueError("episode quality metrics must be finite")

    issues: list[str] = []
    if frames < thresholds.min_frames:
        issues.append(f"frames={frames} < {thresholds.min_frames}")
    if actual_fps < thresholds.min_actual_fps:
        issues.append(
            f"actual_fps={actual_fps:.2f} < {thresholds.min_actual_fps:.2f}"
        )
    if producer_drops > thresholds.max_producer_drops:
        issues.append(
            f"producer_drops={producer_drops} > {thresholds.max_producer_drops}"
        )
    return {
        "valid": not issues,
        "issues": issues,
        "metrics": dict(metrics),
        "thresholds": asdict(thresholds),
    }
