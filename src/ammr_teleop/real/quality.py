"""Episode-level quality assessment for AMMR real-robot recordings."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping


@dataclass(frozen=True)
class QualityThresholds:
    min_frames: int = 30
    min_actual_fps: float = 20.0
    max_rejected_ratio: float = 0.30
    max_duplicate_ratio: float = 0.30
    max_target_skip_ratio: float = 0.05
    max_producer_drops: int = 0


def assess_episode_quality(
    metrics: Mapping[str, float | int], thresholds: QualityThresholds
) -> dict:
    """Return a serializable quality result without deleting bad episodes."""
    checked = {
        "frames": int(metrics.get("frames", 0)),
        "actual_fps": float(metrics.get("actual_fps", 0.0)),
        "rejected_ratio": float(metrics.get("rejected_ratio", 0.0)),
        "duplicate_ratio": float(metrics.get("duplicate_ratio", 0.0)),
        "target_skip_ratio": float(metrics.get("target_skip_ratio", 0.0)),
        "producer_drops": int(metrics.get("producer_drops", 0)),
    }
    if any(
        not math.isfinite(value)
        for name, value in checked.items()
        if name not in ("frames", "producer_drops")
    ):
        raise ValueError("episode quality metrics must be finite")

    issues: list[str] = []
    if checked["frames"] < thresholds.min_frames:
        issues.append(
            f"frames={checked['frames']} < {thresholds.min_frames}"
        )
    if checked["actual_fps"] < thresholds.min_actual_fps:
        issues.append(
            f"actual_fps={checked['actual_fps']:.2f} < "
            f"{thresholds.min_actual_fps:.2f}"
        )
    for metric, limit in (
        ("rejected_ratio", thresholds.max_rejected_ratio),
        ("duplicate_ratio", thresholds.max_duplicate_ratio),
        ("target_skip_ratio", thresholds.max_target_skip_ratio),
    ):
        if checked[metric] > limit:
            issues.append(f"{metric}={checked[metric]:.3f} > {limit:.3f}")
    if checked["producer_drops"] > thresholds.max_producer_drops:
        issues.append(
            f"producer_drops={checked['producer_drops']} > "
            f"{thresholds.max_producer_drops}"
        )
    return {
        "valid": not issues,
        "issues": issues,
        "metrics": dict(metrics),
        "thresholds": asdict(thresholds),
    }
