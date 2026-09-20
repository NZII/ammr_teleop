"""Receive-time matching helpers for real-robot multimodal recording."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


class SyncMatchError(ValueError):
    """A complete sample cannot be formed at the requested target time."""


@dataclass(frozen=True)
class SyncMatch:
    feedback: dict[str, Any]
    actions: dict[str, Any]
    images: dict[str, Any]
    diagnostics: dict[str, float]


def append_bounded(history: deque, sample: Any, duration: float) -> None:
    """Append a receive-time sample and discard entries outside the window."""
    history.append(sample)
    cutoff = float(sample.received_at) - float(duration)
    while history and float(history[0].received_at) < cutoff:
        history.popleft()


def _nearest(history: Sequence[Any], target: float) -> Any | None:
    if not history:
        return None
    return min(history, key=lambda sample: abs(float(sample.received_at) - target))


def _latest_not_after(history: Sequence[Any], target: float) -> Any | None:
    for sample in reversed(history):
        if float(sample.received_at) <= target:
            return sample
    return None


def match_histories(
    feedback_histories: Mapping[str, Sequence[Any]],
    action_histories: Mapping[str, Sequence[Any]],
    image_histories: Mapping[str, Sequence[Any]],
    *,
    target: float,
    state_max_age: float,
    action_max_age: float,
    camera_max_age: float,
    camera_max_skew: float,
    reject_camera_skew: bool = True,
) -> SyncMatch:
    """Match all streams to one target receive time.

    Feedback and images use the nearest sample. Actions deliberately use the
    newest sample at or before the target so a recorded state never receives a
    command from the future.
    """
    selected_feedback: dict[str, Any] = {}
    selected_actions: dict[str, Any] = {}
    selected_images: dict[str, Any] = {}
    problems: list[str] = []
    diagnostics: dict[str, float] = {}

    for side in ("left", "right"):
        feedback = _nearest(feedback_histories.get(side, ()), target)
        action = _latest_not_after(action_histories.get(side, ()), target)
        if feedback is None:
            problems.append(f"missing={side}_feedback")
        else:
            offset = float(feedback.received_at) - target
            diagnostics[f"ammr.sync.{side}_feedback_offset_ms"] = offset * 1000.0
            if abs(offset) > state_max_age:
                problems.append(
                    f"{side}_feedback offset={offset * 1000.0:.0f}ms"
                )
            selected_feedback[side] = feedback
        if action is None:
            problems.append(f"missing={side}_action_before_target")
        else:
            age = target - float(action.received_at)
            diagnostics[f"ammr.sync.{side}_action_age_ms"] = age * 1000.0
            if age > action_max_age:
                problems.append(f"{side}_action age={age * 1000.0:.0f}ms")
            selected_actions[side] = action

    for name, history in image_histories.items():
        image = _nearest(history, target)
        if image is None:
            problems.append(f"missing={name}")
            continue
        offset = float(image.received_at) - target
        diagnostics[f"ammr.sync.{name}_offset_ms"] = offset * 1000.0
        diagnostics[f"ammr.sync.{name}_source_timestamp"] = float(
            image.source_stamp
        )
        if abs(offset) > camera_max_age:
            problems.append(f"{name} offset={offset * 1000.0:.0f}ms")
        selected_images[name] = image

    if selected_images:
        received = [float(sample.received_at) for sample in selected_images.values()]
        skew = max(received) - min(received)
        diagnostics["ammr.sync.camera_skew_ms"] = skew * 1000.0
        if (
            reject_camera_skew
            and len(selected_images) == len(image_histories)
            and skew > camera_max_skew
        ):
            problems.append(f"camera_skew={skew * 1000.0:.0f}ms")

    if problems:
        raise SyncMatchError("; ".join(problems))
    return SyncMatch(
        selected_feedback, selected_actions, selected_images, diagnostics
    )
