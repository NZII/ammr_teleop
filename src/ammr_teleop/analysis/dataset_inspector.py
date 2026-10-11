"""Inspect many AMMR 14-D episodes in one interactive dashboard."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
from typing import Any, Sequence

import numpy as np

from ammr_teleop.common.data_contract import REAL_JOINTS
from ammr_teleop.real.episode import RealEpisode, read_real_episode


@dataclass(frozen=True)
class EpisodeMetrics:
    episode: str
    path: str
    valid: bool
    error: str
    frames: int
    duration_s: float
    capture_duration_s: float
    nominal_hz: float
    timeline_hz: float
    actual_hz: float
    actual_hz_source: str
    legacy_capture_gap_ms: float | None
    camera_reuse_ratio: float
    camera_reuse_summary: str
    camera_sequence_gaps: int
    max_source_age_ms: float
    source_age_summary: str
    producer_drops: int
    legacy_sync_summary: str
    quality_valid: bool | None
    quality_issues: str
    mean_rmse: float
    max_error: float
    max_action_step: float
    joint_rmse: tuple[float, ...]
    joint_max_error: tuple[float, ...]
    joint_max_action_step: tuple[float, ...]

    def table_record(self) -> dict[str, Any]:
        return {
            "episode": self.episode,
            "valid": "yes" if self.valid else "no",
            "frames": self.frames,
            "duration_s": round(self.duration_s, 3),
            "capture_duration_s": round(self.capture_duration_s, 3),
            "nominal_hz": round(self.nominal_hz, 2),
            "timeline_hz": round(self.timeline_hz, 2),
            "actual_hz": round(self.actual_hz, 2),
            "actual_hz_source": self.actual_hz_source,
            "legacy_capture_gap_ms": (
                round(self.legacy_capture_gap_ms, 2)
                if self.legacy_capture_gap_ms is not None else None
            ),
            "camera_reuse_ratio": round(self.camera_reuse_ratio, 4),
            "camera_reuse_summary": self.camera_reuse_summary,
            "camera_sequence_gaps": self.camera_sequence_gaps,
            "max_source_age_ms": round(self.max_source_age_ms, 2),
            "source_age_summary": self.source_age_summary,
            "producer_drops": self.producer_drops,
            "legacy_sync_summary": self.legacy_sync_summary,
            "quality_valid": (
                "unknown"
                if self.quality_valid is None
                else "yes" if self.quality_valid else "no"
            ),
            "quality_issues": self.quality_issues,
            "mean_rmse": round(self.mean_rmse, 6),
            "max_error": round(self.max_error, 6),
            "max_action_step": round(self.max_action_step, 6),
            "error": self.error,
            "path": self.path,
        }


def episode_paths(dataset_path: str | Path) -> tuple[Path, ...]:
    root = Path(dataset_path).expanduser().resolve()
    if root.is_file():
        if root.suffix.lower() != ".parquet":
            raise ValueError(f"expected a parquet episode or dataset directory: {root}")
        return (root,)
    if not root.is_dir():
        raise FileNotFoundError(f"dataset does not exist: {root}")
    paths = tuple(sorted(root.glob("data/chunk-*/episode_*.parquet")))
    if not paths and root.name == "data":
        paths = tuple(sorted(root.glob("chunk-*/episode_*.parquet")))
    if not paths:
        paths = tuple(sorted(root.glob("chunk-*/episode_*.parquet")))
    if not paths:
        raise ValueError(f"no episode parquet files found under: {root}")
    return paths


def _finite_metric(values: dict[str, Any], name: str, default: float = 0.0) -> float:
    try:
        value = float(values.get(name, default))
    except (TypeError, ValueError):
        return default
    return value if np.isfinite(value) else default


def _dataset_root(episode_path: Path) -> Path:
    if (
        episode_path.parent.name.startswith("chunk-")
        and episode_path.parent.parent.name == "data"
    ):
        return episode_path.parent.parent.parent
    return episode_path.parent


def _episode_index(path: Path) -> int | None:
    try:
        return int(path.stem.rsplit("_", 1)[-1])
    except ValueError:
        return None


def read_quality_records(dataset_root: Path) -> dict[int, dict[str, Any]]:
    path = dataset_root / "meta" / "episodes.jsonl"
    if not path.is_file():
        return {}
    records: dict[int, dict[str, Any]] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
            index = int(record["episode_index"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"invalid {path} line {line_number}: {error}") from error
        quality = record.get("quality")
        if isinstance(quality, dict):
            records[index] = quality
    return records


def read_target_timestamps(path: Path) -> np.ndarray | None:
    try:
        import pandas as pd

        table = pd.read_parquet(path, columns=["ammr.sync.target_timestamp"])
    except Exception:
        return None
    values = np.asarray(table["ammr.sync.target_timestamp"], dtype=np.float64)
    if (
        len(values) < 2
        or not np.all(np.isfinite(values))
        or np.any(np.diff(values) <= 0.0)
    ):
        return None
    return values


def read_capture_diagnostics(
    path: Path,
) -> tuple[float, str, int, float, str]:
    """Summarize per-camera reuse, sequence gaps, and per-source age."""
    try:
        import pyarrow.parquet as pq

        schema = pq.read_schema(path)
        reused = [
            column for column in schema.names
            if column.startswith("ammr.capture.camera_") and column.endswith("_reused")
        ]
        sequences = [
            column for column in schema.names
            if column.startswith("ammr.capture.camera_") and column.endswith("_sequence")
        ]
        ages = [
            column for column in schema.names
            if column.startswith("ammr.capture.") and column.endswith("_age_ms")
        ]
        selected = reused + sequences + ages
        if not selected:
            return 0.0, "", 0, 0.0, ""
        table = pq.read_table(path, columns=selected).to_pandas()
    except Exception:
        return 0.0, "", 0, 0.0, ""
    reuse_ratio = (
        float(np.concatenate([table[column].to_numpy(dtype=np.float64) for column in reused]).mean())
        if reused else 0.0
    )
    reuse_by_camera = {
        column.removeprefix("ammr.capture.camera_").removesuffix("_reused"):
        float(table[column].to_numpy(dtype=np.float64).mean())
        for column in reused
    }
    ages_by_source = {
        column.removeprefix("ammr.capture.").removesuffix("_age_ms"):
        float(table[column].max())
        for column in ages
    }
    sequence_gaps = 0
    for column in sequences:
        values = table[column].to_numpy(dtype=np.float64)
        differences = np.diff(values)
        sequence_gaps += int(np.maximum(differences - 1.0, 0.0).sum())
    return (
        reuse_ratio,
        ", ".join(f"{name}={value:.3f}" for name, value in sorted(reuse_by_camera.items())),
        sequence_gaps,
        max(ages_by_source.values(), default=0.0),
        ", ".join(f"{name}={value:.1f}ms" for name, value in sorted(ages_by_source.items())),
    )


def calculate_metrics(
    episode: RealEpisode,
    quality: dict[str, Any] | None = None,
    target_timestamps: np.ndarray | None = None,
) -> EpisodeMetrics:
    timestamps = np.asarray([frame.timestamp for frame in episode.frames])
    actions = np.stack([frame.action for frame in episode.frames])
    states = np.stack([frame.observation for frame in episode.frames])
    errors = actions - states
    joint_rmse = np.sqrt(np.mean(errors * errors, axis=0))
    joint_max_error = np.max(np.abs(errors), axis=0)
    if len(actions) > 1:
        joint_max_step = np.max(np.abs(np.diff(actions, axis=0)), axis=0)
        timeline_hz = float((len(timestamps) - 1) / (timestamps[-1] - timestamps[0]))
    else:
        joint_max_step = np.zeros(len(REAL_JOINTS), dtype=np.float64)
        timeline_hz = 0.0
    quality = quality if isinstance(quality, dict) else None
    quality_metrics = quality.get("metrics", {}) if quality else {}
    (
        camera_reuse_ratio,
        camera_reuse_summary,
        camera_sequence_gaps,
        max_source_age_ms,
        source_age_summary,
    ) = read_capture_diagnostics(Path(episode.path))
    target_timestamps_valid = (
        target_timestamps is not None and len(target_timestamps) == len(timestamps)
    )
    legacy_capture_gap_ms = (
        float(np.max(np.diff(target_timestamps)) * 1000.0)
        if target_timestamps_valid else None
    )
    recorded_duration = _finite_metric(quality_metrics, "duration_s")
    if recorded_duration > 0.0:
        capture_duration_s = recorded_duration
    elif target_timestamps_valid:
        capture_duration_s = float(target_timestamps[-1] - target_timestamps[0])
    else:
        capture_duration_s = float(episode.duration)
    recorded_actual_hz = _finite_metric(quality_metrics, "actual_fps")
    if recorded_actual_hz > 0.0:
        actual_hz = recorded_actual_hz
        actual_hz_source = "episodes.jsonl quality"
    elif target_timestamps_valid:
        actual_hz = float(
            (len(target_timestamps) - 1)
            / (target_timestamps[-1] - target_timestamps[0])
        )
        actual_hz_source = "ammr.sync.target_timestamp"
    else:
        actual_hz = timeline_hz
        actual_hz_source = "parquet timestamp fallback"
    quality_valid = (
        bool(quality["valid"])
        if quality is not None and isinstance(quality.get("valid"), bool)
        else None
    )
    issues = quality.get("issues", []) if quality else []
    quality_issues = "; ".join(str(issue) for issue in issues)
    return EpisodeMetrics(
        episode=episode.path.stem,
        path=str(episode.path),
        valid=True,
        error="",
        frames=len(episode.frames),
        duration_s=float(episode.duration),
        capture_duration_s=capture_duration_s,
        nominal_hz=float(episode.fps),
        timeline_hz=timeline_hz,
        actual_hz=actual_hz,
        actual_hz_source=actual_hz_source,
        legacy_capture_gap_ms=legacy_capture_gap_ms,
        camera_reuse_ratio=camera_reuse_ratio,
        camera_reuse_summary=camera_reuse_summary,
        camera_sequence_gaps=camera_sequence_gaps,
        max_source_age_ms=max_source_age_ms,
        source_age_summary=source_age_summary,
        producer_drops=int(_finite_metric(quality_metrics, "producer_drops")),
        legacy_sync_summary=(
            "rejected={:.3f}, reused/duplicate={:.3f}, target skips={:.3f}".format(
                _finite_metric(quality_metrics, "rejected_ratio"),
                _finite_metric(quality_metrics, "duplicate_ratio"),
                _finite_metric(quality_metrics, "target_skip_ratio"),
            )
            if any(name in quality_metrics for name in (
                "rejected_ratio", "duplicate_ratio", "target_skip_ratio"
            )) else ""
        ),
        quality_valid=quality_valid,
        quality_issues=quality_issues,
        mean_rmse=float(np.mean(joint_rmse)),
        max_error=float(np.max(joint_max_error)),
        max_action_step=float(np.max(joint_max_step)),
        joint_rmse=tuple(float(value) for value in joint_rmse),
        joint_max_error=tuple(float(value) for value in joint_max_error),
        joint_max_action_step=tuple(float(value) for value in joint_max_step),
    )


def invalid_metrics(path: Path, error: Exception) -> EpisodeMetrics:
    zeros = (0.0,) * len(REAL_JOINTS)
    return EpisodeMetrics(
        episode=path.stem,
        path=str(path),
        valid=False,
        error=f"{type(error).__name__}: {error}",
        frames=0,
        duration_s=0.0,
        capture_duration_s=0.0,
        nominal_hz=0.0,
        timeline_hz=0.0,
        actual_hz=0.0,
        actual_hz_source="unavailable",
        legacy_capture_gap_ms=None,
        camera_reuse_ratio=0.0,
        camera_reuse_summary="",
        camera_sequence_gaps=0,
        max_source_age_ms=0.0,
        source_age_summary="",
        producer_drops=0,
        legacy_sync_summary="",
        quality_valid=None,
        quality_issues="",
        mean_rmse=0.0,
        max_error=0.0,
        max_action_step=0.0,
        joint_rmse=zeros,
        joint_max_error=zeros,
        joint_max_action_step=zeros,
    )


def scan_dataset(dataset_path: str | Path) -> tuple[EpisodeMetrics, ...]:
    results: list[EpisodeMetrics] = []
    quality_cache: dict[Path, dict[int, dict[str, Any]]] = {}
    for path in episode_paths(dataset_path):
        try:
            root = _dataset_root(path)
            if root not in quality_cache:
                quality_cache[root] = read_quality_records(root)
            index = _episode_index(path)
            quality = quality_cache[root].get(index) if index is not None else None
            results.append(
                calculate_metrics(
                    read_real_episode(path),
                    quality=quality,
                    target_timestamps=read_target_timestamps(path),
                )
            )
        except Exception as error:
            results.append(invalid_metrics(path, error))
    return tuple(results)


def print_summary(metrics: Sequence[EpisodeMetrics]) -> None:
    valid = [item for item in metrics if item.valid]
    invalid = [item for item in metrics if not item.valid]
    quality_valid = [item for item in valid if item.quality_valid is True]
    quality_invalid = [item for item in valid if item.quality_valid is False]
    quality_unknown = [item for item in valid if item.quality_valid is None]
    frames = sum(item.frames for item in valid)
    print(
        f"episodes={len(metrics)}, contract_valid={len(valid)}, "
        f"contract_invalid={len(invalid)}, frames={frames}"
    )
    print(
        f"quality_valid={len(quality_valid)}, quality_invalid={len(quality_invalid)}, "
        f"quality_unknown={len(quality_unknown)}"
    )
    for item in invalid:
        print(f"INVALID {item.episode}: {item.error}")
    if valid:
        worst = max(valid, key=lambda item: item.max_action_step)
        print(
            f"largest action step: {worst.episode}={worst.max_action_step:.6f}; "
            f"mean tracking RMSE={np.mean([item.mean_rmse for item in valid]):.6f}"
        )


def write_summary(metrics: Sequence[EpisodeMetrics], output: str | Path) -> Path:
    path = Path(output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "joint_names": list(REAL_JOINTS),
        "episodes": [asdict(item) for item in metrics],
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    temporary.replace(path)
    return path


def _episode_arrays(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    episode = read_real_episode(path)
    target_timestamps = read_target_timestamps(Path(path))
    if target_timestamps is not None and len(target_timestamps) == len(episode.frames):
        time_s = target_timestamps - target_timestamps[0]
    else:
        start = episode.frames[0].timestamp
        time_s = np.asarray([frame.timestamp - start for frame in episode.frames])
    states = np.stack([frame.observation for frame in episode.frames])
    actions = np.stack([frame.action for frame in episode.frames])
    return time_s, states, actions


def create_app(metrics: Sequence[EpisodeMetrics]):
    try:
        from dash import Dash, Input, Output, dash_table, dcc, html
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as error:
        raise RuntimeError(
            "Dashboard dependencies are missing; install requirements-analysis.txt"
        ) from error

    valid = [item for item in metrics if item.valid]
    if not valid:
        raise RuntimeError("no valid episodes are available for visualization")
    by_path = {item.path: item for item in valid}
    table_rows = [item.table_record() for item in metrics]
    columns = [
        {"name": label, "id": key, "type": kind}
        for label, key, kind in (
            ("Episode", "episode", "text"),
            ("Contract valid", "valid", "text"),
            ("Quality valid", "quality_valid", "text"),
            ("Frames", "frames", "numeric"),
            ("Timeline duration (s)", "duration_s", "numeric"),
            ("Capture duration (s)", "capture_duration_s", "numeric"),
            ("Nominal Hz", "nominal_hz", "numeric"),
            ("Timeline Hz", "timeline_hz", "numeric"),
            ("Actual capture Hz", "actual_hz", "numeric"),
            ("Actual Hz source", "actual_hz_source", "text"),
            ("Legacy capture gap (ms)", "legacy_capture_gap_ms", "numeric"),
            ("Camera frame reuse ratio", "camera_reuse_ratio", "numeric"),
            ("Reuse by camera", "camera_reuse_summary", "text"),
            ("Camera sequence gaps", "camera_sequence_gaps", "numeric"),
            ("Max source age (ms)", "max_source_age_ms", "numeric"),
            ("Source ages (ms)", "source_age_summary", "text"),
            ("Writer drops", "producer_drops", "numeric"),
            ("Legacy sync metrics", "legacy_sync_summary", "text"),
            ("Mean RMSE", "mean_rmse", "numeric"),
            ("Max error", "max_error", "numeric"),
            ("Max action step", "max_action_step", "numeric"),
            ("Quality issues", "quality_issues", "text"),
            ("Error", "error", "text"),
        )
    ]

    heatmap = go.Figure(
        data=go.Heatmap(
            z=[item.joint_max_action_step for item in valid],
            x=list(REAL_JOINTS),
            y=[item.episode for item in valid],
            colorscale="Viridis",
            colorbar={"title": "max |Δaction|"},
            hovertemplate="%{y}<br>%{x}<br>max step=%{z:.6f}<extra></extra>",
        )
    )
    heatmap.update_layout(
        title="Episode × joint action-step heatmap",
        height=max(360, 28 * len(valid) + 150),
        margin={"l": 110, "r": 40, "t": 60, "b": 100},
    )

    app = Dash(__name__)
    app.title = "AMMR Dataset Inspector"
    app.layout = html.Div(
        [
            html.H1("AMMR Dataset Inspector"),
            html.P(
                f"{len(metrics)} episodes · {len(valid)} contract valid · "
                f"{sum(item.quality_valid is True for item in valid)} quality valid · "
                f"{sum(item.frames for item in valid)} frames"
            ),
            dash_table.DataTable(
                id="episode-table",
                data=table_rows,
                columns=columns,
                sort_action="native",
                filter_action="native",
                page_action="native",
                page_size=15,
                style_table={"overflowX": "auto"},
                style_cell={
                    "fontFamily": "sans-serif",
                    "fontSize": 13,
                    "padding": "6px",
                    "textAlign": "right",
                },
                style_cell_conditional=[
                    {"if": {"column_id": "episode"}, "textAlign": "left"},
                    {"if": {"column_id": "actual_hz_source"}, "textAlign": "left"},
                    {"if": {"column_id": "legacy_sync_summary"}, "textAlign": "left"},
                    {"if": {"column_id": "camera_reuse_summary"}, "textAlign": "left"},
                    {"if": {"column_id": "source_age_summary"}, "textAlign": "left"},
                    {"if": {"column_id": "quality_issues"}, "textAlign": "left"},
                    {"if": {"column_id": "error"}, "textAlign": "left"},
                ],
                style_data_conditional=[
                    {
                        "if": {"filter_query": "{valid} = no"},
                        "backgroundColor": "#ffd7d7",
                    },
                    {
                        "if": {"filter_query": "{quality_valid} = no"},
                        "backgroundColor": "#fff0c2",
                    },
                ],
            ),
            dcc.Graph(figure=heatmap),
            html.H2("Episode detail"),
            dcc.Dropdown(
                id="episode-select",
                options=[{"label": item.episode, "value": item.path} for item in valid],
                value=valid[0].path,
                clearable=False,
            ),
            html.Pre(id="episode-summary", style={"whiteSpace": "pre-wrap"}),
            dcc.Loading(dcc.Graph(id="joint-curves"), type="circle"),
            dcc.Loading(dcc.Graph(id="error-curves"), type="circle"),
        ],
        style={"maxWidth": "1700px", "margin": "0 auto", "padding": "20px"},
    )

    @app.callback(
        Output("joint-curves", "figure"),
        Output("error-curves", "figure"),
        Output("episode-summary", "children"),
        Input("episode-select", "value"),
    )
    def update_episode(path: str):
        item = by_path[path]
        time_s, states, actions = _episode_arrays(path)
        figure = make_subplots(
            rows=7,
            cols=2,
            shared_xaxes=True,
            subplot_titles=[
                name
                for pair in zip(REAL_JOINTS[:7], REAL_JOINTS[7:])
                for name in pair
            ],
            vertical_spacing=0.025,
        )
        for index, name in enumerate(REAL_JOINTS):
            row = index + 1 if index < 7 else index - 6
            column = 1 if index < 7 else 2
            figure.add_trace(
                go.Scattergl(
                    x=time_s,
                    y=states[:, index],
                    name="state",
                    legendgroup="state",
                    showlegend=index == 0,
                    line={"color": "#1f77b4", "width": 1.5},
                    hovertemplate=f"{name} state=%{{y:.6f}}<br>t=%{{x:.3f}}s<extra></extra>",
                ),
                row=row,
                col=column,
            )
            figure.add_trace(
                go.Scattergl(
                    x=time_s,
                    y=actions[:, index],
                    name="action",
                    legendgroup="action",
                    showlegend=index == 0,
                    line={"color": "#ff7f0e", "width": 1.2, "dash": "dot"},
                    hovertemplate=f"{name} action=%{{y:.6f}}<br>t=%{{x:.3f}}s<extra></extra>",
                ),
                row=row,
                col=column,
            )
        figure.update_xaxes(title_text="time (s)", row=7, col=1)
        figure.update_xaxes(title_text="time (s)", row=7, col=2)
        figure.update_layout(
            title=f"{item.episode}: state and action",
            height=1500,
            hovermode="x unified",
            margin={"l": 60, "r": 30, "t": 80, "b": 50},
        )

        error_figure = go.Figure()
        error = actions - states
        for index, name in enumerate(REAL_JOINTS):
            error_figure.add_trace(
                go.Scattergl(
                    x=time_s,
                    y=error[:, index],
                    name=name,
                    line={"width": 1},
                )
            )
        error_figure.update_layout(
            title="action - state",
            xaxis_title="time (s)",
            yaxis_title="error (rad; grippers in m)",
            height=520,
            hovermode="x unified",
        )
        summary = (
            f"path: {item.path}\n"
            f"frames: {item.frames}    timeline/capture duration: "
            f"{item.duration_s:.3f} / {item.capture_duration_s:.3f}s\n"
            f"nominal/timeline/actual: {item.nominal_hz:.3f} / "
            f"{item.timeline_hz:.3f} / {item.actual_hz:.3f}Hz\n"
            f"camera reuse: {item.camera_reuse_ratio:.3f}    "
            f"reuse by camera: {item.camera_reuse_summary or 'not present'}\n"
            f"camera sequence gaps: {item.camera_sequence_gaps}    "
            f"max source age: {item.max_source_age_ms:.1f}ms    "
            f"writer drops: {item.producer_drops}\n"
            f"source ages: {item.source_age_summary or 'not present'}\n"
            f"actual Hz source: {item.actual_hz_source}    "
            f"quality: {item.table_record()['quality_valid']}\n"
            f"legacy sync metrics: {item.legacy_sync_summary or 'not present'}\n"
            f"quality issues: {item.quality_issues or 'none'}\n"
            f"mean RMSE: {item.mean_rmse:.6f}    max error: {item.max_error:.6f}    "
            f"max action step: {item.max_action_step:.6f}"
        )
        return figure, error_figure, summary

    return app


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8050)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--validate-all", action="store_true")
    parser.add_argument("--no-web", action="store_true")
    parser.add_argument("--summary-json")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    metrics = scan_dataset(args.dataset_path)
    print_summary(metrics)
    if args.summary_json:
        print(f"summary={write_summary(metrics, args.summary_json)}")
    if args.no_web:
        return 1 if args.validate_all and any(not item.valid for item in metrics) else 0
    app = create_app(metrics)
    print(f"dashboard=http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=args.debug)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"[AMMR inspector] {error}", file=sys.stderr)
        raise SystemExit(2)
