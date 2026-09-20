"""LeRobot-style recorder for the 14-D real-robot contract."""

from __future__ import annotations

import json
import multiprocessing as mp
from pathlib import Path
import queue
import threading
from typing import Mapping

import cv2
import numpy as np

from ammr_teleop.common.data_contract import CONTRACT_VERSION, REAL_JOINTS, validate_vector


CAMERAS = ("head", "left_wrist", "right_wrist")


class RealEpisodeRecorder:
    def __init__(self, root: str | Path, fps: float, task: str) -> None:
        self.root = Path(root).expanduser().resolve()
        self.fps = float(fps)
        self.task = task
        if not np.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError("recording fps must be finite and positive")
        self.data_dir = self.root / "data" / "chunk-000"
        self.video_dir = self.root / "videos" / "chunk-000"
        self.meta_dir = self.root / "meta"
        self.lock = threading.RLock()
        self.active = False
        self.episode_index = -1
        self.rows: list[dict] = []
        self.writers: dict[str, cv2.VideoWriter] = {}
        self.video_paths: dict[str, str] = {}
        self.staged_video_paths: dict[str, Path] = {}
        self.image_shapes: dict[str, list[int]] = {}
        self.diagnostic_names: set[str] = set()
        self.stage_dir: Path | None = None

    def _next_episode_index(self) -> int:
        existing = []
        if self.data_dir.is_dir():
            for path in self.data_dir.glob("episode_*.parquet"):
                try:
                    existing.append(int(path.stem.split("_")[-1]))
                except ValueError:
                    continue
        in_progress = self.root / ".in_progress"
        if in_progress.is_dir():
            for path in in_progress.glob("episode_*"):
                try:
                    existing.append(int(path.name.split("_")[-1]))
                except ValueError:
                    continue
        return max(existing, default=-1) + 1

    def start(self) -> int:
        with self.lock:
            if self.active:
                raise RuntimeError("an episode is already recording")
            self.data_dir.mkdir(parents=True, exist_ok=True)
            self.video_dir.mkdir(parents=True, exist_ok=True)
            self.meta_dir.mkdir(parents=True, exist_ok=True)
            self.episode_index = self._next_episode_index()
            self.rows = []
            self.writers = {}
            self.video_paths = {}
            self.staged_video_paths = {}
            self.image_shapes = {}
            self.diagnostic_names = set()
            self.stage_dir = (
                self.root / ".in_progress" / f"episode_{self.episode_index:06d}"
            )
            self.stage_dir.mkdir(parents=True, exist_ok=False)
            self.active = True
            return self.episode_index

    def _writer(self, name: str, image: np.ndarray) -> cv2.VideoWriter:
        writer = self.writers.get(name)
        if writer is not None:
            return writer
        image_key = f"observation.images.{name}"
        directory = self.video_dir / image_key
        final_path = directory / f"episode_{self.episode_index:06d}.mp4"
        if self.stage_dir is None:
            raise RuntimeError("episode staging directory is unavailable")
        path = self.stage_dir / f"{image_key}.mp4"
        height, width = image.shape[:2]
        writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError(f"failed to open video writer: {path}")
        self.writers[name] = writer
        self.video_paths[name] = str(final_path.relative_to(self.root))
        self.staged_video_paths[name] = path
        self.image_shapes[name] = [height, width, 3]
        return writer

    def add_frame(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        images: Mapping[str, np.ndarray],
        diagnostics: Mapping[str, float] | None = None,
        timestamp: float | None = None,
    ) -> None:
        state = validate_vector(observation)
        command = validate_vector(action)
        missing = [name for name in CAMERAS if name not in images]
        if missing:
            raise ValueError(f"recording frame is missing cameras: {missing}")
        with self.lock:
            if not self.active:
                raise RuntimeError("no active episode")
            frame_index = len(self.rows)
            frame_timestamp = (
                float(frame_index) / self.fps if timestamp is None else float(timestamp)
            )
            if not np.isfinite(frame_timestamp) or frame_timestamp < 0.0:
                raise ValueError("recording timestamp must be finite and non-negative")
            if self.rows and frame_timestamp <= float(self.rows[-1]["timestamp"]):
                raise ValueError("recording timestamps must be strictly increasing")
            row = {
                "observation.state": state.astype(np.float32).tolist(),
                "action": command.astype(np.float32).tolist(),
                "timestamp": frame_timestamp,
                "frame_index": frame_index,
                "episode_index": self.episode_index,
                "index": frame_index,
                "task_index": 0,
                "next.done": False,
            }
            for name, value in (diagnostics or {}).items():
                scalar = float(value)
                if not np.isfinite(scalar):
                    raise ValueError(f"recording diagnostic {name!r} is not finite")
                row[str(name)] = scalar
                self.diagnostic_names.add(str(name))
            for name in CAMERAS:
                image = np.asarray(images[name])
                if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
                    raise ValueError(f"{name} image must be HxWx3 uint8")
                self._writer(name, image).write(image)
                row[f"observation.images.{name}"] = {
                    "path": self.video_paths[name],
                    "timestamp": row["timestamp"],
                }
            self.rows.append(row)

    def stop(self, quality: Mapping | None = None) -> Path:
        with self.lock:
            if not self.active:
                raise RuntimeError("no active episode")
            for writer in self.writers.values():
                writer.release()
            self.writers = {}
            if not self.rows:
                raise RuntimeError("cannot finalize an episode with zero frames")
            if self.rows:
                self.rows[-1]["next.done"] = True
            try:
                import pandas as pd
            except ImportError as exc:
                raise RuntimeError("pandas and pyarrow are required for recording") from exc
            if self.stage_dir is None:
                raise RuntimeError("episode staging directory is unavailable")
            staged_parquet = self.stage_dir / "episode.parquet"
            pd.DataFrame(self.rows).to_parquet(staged_parquet, index=False)
            self._validate_staged_episode(staged_parquet)

            path = self.data_dir / f"episode_{self.episode_index:06d}.parquet"
            if path.exists():
                raise RuntimeError(f"refusing to replace existing episode: {path}")
            self.data_dir.mkdir(parents=True, exist_ok=True)
            staged_parquet.replace(path)
            for name in CAMERAS:
                final_video = self.root / self.video_paths[name]
                if final_video.exists():
                    raise RuntimeError(
                        f"refusing to replace existing episode video: {final_video}"
                    )
                final_video.parent.mkdir(parents=True, exist_ok=True)
                self.staged_video_paths[name].replace(final_video)
            self.stage_dir.rmdir()
            self._write_metadata(dict(quality or {}))
            self.active = False
            return path

    def _validate_staged_episode(self, parquet: Path) -> None:
        import pandas as pd

        rows = len(pd.read_parquet(parquet))
        if rows != len(self.rows):
            raise RuntimeError(
                f"staged parquet rows={rows}, expected={len(self.rows)}"
            )
        for name in CAMERAS:
            path = self.staged_video_paths.get(name)
            if path is None or not path.is_file():
                raise RuntimeError(f"staged episode is missing {name} video")
            capture = cv2.VideoCapture(str(path))
            try:
                frames = int(round(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
            finally:
                capture.release()
            if frames != len(self.rows):
                raise RuntimeError(
                    f"staged {name} video frames={frames}, expected={len(self.rows)}"
                )

    def _write_metadata(self, quality: Mapping) -> None:
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": [14],
                "names": list(REAL_JOINTS),
            },
            "action": {
                "dtype": "float32",
                "shape": [14],
                "names": list(REAL_JOINTS),
            },
        }
        for name, shape in self.image_shapes.items():
            features[f"observation.images.{name}"] = {
                "dtype": "video",
                "shape": shape,
                "names": ["height", "width", "channel"],
                "video_info": {"video.fps": self.fps, "video.codec": "mp4v"},
            }
        episode_files = sorted(self.data_dir.glob("episode_*.parquet"))
        total_frames = 0
        try:
            import pandas as pd

            total_frames = sum(len(pd.read_parquet(path)) for path in episode_files)
        except Exception:
            total_frames = len(self.rows)
        info = {
            "codebase_version": "v2.0",
            "robot_type": "ammr_real",
            "control_source": "xr",
            "contract_version": CONTRACT_VERSION,
            "base_fixed_during_recording": True,
            "camera_names": list(CAMERAS),
            "sync_diagnostics": sorted(self.diagnostic_names),
            "fps": self.fps,
            "total_episodes": len(episode_files),
            "total_frames": total_frames,
            "total_tasks": 1,
            "features": features,
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        }
        self._atomic_write_text(
            self.meta_dir / "info.json",
            json.dumps(info, ensure_ascii=False, indent=2),
        )
        self._atomic_write_text(
            self.meta_dir / "tasks.jsonl",
            json.dumps({"task_index": 0, "task": self.task}, ensure_ascii=False)
            + "\n",
        )
        self._upsert_jsonl(
            self.meta_dir / "episodes.jsonl",
            {
                "episode_index": self.episode_index,
                "tasks": [self.task],
                "length": len(self.rows),
                "quality": dict(quality),
            },
        )
        self._upsert_jsonl(
            self.meta_dir / "episodes_stats.jsonl",
            {
                "episode_index": self.episode_index,
                "stats": self._diagnostic_stats(),
                "quality": dict(quality),
            },
        )

    @staticmethod
    def _atomic_write_text(path: Path, content: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(content, encoding="utf-8")
        temporary.replace(path)

    def _diagnostic_stats(self) -> dict:
        stats = {}
        for name in sorted(self.diagnostic_names):
            values = np.asarray(
                [row[name] for row in self.rows if name in row], dtype=np.float64
            )
            if values.size:
                stats[name] = {
                    "count": int(values.size),
                    "min": float(values.min()),
                    "max": float(values.max()),
                    "mean": float(values.mean()),
                    "std": float(values.std()),
                }
        return stats

    @staticmethod
    def _upsert_jsonl(path: Path, row: dict) -> None:
        rows = []
        if path.is_file():
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
        rows = [item for item in rows if item.get("episode_index") != row["episode_index"]]
        rows.append(row)
        RealEpisodeRecorder._atomic_write_text(
            path,
            "".join(
                json.dumps(item, ensure_ascii=False) + "\n"
                for item in sorted(rows, key=lambda item: item["episode_index"])
            ),
        )

    def preserve_partial(self) -> Path | None:
        """Close encoders but keep staged files after a failed finalization."""
        with self.lock:
            for writer in self.writers.values():
                writer.release()
            self.writers = {}
            self.active = False
            return self.stage_dir

    def close(self) -> Path | None:
        if self.active:
            return self.stop()
        return None


class AsyncEpisodeRecorder:
    """Bounded asynchronous front-end for :class:`RealEpisodeRecorder`.

    ``try_add_frame`` transfers ownership of the supplied arrays to the writer
    thread. Callers must not mutate them after a successful enqueue.
    """

    _STOP = object()

    def __init__(
        self,
        root: str | Path,
        fps: float,
        task: str,
        *,
        queue_size: int = 16,
        stop_timeout: float = 30.0,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("writer queue size must be positive")
        if not np.isfinite(stop_timeout) or stop_timeout <= 0.0:
            raise ValueError("writer stop timeout must be finite and positive")
        self.writer = RealEpisodeRecorder(root, fps, task)
        self.queue_size = int(queue_size)
        self.stop_timeout = float(stop_timeout)
        self.lock = threading.RLock()
        self.frame_queue: queue.Queue | None = None
        self.writer_thread: threading.Thread | None = None
        self.accepting = False
        self.frames_written = 0
        self.producer_drops = 0
        self.writer_error: Exception | None = None

    @property
    def root(self) -> Path:
        return self.writer.root

    @property
    def episode_index(self) -> int:
        return self.writer.episode_index

    @property
    def active(self) -> bool:
        with self.lock:
            return self.accepting

    @property
    def queue_depth(self) -> int:
        with self.lock:
            return self.frame_queue.qsize() if self.frame_queue is not None else 0

    def start(self) -> int:
        with self.lock:
            if self.accepting or self.writer.active:
                raise RuntimeError("an episode is already recording")
            episode = self.writer.start()
            self.frame_queue = queue.Queue(maxsize=self.queue_size)
            self.frames_written = 0
            self.producer_drops = 0
            self.writer_error = None
            self.accepting = True
            self.writer_thread = threading.Thread(
                target=self._writer_loop,
                name=f"ammr-episode-{episode:06d}-writer",
                daemon=True,
            )
            self.writer_thread.start()
            return episode

    def try_add_frame(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        images: Mapping[str, np.ndarray],
        diagnostics: Mapping[str, float] | None = None,
        timestamp: float | None = None,
    ) -> bool:
        with self.lock:
            if not self.accepting or self.frame_queue is None:
                raise RuntimeError("no active episode")
            if self.writer_error is not None:
                raise RuntimeError(f"episode writer failed: {self.writer_error}")
            try:
                self.frame_queue.put_nowait(
                    (observation, action, images, dict(diagnostics or {}), timestamp)
                )
            except queue.Full:
                self.producer_drops += 1
                return False
            return True

    def _writer_loop(self) -> None:
        assert self.frame_queue is not None
        while True:
            item = self.frame_queue.get()
            try:
                if item is self._STOP:
                    return
                observation, action, images, diagnostics, timestamp = item
                self.writer.add_frame(
                    observation, action, images, diagnostics, timestamp=timestamp
                )
                self.frames_written += 1
            except Exception as error:
                self.writer_error = error
            finally:
                self.frame_queue.task_done()

    def stop(self, quality: Mapping | None = None) -> Path:
        with self.lock:
            if not self.accepting or self.frame_queue is None:
                raise RuntimeError("no active episode")
            self.accepting = False
            frame_queue = self.frame_queue
            writer_thread = self.writer_thread
            frame_queue.put(self._STOP)
        assert writer_thread is not None
        writer_thread.join(timeout=self.stop_timeout)
        if writer_thread.is_alive():
            raise RuntimeError(
                f"episode writer did not stop within {self.stop_timeout:.1f}s"
            )
        path = self.writer.stop(quality)
        with self.lock:
            self.frame_queue = None
            self.writer_thread = None
            error = self.writer_error
        if error is not None:
            raise RuntimeError(f"episode writer failed: {error}")
        return path

    def close(self) -> Path | None:
        if self.active:
            return self.stop()
        return None


def _process_writer_main(
    root: str,
    fps: float,
    task: str,
    frame_queue,
    status_queue,
) -> None:
    """Child-process entry point; kept at module scope for spawn mode."""
    recorder = RealEpisodeRecorder(root, fps, task)
    frames_written = 0
    quality: dict = {}
    try:
        episode = recorder.start()
        status_queue.put(("ready", episode))
        while True:
            item = frame_queue.get()
            if item is None:
                break
            if isinstance(item, dict) and item.get("type") == "stop":
                quality = dict(item.get("quality") or {})
                break
            observation, action, images, diagnostics, timestamp = item
            recorder.add_frame(
                observation, action, images, diagnostics, timestamp=timestamp
            )
            frames_written += 1
        path = recorder.stop(quality)
        status_queue.put(("stopped", str(path), frames_written, None))
    except Exception as error:
        partial = recorder.preserve_partial() if recorder.active else recorder.stage_dir
        partial_path = str(partial) if partial is not None else None
        status_queue.put(
            ("error", partial_path, frames_written, f"{type(error).__name__}: {error}")
        )


class ProcessEpisodeRecorder:
    """Run all OpenCV video encoding and Parquet finalization in a child process.

    The ROS process only validates/snapshots complete frames and enqueues them.
    A spawn context is used because forking a live ROS 2/DDS process is unsafe.
    """

    def __init__(
        self,
        root: str | Path,
        fps: float,
        task: str,
        *,
        queue_size: int = 16,
        startup_timeout: float = 10.0,
        stop_timeout: float = 30.0,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("writer queue size must be positive")
        for name, value in (
            ("startup timeout", startup_timeout),
            ("stop timeout", stop_timeout),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"writer {name} must be finite and positive")
        self.root = Path(root).expanduser().resolve()
        self.fps = float(fps)
        self.task = task
        self.queue_size = int(queue_size)
        self.startup_timeout = float(startup_timeout)
        self.stop_timeout = float(stop_timeout)
        self.context = mp.get_context("spawn")
        self.lock = threading.RLock()
        self.frame_queue = None
        self.status_queue = None
        self.process = None
        self.accepting = False
        self.episode_index = -1
        self.frames_written = 0
        self.frames_accepted = 0
        self.producer_drops = 0
        self.writer_error: str | None = None

    @property
    def active(self) -> bool:
        with self.lock:
            return self.accepting

    @property
    def queue_depth(self) -> int:
        with self.lock:
            if self.frame_queue is None:
                return 0
            try:
                return self.frame_queue.qsize()
            except (NotImplementedError, OSError):
                return -1

    def start(self) -> int:
        with self.lock:
            if self.accepting or (
                self.process is not None and self.process.is_alive()
            ):
                raise RuntimeError("an episode is already recording")
            self.root.mkdir(parents=True, exist_ok=True)
            self.frame_queue = self.context.Queue(maxsize=self.queue_size)
            self.status_queue = self.context.Queue(maxsize=4)
            self.frames_written = 0
            self.frames_accepted = 0
            self.producer_drops = 0
            self.writer_error = None
            self.process = self.context.Process(
                target=_process_writer_main,
                args=(
                    str(self.root),
                    self.fps,
                    self.task,
                    self.frame_queue,
                    self.status_queue,
                ),
                name="ammr-episode-writer",
                daemon=False,
            )
            self.process.start()
            status_queue = self.status_queue
        try:
            message = status_queue.get(timeout=self.startup_timeout)
        except queue.Empty as error:
            self._terminate_writer()
            raise RuntimeError(
                f"episode writer did not start within {self.startup_timeout:.1f}s"
            ) from error
        if not message or message[0] != "ready":
            self._terminate_writer()
            detail = message[-1] if message else "unknown child-process error"
            raise RuntimeError(f"episode writer failed during startup: {detail}")
        with self.lock:
            self.episode_index = int(message[1])
            self.accepting = True
            return self.episode_index

    def try_add_frame(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        images: Mapping[str, np.ndarray],
        diagnostics: Mapping[str, float] | None = None,
        timestamp: float | None = None,
    ) -> bool:
        with self.lock:
            if not self.accepting or self.frame_queue is None:
                raise RuntimeError("no active episode")
            if self.process is None or not self.process.is_alive():
                self.writer_error = "episode writer process exited unexpectedly"
                raise RuntimeError(self.writer_error)
            try:
                self.frame_queue.put_nowait(
                    (observation, action, images, dict(diagnostics or {}), timestamp)
                )
            except queue.Full:
                self.producer_drops += 1
                return False
            self.frames_accepted += 1
            return True

    def stop(self, quality: Mapping | None = None) -> Path:
        with self.lock:
            if not self.accepting or self.frame_queue is None:
                raise RuntimeError("no active episode")
            self.accepting = False
            frame_queue = self.frame_queue
            status_queue = self.status_queue
            process = self.process
        assert status_queue is not None
        assert process is not None
        try:
            frame_queue.put(
                {"type": "stop", "quality": dict(quality or {})},
                timeout=self.stop_timeout,
            )
            message = status_queue.get(timeout=self.stop_timeout)
        except queue.Empty as error:
            self._terminate_writer()
            raise RuntimeError(
                f"episode writer did not finish within {self.stop_timeout:.1f}s"
            ) from error
        except queue.Full as error:
            self._terminate_writer()
            raise RuntimeError("episode writer queue could not be stopped") from error
        process.join(timeout=2.0)
        if process.is_alive():
            self._terminate_writer()
            raise RuntimeError("episode writer process remained alive after saving")

        kind, path_value, frames_written, detail = message
        with self.lock:
            self.frames_written = int(frames_written)
            self.writer_error = detail
            self._close_queues()
        if kind != "stopped" or detail is not None or path_value is None:
            raise RuntimeError(f"episode writer failed: {detail or kind}")
        return Path(path_value)

    def _close_queues(self) -> None:
        for current in (self.frame_queue, self.status_queue):
            if current is not None:
                current.close()
                current.join_thread()
        self.frame_queue = None
        self.status_queue = None
        self.process = None

    def _terminate_writer(self) -> None:
        with self.lock:
            process = self.process
            self.accepting = False
        if process is not None and process.is_alive():
            process.terminate()
            process.join(timeout=2.0)
        with self.lock:
            self._close_queues()

    def close(self) -> Path | None:
        if self.active:
            return self.stop()
        return None
