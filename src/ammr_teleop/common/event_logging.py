"""Low-overhead, structured operator-event logging for AMMR teleoperation."""

from __future__ import annotations

import json
import logging
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
import os
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any

try:
    from rich.logging import RichHandler
except ImportError:  # pragma: no cover - the teleoperation environment has Rich
    RichHandler = None


class _MillisecondFormatter(logging.Formatter):
    default_msec_format = "%s.%03d"


def _field_value(value: Any) -> str:
    if value is None:
        return "none"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.3f}"
    text = str(value)
    if not text or any(character.isspace() for character in text):
        return json.dumps(text, ensure_ascii=False)
    return text


class AmmrEventLogger:
    """Asynchronous console and rotating-file logger for sparse events."""

    def __init__(self, mode: str, log_dir: str | Path | None = None) -> None:
        self.mode = str(mode).upper()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_id = f"{timestamp}_p{os.getpid()}"
        requested_dir = Path(
            log_dir
            or os.environ.get("AMMR_EVENT_LOG_DIR")
            or os.environ.get("ROS_LOG_DIR")
            or "/tmp/ammr_xr_logs"
        ).expanduser()
        try:
            requested_dir.mkdir(parents=True, exist_ok=True)
            self.log_dir = requested_dir.resolve()
        except OSError:
            self.log_dir = Path("/tmp/ammr_xr_logs").resolve()
            self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f"ammr_{self.mode.lower()}_{self.session_id}.log"

        file_formatter = _MillisecondFormatter(
            "%(asctime)s %(levelname)-8s "
            "[AMMR][%(teleop_mode)s][%(event_code)s] %(message)s "
            "%(filename)s:%(lineno)d",
            datefmt="%H:%M:%S",
        )
        if RichHandler is not None:
            console = RichHandler(
                show_time=True,
                log_time_format="%H:%M:%S.%f",
                omit_repeated_times=False,
                show_level=True,
                show_path=True,
                enable_link_path=False,
                rich_tracebacks=True,
                markup=False,
            )
            console.setFormatter(
                logging.Formatter(
                    "[%(teleop_mode)s][%(event_code)s] %(message)s"
                )
            )
        else:
            console = logging.StreamHandler(sys.stdout)
            console.setFormatter(file_formatter)
        file_handler = RotatingFileHandler(
            self.path,
            maxBytes=20 * 1024 * 1024,
            backupCount=5,
            encoding="utf-8",
        )
        file_handler.setFormatter(file_formatter)

        self._queue: queue.SimpleQueue[logging.LogRecord] = queue.SimpleQueue()
        self._listener = QueueListener(
            self._queue, console, file_handler, respect_handler_level=True
        )
        self._outputs = (console, file_handler)
        self._logger = logging.getLogger(
            f"ammr.teleop.event.{self.mode.lower()}.{self.session_id}"
        )
        self._logger.handlers.clear()
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self._logger.addHandler(QueueHandler(self._queue))
        self._closed = False
        self._close_lock = threading.Lock()
        self.started_monotonic = time.monotonic()
        self._listener.start()

    def log(
        self,
        level: int,
        event_code: str,
        message: str = "",
        **fields: Any,
    ) -> None:
        if self._closed:
            return
        suffix = " ".join(
            f"{key}={_field_value(value)}" for key, value in fields.items()
        )
        rendered = " ".join(part for part in (message.strip(), suffix) if part)
        self._logger.log(
            level,
            rendered,
            extra={
                "teleop_mode": self.mode,
                "event_code": str(event_code).upper(),
            },
            stacklevel=3,
        )

    def info(self, event_code: str, message: str = "", **fields: Any) -> None:
        self.log(logging.INFO, event_code, message, **fields)

    def warning(self, event_code: str, message: str = "", **fields: Any) -> None:
        self.log(logging.WARNING, event_code, message, **fields)

    def error(self, event_code: str, message: str = "", **fields: Any) -> None:
        self.log(logging.ERROR, event_code, message, **fields)

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.started_monotonic

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._listener.stop()
            for handler in tuple(self._logger.handlers):
                self._logger.removeHandler(handler)
                handler.close()
            for handler in self._outputs:
                handler.flush()
                handler.close()


def configure_event_logger(mode: str) -> AmmrEventLogger:
    return AmmrEventLogger(mode=mode)
