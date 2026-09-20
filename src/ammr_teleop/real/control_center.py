"""Legacy Tk frontend and shared backend for AMMR real-robot processes.

The GUI itself never runs ROS on the AGX.  It sends a checked action to the
SSH-side process manager, which starts every AGX command in its own process
group and cleans that group (and known stale ROS launch processes) on stop.
Password authentication defaults to the current deployment credential and
can be overridden at runtime through ``AMMR_AGX_PASSWORD``.
Camera images are published directly by vendor-SDK ZMQ processes on the AGX.
"""

from __future__ import annotations

import argparse
from ammr_teleop.common.paths import configured_path
import base64
from dataclasses import dataclass
import errno
import os
from pathlib import Path
import pty
import queue
import re
import select
import shlex
import signal
import subprocess
import threading
import time
from typing import Callable, Sequence

try:  # The browser control center must also run on hosts without Tk/X11.
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import messagebox, ttk
except ImportError:  # pragma: no cover - depends on the host's system packages
    tk = None  # type: ignore[assignment]
    tkfont = None  # type: ignore[assignment]
    messagebox = None  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MANAGER_PATH = Path(__file__).with_name("agx_process_manager.sh")
# Requested deployment default. AMMR_AGX_PASSWORD can override it at runtime.
DEFAULT_AGX_PASSWORD = "1212"
AGX_COMPONENTS = (
    ("left_arm", "Left Arm"),
    ("right_arm", "Right Arm"),
    ("left_camera", "Left Wrist Camera"),
    ("right_camera", "Right Wrist Camera"),
    ("head_camera", "Head Camera"),
    ("recorder", "AGX Recorder"),
    ("chassis", "Chassis Driver"),
    ("lift", "Lift Column"),
)


@dataclass(frozen=True)
class ActionResult:
    returncode: int
    output: str


class RemoteAgxManager:
    def __init__(
        self,
        target: str,
        *,
        agx_root: str,
        data_dir: str,
        task: str,
        domain_id: int,
        timeout: float = 20.0,
        password: str | None = None,
    ) -> None:
        if not MANAGER_PATH.is_file():
            raise FileNotFoundError(f"AGX manager script is missing: {MANAGER_PATH}")
        self.target = target
        self.agx_root = agx_root
        self.data_dir = data_dir
        self.task = task
        self.domain_id = int(domain_id)
        self.timeout = float(timeout)
        self.password = password or DEFAULT_AGX_PASSWORD
        self.script = MANAGER_PATH.read_bytes()
        self.relay_source = Path(__file__).with_name("agx_image_relay.py")

    def _ssh_command(self, remote_command: str) -> list[str]:
        command = [
            "ssh",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "ServerAliveInterval=5",
            "-o",
            "ServerAliveCountMax=2",
            self.target,
            remote_command,
        ]
        if self.password:
            # Force the password method when a password was explicitly
            # supplied. This makes the PTY prompt deterministic and avoids
            # accidentally asking for a private-key passphrase instead.
            command[1:1] = [
                "-o",
                "BatchMode=no",
                "-o",
                "PreferredAuthentications=password",
                "-o",
                "PubkeyAuthentication=no",
                "-o",
                "KbdInteractiveAuthentication=no",
                "-o",
                "NumberOfPasswordPrompts=1",
            ]
        else:
            command[1:1] = ["-o", "BatchMode=yes"]
        return command

    def _run_ssh_with_password(
        self, command: list[str], payload: bytes, timeout: float
    ) -> ActionResult:
        """Run SSH and answer its password prompt through a temporary PTY.

        OpenSSH reads passwords from the controlling terminal rather than
        stdin, while the remote manager script itself must be sent on stdin.
        This bridge keeps those two streams separate without sshpass/pexpect.
        The password is written only to the PTY and never to captured logs.
        """
        child_env = {
            key: value
            for key, value in os.environ.items()
            if key not in {
                "AMMR_AGX_PASSWORD",
                "SSH_ASKPASS",
                "SSH_ASKPASS_REQUIRE",
            }
        }
        # Passwords must come from the controlling PTY, never from an
        # askpass helper that may be missing or show a desktop prompt.
        child_env["SSH_ASKPASS_REQUIRE"] = "never"
        # Keep the manager script on a pipe and use the PTY only as ssh's
        # controlling terminal. Sending a multi-kilobyte script directly to
        # a canonical-mode PTY can overflow its line buffer and truncate the
        # beginning of the script (which then looks like a bash syntax error).
        input_read_fd, input_write_fd = os.pipe()
        # pty.fork() gives ssh a real controlling terminal. A plain
        # subprocess.Popen(..., stdin=slave_fd) is not sufficient: OpenSSH
        # then cannot open /dev/tty and falls back to ssh-askpass.
        child_pid, master_fd = pty.fork()
        if child_pid == 0:  # pragma: no cover - executed in the forked child
            os.close(input_write_fd)
            os.dup2(input_read_fd, 0)
            if input_read_fd > 2:
                os.close(input_read_fd)
            try:
                os.execvpe(command[0], command, child_env)
            except BaseException:
                os._exit(127)
        os.close(input_read_fd)
        input_read_fd = -1
        captured = bytearray()
        password_sent = False
        hostkey_answered = False
        timed_out = False
        returncode: int | None = None
        try:
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    timed_out = True
                    break
                ready, _, _ = select.select([master_fd], [], [], min(0.20, remaining))
                if ready:
                    try:
                        chunk = os.read(master_fd, 4096)
                    except OSError as error:
                        if error.errno == errno.EIO:
                            chunk = b""
                        else:
                            raise
                    if chunk:
                        captured.extend(chunk)
                        tail = bytes(captured[-4096:]).lower()
                        if not hostkey_answered and (
                            b"yes/no" in tail or b"yes/no/" in tail
                        ):
                            # Never silently trust a new host key for a robot
                            # control connection. The operator must accept it
                            # once with a normal interactive ssh command.
                            os.write(master_fd, b"no\n")
                            hostkey_answered = True
                        if not password_sent and b"password:" in tail:
                            os.write(master_fd, self.password.encode("utf-8") + b"\n")
                            password_sent = True
                            # SSH reads the remote command's stdin from the
                            # pipe, independently of the password PTY.
                            offset = 0
                            while offset < len(payload):
                                offset += os.write(input_write_fd, payload[offset:])
                            os.close(input_write_fd)
                            input_write_fd = -1
                waited_pid, wait_status = os.waitpid(child_pid, os.WNOHANG)
                if waited_pid == child_pid:
                    returncode = os.waitstatus_to_exitcode(wait_status)
                    break
            if timed_out and returncode is None:
                try:
                    os.killpg(child_pid, signal.SIGKILL)
                except OSError:
                    try:
                        os.kill(child_pid, signal.SIGKILL)
                    except OSError:
                        pass
                _, wait_status = os.waitpid(child_pid, 0)
                returncode = os.waitstatus_to_exitcode(wait_status)
        finally:
            if input_write_fd >= 0:
                os.close(input_write_fd)
            os.close(master_fd)
        output = captured.decode("utf-8", errors="replace").strip()
        if self.password:
            output = output.replace(self.password, "<redacted>")
        if timed_out:
            output = f"SSH action timed out after {timeout:.1f}s\n{output}".strip()
            return ActionResult(1, output)
        return ActionResult(int(returncode if returncode is not None else 1), output)

    def _sync_image_relay(self) -> ActionResult | None:
        """Install the relay beside the remote manager before starting it.

        The UI is commonly run from a server checkout while the AGX checkout
        may lag behind.  Uploading this one small, self-contained script over
        the already configured SSH key keeps the image button self-contained;
        no Python environment or source tree mutation is required on AGX.
        """
        if not self.relay_source.is_file():
            return ActionResult(1, f"image relay source is missing: {self.relay_source}")
        remote_command = (
            "mkdir -p \"$HOME/.cache/ammr_xr_ui\" && "
            "base64 -d > \"$HOME/.cache/ammr_xr_ui/agx_image_relay.py\" && "
            "chmod 755 \"$HOME/.cache/ammr_xr_ui/agx_image_relay.py\""
        )
        payload = base64.b64encode(self.relay_source.read_bytes())
        try:
            command = self._ssh_command(remote_command)
            if self.password:
                result = self._run_ssh_with_password(command, payload, self.timeout)
                if result.returncode == 0:
                    return None
                return ActionResult(
                    result.returncode,
                    f"image relay sync failed (returncode={result.returncode})\n{result.output}".strip(),
                )
            completed = subprocess.run(
                command,
                input=payload,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return ActionResult(1, f"image relay sync failed: {error}")
        output = completed.stdout.decode("utf-8", errors="replace").strip()
        if completed.returncode != 0:
            return ActionResult(
                completed.returncode,
                f"image relay sync failed (returncode={completed.returncode})\n{output}".strip(),
            )
        return None

    def _manager_payload(self) -> bytes:
        """Prefix the remote manager with its short-lived sudo credential."""
        if not self.password:
            return self.script
        assignment = f"export AMMR_SUDO_PASSWORD={shlex.quote(self.password)}\n"
        return assignment.encode("utf-8") + self.script

    def run(self, action: str, component: str = "") -> ActionResult:
        if action == "start" and component == "image_stream":
            sync_error = self._sync_image_relay()
            if sync_error is not None:
                return sync_error
        remote_command = "env " + " ".join(
            (
                f"AMMR_AGX_ROOT={shlex.quote(self.agx_root)}",
                f"AMMR_AGX_OVERLAY_SETUP={shlex.quote(configured_path('agx_overlay_setup'))}",
                f"AMMR_AGX_CAMERA_SETUP={shlex.quote(configured_path('agx_camera_setup'))}",
                f"AMMR_DATA_DIR={shlex.quote(self.data_dir)}",
                f"AMMR_TASK={shlex.quote(self.task)}",
                f"AMMR_DOMAIN_ID={self.domain_id}",
                "bash -s --",
                shlex.quote(action),
                shlex.quote(component),
            )
        )
        # Saving an episode can take up to the recorder's writer stop timeout
        # (30 s by default).  Allow cleanup and explicit record_stop to wait
        # for that response while keeping ordinary component operations fast.
        run_timeout = (
            max(self.timeout, 50.0)
            if action in {"record_start", "record_stop", "master_start", "master_stop", "cleanup"}
            else self.timeout
        )
        try:
            command = self._ssh_command(remote_command)
            payload = self._manager_payload()
            if self.password:
                return self._run_ssh_with_password(command, payload, run_timeout)
            completed = subprocess.run(
                command,
                input=payload,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=run_timeout,
                check=False,
            )
            return ActionResult(
                completed.returncode,
                completed.stdout.decode("utf-8", errors="replace").strip(),
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            return ActionResult(1, f"SSH action failed: {error}")


class LocalManagedProcess:
    """Start/stop one server-side command by process group."""

    def __init__(self, state_dir: Path, name: str) -> None:
        self.state_dir = state_dir
        self.name = name
        self.pid_path = state_dir / f"{name}.pid"
        self.log_path = state_dir / f"{name}.log"
        self.lock = threading.RLock()

    def _read_pid(self) -> int | None:
        try:
            pid = int(self.pid_path.read_text(encoding="ascii").strip())
        except (OSError, ValueError):
            return None
        return pid if pid > 1 else None

    def running(self) -> bool:
        pid = self._read_pid()
        if pid is None:
            return False
        try:
            os.kill(pid, 0)
        except OSError:
            return False
        return True

    def stop(self) -> str:
        with self.lock:
            pid = self._read_pid()
            if pid is None:
                self.pid_path.unlink(missing_ok=True)
                return f"{self.name}: already stopped"
            try:
                pgid = os.getpgid(pid)
            except OSError:
                pgid = pid
            if pgid <= 1 or pgid == os.getpgrp():
                # Never let a stale/reused pid make the GUI kill its own
                # process group. The AGX manager has the same safeguard for
                # its remote process groups.
                self.pid_path.unlink(missing_ok=True)
                return f"{self.name}: stale pid ignored ({pid})"
            try:
                os.killpg(pgid, signal.SIGTERM)
            except OSError:
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and self.running():
                time.sleep(0.05)
            if self.running():
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except OSError:
                    pass
            self.pid_path.unlink(missing_ok=True)
            return f"{self.name}: stopped"

    def start(self, command: str, cwd: Path) -> str:
        with self.lock:
            self.stop()
            self.state_dir.mkdir(parents=True, exist_ok=True)
            log = self.log_path.open("ab")
            try:
                process = subprocess.Popen(
                    ["setsid", "bash", "-lc", command],
                    cwd=str(cwd),
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=False,
                    close_fds=True,
                )
            except Exception:
                log.close()
                raise
            finally:
                log.close()
            self.pid_path.write_text(f"{process.pid}\n", encoding="ascii")
            return f"{self.name}: started PID={process.pid}, log={self.log_path}"


class ControlCenter:
    def __init__(self, args: argparse.Namespace) -> None:
        if tk is None or tkfont is None or ttk is None or messagebox is None:
            raise RuntimeError(
                "Tk control center requires python3-tk; use the browser UI "
                "with scripts/ammr.sh control-center instead."
            )
        self.args = args
        self.root = tk.Tk()
        self.root.title("AMMR XR Real Robot Control Center")
        self.root.geometry("1160x840")
        self.root.minsize(1020, 720)
        self.root.configure(bg="#f5f7fb")
        self.left_enable_var = tk.BooleanVar(value=bool(args.left_enable))
        self.right_enable_var = tk.BooleanVar(value=bool(args.right_enable))
        # Keep the display's native Tk scale.  A fractional override such as
        # 1.15 causes X11/remote-desktop compositors to resample glyphs and
        # makes otherwise sharp text look blurry.  Readability is handled by
        # integer font sizes below instead.
        self._configure_style()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.busy = False
        self.master_ready = False
        self.component_states = {name: False for name, _ in AGX_COMPONENTS}
        self.recording_active = False
        self.recording_episode: int | None = None
        self.server_xr = LocalManagedProcess(
            Path(args.server_state_dir).expanduser(), "server_xr"
        )
        self.agx = RemoteAgxManager(
            f"{args.agx_user}@{args.agx_host}",
            agx_root=args.agx_root,
            data_dir=args.agx_data_dir,
            task=args.task,
            domain_id=args.domain_id,
            password=os.environ.get("AMMR_AGX_PASSWORD"),
        )
        self.buttons: dict[str, ttk.Button] = {}
        self.status_labels: dict[str, ttk.Label] = {}
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._drain_events)
        self.root.after(1000, self._refresh_status)

    def _configure_style(self) -> None:
        """Configure a readable, high-contrast style for the control panel."""
        # Use Tk font objects instead of tuple font descriptions.  The
        # single-file Droid fallback font is reliably rasterized by Tk/X11
        # and provides the Chinese glyphs that the desktop default may lack.
        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(size=12)
        self.ui_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_font",
            exists=False,
            family="Droid Sans Fallback",
            size=12,
        )
        self.ui_small_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_small_font",
            exists=False,
            family="Droid Sans Fallback",
            size=11,
        )
        self.ui_section_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_section_font",
            exists=False,
            family="Droid Sans Fallback",
            size=13,
            weight="bold",
        )
        self.ui_title_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_title_font",
            exists=False,
            family="Droid Sans Fallback",
            size=19,
            weight="bold",
        )
        self.ui_button_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_button_font",
            exists=False,
            family="Droid Sans Fallback",
            size=12,
            weight="bold",
        )
        self.ui_status_font = tkfont.Font(
            root=self.root,
            name="ammr_ui_status_font",
            exists=False,
            family="Droid Sans Fallback",
            size=12,
            weight="bold",
        )
        self.log_font = tkfont.Font(
            root=self.root,
            name="ammr_log_font",
            exists=False,
            family="Droid Sans Fallback",
            size=11,
        )
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=self.ui_font)
        style.configure("TFrame", background="#f5f7fb")
        style.configure(
            "TLabelframe",
            background="#f5f7fb",
            bordercolor="#d7dee9",
            relief="solid",
            borderwidth=1,
        )
        style.configure(
            "TLabelframe.Label",
            background="#f5f7fb",
            foreground="#1f2937",
            font=self.ui_section_font,
        )
        style.configure(
            "TLabel",
            background="#f5f7fb",
            foreground="#263548",
            font=self.ui_font,
        )
        style.configure(
            "Title.TLabel",
            background="#f5f7fb",
            foreground="#0f172a",
            font=self.ui_title_font,
        )
        style.configure(
            "Subtitle.TLabel",
            background="#f5f7fb",
            foreground="#64748b",
            font=self.ui_small_font,
        )
        style.configure(
            "Hint.TLabel",
            background="#f5f7fb",
            foreground="#64748b",
            font=self.ui_small_font,
        )
        style.configure(
            "TButton",
            font=self.ui_button_font,
            padding=(12, 8),
            foreground="#1f2937",
            background="#ffffff",
            bordercolor="#cbd5e1",
            lightcolor="#ffffff",
            darkcolor="#cbd5e1",
        )
        style.map(
            "TButton",
            background=[
                ("disabled", "#e5e7eb"),
                ("pressed", "#e2e8f0"),
                ("active", "#f1f5f9"),
            ],
            foreground=[("disabled", "#94a3b8")],
        )
        for name, background, active in (
            ("Primary.TButton", "#2563eb", "#1d4ed8"),
            ("Start.TButton", "#0f766e", "#0d9488"),
            ("Stop.TButton", "#dc2626", "#b91c1c"),
            ("RecordStart.TButton", "#15803d", "#166534"),
            ("RecordStop.TButton", "#c2410c", "#9a3412"),
            ("Danger.TButton", "#991b1b", "#7f1d1d"),
        ):
            style.configure(
                name,
                font=self.ui_button_font,
                padding=(12, 8),
                foreground="#ffffff",
                background=background,
                bordercolor=background,
                lightcolor=background,
                darkcolor=background,
            )
            style.map(
                name,
                background=[
                    ("disabled", "#d1d5db"),
                    ("pressed", active),
                    ("active", active),
                ],
                foreground=[("disabled", "#6b7280")],
            )
        style.configure(
            "Running.TLabel",
            background="#f5f7fb",
            foreground="#15803d",
            font=self.ui_status_font,
        )
        style.configure(
            "Stopped.TLabel",
            background="#f5f7fb",
            foreground="#64748b",
            font=self.ui_font,
        )
        style.configure(
            "Ready.TLabel",
            background="#f5f7fb",
            foreground="#15803d",
            font=self.ui_status_font,
        )
        style.configure(
            "NotReady.TLabel",
            background="#f5f7fb",
            foreground="#b45309",
            font=self.ui_font,
        )
        style.configure(
            "Recording.TLabel",
            background="#f5f7fb",
            foreground="#c2410c",
            font=self.ui_status_font,
        )

    def _build(self) -> None:
        outer = ttk.Frame(self.root, padding=(18, 14, 18, 16))
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="AMMR XR Real Robot Control Center", style="Title.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            outer,
            text="Remote AGX components | XR teleoperation | episode recording",
            style="Subtitle.TLabel",
        ).pack(anchor="w", pady=(2, 10))

        settings = ttk.LabelFrame(outer, text="Connection & Recording", padding=(12, 10))
        settings.pack(fill="x")
        ttk.Label(settings, text="AGX", style="Hint.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(settings, text=f"{self.args.agx_user}@{self.args.agx_host}").grid(
            row=0, column=1, sticky="w", padx=(4, 24)
        )
        ttk.Label(settings, text="ROS_DOMAIN_ID", style="Hint.TLabel").grid(
            row=0, column=2, sticky="w"
        )
        ttk.Label(settings, text=str(self.args.domain_id)).grid(
            row=0, column=3, sticky="w", padx=(4, 24)
        )
        ttk.Label(settings, text="Task", style="Hint.TLabel").grid(
            row=0, column=4, sticky="w"
        )
        task_display = (
            self.args.task if str(self.args.task).isascii() else "Custom UTF-8 task"
        )
        ttk.Label(
            settings, text=task_display, wraplength=320, justify="left"
        ).grid(row=0, column=5, sticky="w", padx=(4, 0))
        ttk.Label(settings, text="AGX data directory", style="Hint.TLabel").grid(
            row=1, column=0, sticky="w", pady=(7, 0)
        )
        ttk.Label(
            settings,
            text=self.args.agx_data_dir,
            style="Hint.TLabel",
            wraplength=760,
            justify="left",
        ).grid(row=1, column=1, columnspan=5, sticky="w", padx=4, pady=(7, 0))
        settings.columnconfigure(5, weight=1)

        top = ttk.Frame(outer)
        top.pack(fill="x", pady=(12, 8))
        self.buttons["master"] = ttk.Button(
            top,
            text="AGX Master: Start",
            command=self._toggle_master,
            style="Primary.TButton",
        )
        self.buttons["master"].pack(side="left")
        ttk.Button(top, text="Refresh Status", command=self._refresh_status).pack(
            side="left", padx=8
        )
        ttk.Button(
            top,
            text="Stop All AGX Components",
            command=self._stop_all,
            style="Danger.TButton",
        ).pack(side="left")
        self.status_labels["master"] = ttk.Label(
            top, text="[OFFLINE] Disconnected", style="NotReady.TLabel"
        )
        self.status_labels["master"].pack(side="left", padx=14)

        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True, pady=(0, 8))
        body.columnconfigure(0, weight=1, uniform="control-panels")
        body.columnconfigure(1, weight=1, uniform="control-panels")
        body.rowconfigure(0, weight=1)
        agx_frame = ttk.LabelFrame(
            body, text="AGX Components (start master first)", padding=(12, 10)
        )
        agx_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        for row, (name, label) in enumerate(AGX_COMPONENTS):
            button = ttk.Button(
                agx_frame,
                text=f"Start {label}",
                command=lambda selected=name: self._toggle_component(selected),
                state="disabled",
                style="Start.TButton",
            )
            row_padding = (4, 11) if row in (1, 4, 6) else 4
            button.grid(row=row, column=0, sticky="ew", pady=row_padding)
            self.buttons[name] = button
            status = ttk.Label(agx_frame, text="[STOPPED]", style="Stopped.TLabel")
            status.grid(row=row, column=1, sticky="w", padx=8)
            self.status_labels[name] = status
        agx_frame.columnconfigure(0, weight=1)

        server_frame = ttk.LabelFrame(body, text="Server Processes", padding=(12, 10))
        server_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        arm_selection = ttk.Frame(server_frame)
        arm_selection.pack(fill="x", pady=(2, 6))
        self.left_enable_check = ttk.Checkbutton(
            arm_selection, text="Left enable", variable=self.left_enable_var
        )
        self.left_enable_check.pack(side="left", padx=(0, 16))
        self.right_enable_check = ttk.Checkbutton(
            arm_selection, text="Right enable", variable=self.right_enable_var
        )
        self.right_enable_check.pack(side="left")
        self.buttons["server_xr"] = ttk.Button(
            server_frame,
            text="Start Server XR Teleoperation",
            command=self._toggle_server_xr,
            style="Primary.TButton",
        )
        self.buttons["server_xr"].pack(fill="x", pady=4)
        self.status_labels["server_xr"] = ttk.Label(
            server_frame, text="[STOPPED]", style="Stopped.TLabel"
        )
        self.status_labels["server_xr"].pack(anchor="w", pady=(0, 12))
        recording_frame = ttk.LabelFrame(
            server_frame, text="Data Recording", padding=(10, 9)
        )
        recording_frame.pack(fill="x", pady=(0, 12))
        self.buttons["recording"] = ttk.Button(
            recording_frame,
            text="Start Episode",
            command=self._toggle_recording,
            state="disabled",
            style="RecordStart.TButton",
        )
        self.buttons["recording"].pack(fill="x", pady=(0, 4))
        self.status_labels["recording"] = ttk.Label(
            recording_frame, text="[STOPPED] Recorder Not Running", style="Stopped.TLabel"
        )
        self.status_labels["recording"].pack(anchor="w")
        ttk.Label(
            recording_frame,
            text="Calls AGX /ammr/recording/set_enabled.\n"
            "Stop waits for the episode to be saved. Left X/[s] remains available.",
            justify="left",
            style="Hint.TLabel",
        ).pack(anchor="w", pady=(5, 0))
        ttk.Label(
            server_frame,
            text="Server XR controls:\nSelected arms + chassis + lift\n"
            "Both arms stay online and are recorded; a disabled side returns to zero.\n"
            "Three ZMQ image streams are pre-subscribed;\n"
            "actual acquisition is controlled by the AGX camera buttons.\n\n"
            "AGX Master manages AGX processes only.\n"
            "Stopping Server XR does not stop AGX components.",
            justify="left",
            style="Hint.TLabel",
        ).pack(anchor="w")

        log_frame = ttk.LabelFrame(outer, text="Runtime Log", padding=(8, 8))
        log_frame.pack(fill="both", expand=True)
        self.log_text = tk.Text(
            log_frame,
            height=8,
            state="disabled",
            wrap="word",
            font=self.log_font,
            background="#111827",
            foreground="#e5e7eb",
            insertbackground="#e5e7eb",
            selectbackground="#334155",
            relief="flat",
            borderwidth=0,
            padx=10,
            pady=8,
        )
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"{timestamp} {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        if busy:
            for button in self.buttons.values():
                button.configure(state="disabled")
            self.left_enable_check.configure(state="disabled")
            self.right_enable_check.configure(state="disabled")
        else:
            self._update_buttons()

    def _update_buttons(self) -> None:
        self.buttons["master"].configure(
            text="AGX Master: Stop" if self.master_ready else "AGX Master: Start",
            state="normal",
            style="Stop.TButton" if self.master_ready else "Primary.TButton",
        )
        self.status_labels["master"].configure(
            text="[READY]" if self.master_ready else "[OFFLINE] Disconnected",
            style="Ready.TLabel" if self.master_ready else "NotReady.TLabel",
        )
        for name, _ in AGX_COMPONENTS:
            running = self.component_states[name]
            self.buttons[name].configure(
                text=("Stop " if running else "Start ")
                + dict(AGX_COMPONENTS)[name],
                state="normal" if self.master_ready else "disabled",
                style="Stop.TButton" if running else "Start.TButton",
            )
            self.status_labels[name].configure(
                text="[RUNNING]" if running else "[STOPPED]",
                style="Running.TLabel" if running else "Stopped.TLabel",
            )
        server_running = self.server_xr.running()
        selector_state = "disabled" if server_running else "normal"
        self.left_enable_check.configure(state=selector_state)
        self.right_enable_check.configure(state=selector_state)
        self.buttons["server_xr"].configure(
            text=(
                "Stop Server XR Teleoperation"
                if server_running
                else "Start Server XR Teleoperation"
            ),
            state="normal",
            style="Stop.TButton" if server_running else "Primary.TButton",
        )
        self.status_labels["server_xr"].configure(
            text="[RUNNING]" if server_running else "[STOPPED]",
            style="Running.TLabel" if server_running else "Stopped.TLabel",
        )
        recorder_running = self.component_states.get("recorder", False)
        can_record = self.master_ready and (recorder_running or self.recording_active)
        self.buttons["recording"].configure(
            text=(
                "Stop & Save Episode"
                if self.recording_active
                else "Start Episode"
            ),
            state="normal" if can_record else "disabled",
            style=("RecordStop.TButton" if self.recording_active else "RecordStart.TButton"),
        )
        if self.recording_active:
            episode = (
                f" episode_{self.recording_episode:06d}"
                if self.recording_episode is not None
                else ""
            )
            self.status_labels["recording"].configure(
                text=f"[RECORDING]{episode}", style="Recording.TLabel"
            )
        elif recorder_running:
            self.status_labels["recording"].configure(
                text="[IDLE] Recorder ready", style="Stopped.TLabel"
            )
        else:
            self.status_labels["recording"].configure(
                text="[STOPPED] Recorder Not Running", style="Stopped.TLabel"
            )

    def _run_remote(self, action: str, component: str = "") -> None:
        self._set_busy(True)
        self._log(f"SSH -> {action} {component}".rstrip())

        def worker() -> None:
            result = self.agx.run(action, component)
            self.events.put(("remote", (action, component, result)))

        threading.Thread(target=worker, daemon=True).start()

    def _toggle_master(self) -> None:
        if self.busy:
            return
        self._run_remote("master_stop" if self.master_ready else "master_start")

    def _toggle_component(self, component: str) -> None:
        if self.busy or not self.master_ready:
            return
        self._run_remote(
            "stop" if self.component_states[component] else "start", component
        )

    def _toggle_recording(self) -> None:
        if self.busy:
            return
        recorder_running = self.component_states.get("recorder", False)
        if not self.master_ready or (not recorder_running and not self.recording_active):
            self._log("Start AGX Master and AGX Recorder first.")
            return
        self._run_remote("record_stop" if self.recording_active else "record_start")

    def _stop_all(self) -> None:
        if self.busy:
            return
        self._run_remote("cleanup")

    def _refresh_status(self) -> None:
        if self.busy:
            return
        self._run_remote("status")

    def _server_command(self) -> str:
        root = shlex.quote(str(PROJECT_ROOT))
        lifting = str(Path(configured_path("ros_ws")) / "install/local_setup.bash")
        arm_selection = " ".join(
            (
                "--left-enable" if self.left_enable_var.get() else "--no-left-enable",
                "--right-enable" if self.right_enable_var.get() else "--no-right-enable",
            )
        )
        return (
            "source /opt/ros/humble/setup.bash; "
            f"if [ -f {shlex.quote(lifting)} ]; then source {shlex.quote(lifting)}; fi; "
            f"cd {root}; export ROS_DOMAIN_ID={int(self.args.domain_id)}; "
            "exec ./scripts/ammr.sh teleop --real "
            "--record "
            "--profile piper_absolute_smooth_v1 "
            "--enable-hardware-output "
            "--hardware-components left_arm right_arm lift chassis "
            f"{arm_selection} "
            "--image-stream "
            "--image-transport zmq "
            f"--display-mode {shlex.quote(self.args.display_mode)} "
            f"--task {shlex.quote(self.args.task)}"
        )

    def _toggle_server_xr(self) -> None:
        if self.busy:
            return
        server_running = self.server_xr.running()
        if (
            not server_running
            and not self.left_enable_var.get()
            and not self.right_enable_var.get()
        ):
            self._log("Select Left enable and/or Right enable before starting Server XR.")
            return
        # Read Tk variables on the UI thread; Tk variables are not safe to
        # access from the process-launch worker.
        server_command = None if server_running else self._server_command()
        self._set_busy(True)
        if server_running:
            self.events.put(("local", self.server_xr.stop()))
            return

        def worker() -> None:
            try:
                assert server_command is not None
                message = self.server_xr.start(server_command, PROJECT_ROOT)
            except Exception as error:
                message = f"server_xr: start failed: {error}"
            self.events.put(("local", message))

        threading.Thread(target=worker, daemon=True).start()

    def _handle_remote(self, payload: tuple[str, str, ActionResult]) -> None:
        action, component, result = payload
        if result.output:
            self._log(result.output)
        if result.returncode != 0:
            self._log(f"FAILED: {action} {component} (returncode={result.returncode})")
        elif action == "master_start":
            self.master_ready = True
        elif action in ("master_stop", "cleanup"):
            self.master_ready = False
            self.component_states = {name: False for name, _ in AGX_COMPONENTS}
            self.recording_active = False
            self.recording_episode = None
        elif action == "start" and component:
            self.component_states[component] = True
            if component == "recorder":
                self.recording_active = False
                self.recording_episode = None
        elif action == "stop" and component:
            self.component_states[component] = False
            if component == "recorder":
                self.recording_active = False
                self.recording_episode = None
        elif action in ("record_start", "record_stop"):
            if not self._recording_service_succeeded(result):
                self._log(
                    "Recording request rejected; check the AGX Recorder log and "
                    "/ammr/recording/status."
                )
            elif action == "record_start":
                self.recording_active = True
                self.recording_episode = self._extract_episode(result.output)
            else:
                self.recording_active = False
                self.recording_episode = None
        elif action == "status":
            self._parse_status(result.output)

    @staticmethod
    def _recording_service_succeeded(result: ActionResult) -> bool:
        if result.returncode != 0:
            return False
        normalized = "".join(result.output.lower().split())
        return "success=false" not in normalized and "success:false" not in normalized

    @staticmethod
    def _extract_episode(output: str) -> int | None:
        for pattern in (
            r"[\"']episode[\"']\s*:\s*(-?\d+)",
            r"\bepisode\s*[=:]\s*(-?\d+)",
        ):
            match = re.search(pattern, output, flags=re.IGNORECASE)
            if match:
                try:
                    value = int(match.group(1))
                except ValueError:
                    continue
                return value if value >= 0 else None
        return None

    def _parse_status(self, output: str) -> None:
        for line in output.splitlines():
            fields = line.split()
            if fields[:2] == ["MASTER", "READY"]:
                self.master_ready = True
            elif fields[:2] in (["MASTER", "STOPPED"], ["MASTER", "DEGRADED"]):
                self.master_ready = False
            elif len(fields) >= 3 and fields[0] == "STATUS":
                self.component_states[fields[1]] = fields[2] == "RUNNING"
        self.status_labels["master"].configure(
            text="[READY]" if self.master_ready else "[OFFLINE] Not Ready",
            style="Ready.TLabel" if self.master_ready else "NotReady.TLabel",
        )

    def _drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "remote":
                    self._handle_remote(payload)  # type: ignore[arg-type]
                else:
                    self._log(str(payload))
                self._set_busy(False)
        except queue.Empty:
            pass
        self.root.after(100, self._drain_events)

    def _on_close(self) -> None:
        running = (
            self.master_ready
            or self.server_xr.running()
            or self.recording_active
            or any(self.component_states.values())
        )
        if running and not messagebox.askyesno(
            "Confirm Exit",
            "Managed processes are still running. Stop Server XR and all AGX components first?",
        ):
            self.root.destroy()
            return
        if running:
            self.server_xr.stop()
            try:
                if self.recording_active:
                    result = self.agx.run("record_stop")
                    if result.output:
                        self._log(result.output)
                self.agx.run("master_stop")
            except Exception:
                pass
        self.root.destroy()

    def run(self) -> None:
        self._log("Control center started; click AGX Master first.")
        self.root.mainloop()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--agx-host", default="192.168.3.99")
    parser.add_argument("--agx-user", default="lin")
    parser.add_argument("--agx-root", default=configured_path("agx_root"))
    parser.add_argument(
        "--agx-data-dir",
        default=configured_path("agx_data_dir"),
    )
    parser.add_argument("--task", default="XR双臂遥操作数据采集")
    parser.add_argument("--domain-id", type=int, default=101)
    parser.add_argument(
        "--left-enable", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument(
        "--right-enable", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument(
        "--display-mode",
        choices=("pass-through", "immersive", "ego"),
        default="pass-through",
        help="XR display mode; immersive/ego renders the returned head image.",
    )
    parser.add_argument(
        "--server-state-dir", default="/tmp/ammr_xr_control_center"
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    ControlCenter(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
