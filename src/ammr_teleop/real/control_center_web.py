"""Browser-based control center for AMMR real-robot processes.

The web server is intentionally dependency-free: it uses Python's standard
library HTTP server and serves a self-contained HTML/CSS/JavaScript page.  The
SSH process manager and the local Server-XR process are the same backends used
by the former Tk control center, so changing the UI does not change the robot
control or recording path.
"""

from __future__ import annotations

import argparse
from ammr_teleop.common.paths import configured_path
from collections import deque
import json
import os
import re
import shlex
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Sequence

from ammr_teleop.real.control_center import (
    AGX_COMPONENTS,
    DEFAULT_AGX_PASSWORD,
    PROJECT_ROOT,
    ActionResult,
    LocalManagedProcess,
    RemoteAgxManager,
)


WEB_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AMMR XR Control Center</title>
  <style>
    :root {
      color-scheme: light;
      --page: #eef2f7;
      --card: #ffffff;
      --card-soft: #f8fafc;
      --ink: #172033;
      --muted: #64748b;
      --line: #dbe3ee;
      --blue: #2563eb;
      --blue-dark: #1d4ed8;
      --teal: #0f766e;
      --teal-dark: #0d9488;
      --green: #15803d;
      --green-dark: #166534;
      --red: #b91c1c;
      --red-dark: #991b1b;
      --orange: #c2410c;
      --shadow: 0 16px 42px rgba(15, 23, 42, .08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-width: 320px;
      color: var(--ink);
      background: radial-gradient(circle at top right, #dce8ff 0, var(--page) 38rem);
      font-family: Inter, "Segoe UI", Arial, sans-serif;
      -webkit-font-smoothing: antialiased;
      text-rendering: optimizeLegibility;
    }
    .shell { max-width: 1380px; margin: 0 auto; padding: 28px; }
    .topbar {
      display: flex; align-items: flex-start; justify-content: space-between;
      gap: 20px; margin-bottom: 22px;
    }
    h1 { margin: 0; font-size: clamp(25px, 3vw, 36px); letter-spacing: -.035em; }
    .subtitle { margin: 7px 0 0; color: var(--muted); font-size: 14px; }
    .badges { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 8px; }
    .badge {
      display: inline-flex; align-items: center; gap: 7px; padding: 8px 11px;
      border: 1px solid var(--line); border-radius: 999px; background: rgba(255,255,255,.82);
      color: #475569; font-size: 12px; font-weight: 650; white-space: nowrap;
    }
    .badge strong { color: var(--ink); font-weight: 750; }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: #94a3b8; }
    .dot.ready { background: var(--green); box-shadow: 0 0 0 4px #dcfce7; }
    .dot.busy { background: var(--orange); box-shadow: 0 0 0 4px #ffedd5; }
    .toolbar {
      display: flex; flex-wrap: wrap; align-items: center; gap: 10px;
      padding: 14px; margin-bottom: 18px; border: 1px solid var(--line);
      border-radius: 18px; background: rgba(255,255,255,.78); box-shadow: var(--shadow);
    }
    .toolbar .status { margin-left: 3px; }
    .grid { display: grid; grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); gap: 18px; }
    .card {
      min-width: 0; padding: 18px; border: 1px solid var(--line); border-radius: 20px;
      background: var(--card); box-shadow: var(--shadow);
    }
    .card-head { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; margin-bottom: 13px; }
    .card h2 { margin: 0; font-size: 17px; letter-spacing: -.015em; }
    .card-note { color: var(--muted); font-size: 12px; text-align: right; }
    .component-list { display: grid; grid-template-columns: minmax(0, 1fr); gap: 9px; }
    .component-row { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: center; gap: 12px; }
    button {
      min-height: 42px; border: 1px solid transparent; border-radius: 11px; padding: 10px 14px;
      color: #fff; background: var(--teal); font: inherit; font-size: 13px; font-weight: 750;
      letter-spacing: .005em; cursor: pointer; transition: transform .12s ease, background .12s ease, opacity .12s ease, box-shadow .12s ease;
    }
    button:hover:not(:disabled) { transform: translateY(-1px); box-shadow: 0 7px 16px rgba(15, 23, 42, .14); }
    button:active:not(:disabled) { transform: translateY(0); }
    button:disabled { cursor: not-allowed; color: #64748b; background: #e2e8f0; border-color: #cbd5e1; }
    button.primary { background: var(--blue); }
    button.primary:hover:not(:disabled) { background: var(--blue-dark); }
    /* Keep all start actions as legible as the AGX Master button. */
    button.start { background: var(--blue); }
    button.start:hover:not(:disabled) { background: var(--blue-dark); }
    button.stop { background: var(--red); }
    button.stop:hover:not(:disabled) { background: var(--red-dark); }
    button.record-start { background: var(--blue); }
    button.record-start:hover:not(:disabled) { background: var(--blue-dark); }
    button.record-stop { background: var(--orange); }
    button.record-stop:hover:not(:disabled) { background: #9a3412; }
    button.danger { background: var(--red-dark); }
    button.ghost { color: #334155; background: #fff; border-color: var(--line); }
    button.ghost:hover:not(:disabled) { background: #f8fafc; }
    .status { color: var(--muted); font-size: 12px; font-weight: 700; white-space: nowrap; }
    .status.running, .status.ready { color: var(--green); }
    .status.recording { color: var(--orange); }
    .status.stopped { color: #64748b; }
    .server-stack { display: grid; gap: 14px; }
    .subcard { padding: 14px; border: 1px solid var(--line); border-radius: 15px; background: var(--card-soft); }
    .subcard h3 { margin: 0 0 10px; font-size: 13px; letter-spacing: .02em; text-transform: uppercase; color: #475569; }
    .subcard button { width: 100%; }
    .arm-options { display: flex; flex-wrap: wrap; gap: 10px 18px; margin: 0 0 11px; }
    .arm-option { display: inline-flex; align-items: center; gap: 7px; color: #334155; font-size: 13px; font-weight: 700; }
    .arm-option input { width: 17px; height: 17px; accent-color: var(--blue); }
    .help { margin: 10px 0 0; color: var(--muted); font-size: 12px; line-height: 1.55; }
    .log-card { margin-top: 18px; }
    .log {
      min-height: 180px; max-height: 320px; overflow: auto; margin: 0; padding: 14px;
      border-radius: 13px; background: #0f172a; color: #dbeafe; font: 12px/1.55 "SFMono-Regular", Consolas, "Liberation Mono", monospace;
      white-space: pre-wrap; overflow-wrap: anywhere;
    }
    .empty { color: #94a3b8; }
    .toast {
      position: fixed; right: 22px; bottom: 22px; max-width: min(420px, calc(100vw - 44px));
      padding: 12px 15px; border: 1px solid #fecaca; border-radius: 12px; background: #fff1f2;
      color: #991b1b; box-shadow: var(--shadow); font-size: 13px; font-weight: 650; opacity: 0;
      pointer-events: none; transform: translateY(9px); transition: opacity .18s ease, transform .18s ease;
    }
    .toast.show { opacity: 1; transform: translateY(0); }
    @media (max-width: 860px) {
      .shell { padding: 18px; }
      .topbar { display: block; }
      .badges { justify-content: flex-start; margin-top: 14px; }
      .grid { grid-template-columns: 1fr; }
    }
    @media (max-width: 520px) {
      .component-row { grid-template-columns: 1fr; gap: 4px; }
      .component-row .status { padding-left: 3px; }
      .toolbar button { flex: 1 1 145px; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header class="topbar">
      <div>
        <h1>AMMR XR Real Robot Control Center</h1>
        <p class="subtitle">Remote AGX components | XR teleoperation | episode recording</p>
      </div>
      <div class="badges">
        <span class="badge"><span class="dot" id="masterDot"></span><strong id="masterBadge">AGX offline</strong></span>
        <span class="badge">AGX <strong id="agxTarget">-</strong></span>
        <span class="badge">ROS domain <strong id="domainBadge">-</strong></span>
        <span class="badge">Task <strong id="taskBadge">-</strong></span>
      </div>
    </header>

    <section class="toolbar">
      <button id="masterBtn" class="primary">AGX Master: Start</button>
      <button id="refreshBtn" class="ghost">Refresh Status</button>
      <button id="stopAllBtn" class="danger">Stop All AGX Components</button>
      <span class="status" id="masterStatus">[OFFLINE] Disconnected</span>
      <span class="status" id="busyStatus"></span>
    </section>

    <section class="grid">
      <article class="card">
        <div class="card-head">
          <h2>AGX Components</h2>
          <span class="card-note">Start master first</span>
        </div>
        <div class="component-list" id="componentList"></div>
      </article>

      <article class="card server-stack">
        <div class="card-head">
          <h2>Server Processes</h2>
          <span class="card-note">Runs on the server</span>
        </div>
        <div class="subcard">
          <h3>XR teleoperation</h3>
          <div class="arm-options">
            <label class="arm-option"><input id="leftEnable" type="checkbox" checked> Left enable</label>
            <label class="arm-option"><input id="rightEnable" type="checkbox" checked> Right enable</label>
          </div>
          <button id="serverBtn" class="primary">Start Server XR Teleoperation</button>
          <div class="status stopped" id="serverStatus">[STOPPED]</div>
          <p class="help">Both arm drivers stay online and both sides are recorded. A disabled side smoothly returns to zero.</p>
        </div>
        <div class="subcard">
          <h3>Data recording</h3>
          <button id="recordBtn" class="record-start">Start Episode</button>
          <div class="status stopped" id="recordStatus">[STOPPED] Recorder Not Running</div>
          <p class="help">Calls AGX <code>/ammr/recording/set_enabled</code>. Stop waits for the episode to be saved. Left X/[s] remains available.</p>
        </div>
        <p class="help">Server XR controls selected arms, chassis and lift. Three ZMQ image streams are pre-subscribed; actual acquisition is controlled by the AGX camera components.</p>
      </article>
    </section>

    <section class="card log-card">
      <div class="card-head">
        <h2>Runtime Log</h2>
        <span class="card-note" id="lastAction">Waiting for an action</span>
      </div>
      <pre class="log" id="log"><span class="empty">No events yet.</span></pre>
    </section>
  </main>
  <div class="toast" id="toast"></div>

  <script>
    const componentList = document.getElementById('componentList');
    const masterBtn = document.getElementById('masterBtn');
    const refreshBtn = document.getElementById('refreshBtn');
    const stopAllBtn = document.getElementById('stopAllBtn');
    const serverBtn = document.getElementById('serverBtn');
    const recordBtn = document.getElementById('recordBtn');
    const leftEnable = document.getElementById('leftEnable');
    const rightEnable = document.getElementById('rightEnable');
    const toast = document.getElementById('toast');
    let toastTimer = null;
    let state = null;

    function showToast(message) {
      toast.textContent = message;
      toast.classList.add('show');
      clearTimeout(toastTimer);
      toastTimer = setTimeout(() => toast.classList.remove('show'), 4200);
    }

    function makeComponentRows(components) {
      componentList.innerHTML = '';
      Object.entries(components).forEach(([name, component]) => {
        const row = document.createElement('div');
        row.className = 'component-row';
        const button = document.createElement('button');
        button.dataset.component = name;
        button.className = 'start';
        const status = document.createElement('span');
        status.className = 'status stopped';
        status.dataset.status = name;
        row.append(button, status);
        componentList.appendChild(row);
      });
    }

    async function sendAction(action, component = '') {
      try {
        const request = {action, component};
        if (action === 'server_start') {
          request.left_enable = leftEnable.checked;
          request.right_enable = rightEnable.checked;
          if (!request.left_enable && !request.right_enable) {
            throw new Error('Select Left enable and/or Right enable.');
          }
        }
        const response = await fetch('/api/action', {
          method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify(request),
        });
        const payload = await response.json();
        if (!response.ok) throw new Error(payload.error || payload.message || `HTTP ${response.status}`);
      } catch (error) {
        showToast(error.message || String(error));
      }
    }

    function render(next) {
      state = next;
      if (!componentList.children.length || Object.keys(next.components).length !== componentList.children.length) {
        makeComponentRows(next.components);
      }
      const busy = Boolean(next.busy);
      const masterReady = Boolean(next.master_ready);
      document.getElementById('agxTarget').textContent = next.agx_target;
      document.getElementById('domainBadge').textContent = next.domain_id;
      document.getElementById('taskBadge').textContent = next.task_display;
      document.getElementById('masterBadge').textContent = masterReady ? 'AGX ready' : 'AGX offline';
      document.getElementById('masterDot').className = `dot ${busy ? 'busy' : (masterReady ? 'ready' : '')}`;
      document.getElementById('masterStatus').textContent = masterReady ? '[READY]' : '[OFFLINE] Disconnected';
      document.getElementById('masterStatus').className = `status ${masterReady ? 'ready' : 'stopped'}`;
      document.getElementById('busyStatus').textContent = busy ? `Working: ${next.action}` : '';
      document.getElementById('lastAction').textContent = next.last_action || 'Waiting for an action';

      masterBtn.textContent = masterReady ? 'AGX Master: Stop' : 'AGX Master: Start';
      masterBtn.className = masterReady ? 'stop' : 'primary';
      masterBtn.disabled = busy;
      refreshBtn.disabled = busy;
      stopAllBtn.disabled = busy;
      serverBtn.textContent = next.server_xr_running ? 'Stop Server XR Teleoperation' : 'Start Server XR Teleoperation';
      serverBtn.className = next.server_xr_running ? 'stop' : 'primary';
      serverBtn.disabled = busy;
      if (document.activeElement !== leftEnable && !next.server_xr_running) leftEnable.checked = Boolean(next.left_enable);
      if (document.activeElement !== rightEnable && !next.server_xr_running) rightEnable.checked = Boolean(next.right_enable);
      leftEnable.disabled = busy || next.server_xr_running;
      rightEnable.disabled = busy || next.server_xr_running;
      document.getElementById('serverStatus').textContent = next.server_xr_running ? '[RUNNING]' : '[STOPPED]';
      document.getElementById('serverStatus').className = `status ${next.server_xr_running ? 'running' : 'stopped'}`;

      Object.entries(next.components).forEach(([name, component]) => {
        const button = componentList.querySelector(`button[data-component="${CSS.escape(name)}"]`);
        const status = componentList.querySelector(`.status[data-status="${CSS.escape(name)}"]`);
        if (!button || !status) return;
        button.textContent = `${component.running ? 'Stop ' : 'Start '}${component.label}`;
        button.className = component.running ? 'stop' : 'start';
        button.disabled = busy || !masterReady;
        status.textContent = component.running ? '[RUNNING]' : '[STOPPED]';
        status.className = `status ${component.running ? 'running' : 'stopped'}`;
      });

      const canRecord = masterReady && (next.components.recorder?.running || next.recording_active);
      recordBtn.textContent = next.recording_active ? 'Stop & Save Episode' : 'Start Episode';
      recordBtn.className = next.recording_active ? 'record-stop' : 'record-start';
      recordBtn.disabled = busy || !canRecord;
      const recStatus = document.getElementById('recordStatus');
      if (next.recording_active) {
        recStatus.textContent = `[RECORDING]${next.recording_episode == null ? '' : ` episode_${String(next.recording_episode).padStart(6, '0')}`}`;
        recStatus.className = 'status recording';
      } else if (next.components.recorder?.running) {
        recStatus.textContent = '[IDLE] Recorder ready';
        recStatus.className = 'status stopped';
      } else {
        recStatus.textContent = '[STOPPED] Recorder Not Running';
        recStatus.className = 'status stopped';
      }
      const lines = next.logs || [];
      document.getElementById('log').textContent = lines.length ? lines.join('\n') : 'No events yet.';
    }

    async function refresh() {
      try {
        const response = await fetch('/api/state', {cache: 'no-store'});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        render(await response.json());
      } catch (error) {
        document.getElementById('busyStatus').textContent = 'Server connection lost';
      }
    }

    masterBtn.addEventListener('click', () => sendAction(state?.master_ready ? 'master_stop' : 'master_start'));
    refreshBtn.addEventListener('click', () => sendAction('status'));
    stopAllBtn.addEventListener('click', () => sendAction('cleanup'));
    serverBtn.addEventListener('click', () => sendAction(state?.server_xr_running ? 'server_stop' : 'server_start'));
    recordBtn.addEventListener('click', () => sendAction(state?.recording_active ? 'record_stop' : 'record_start'));
    componentList.addEventListener('click', (event) => {
      const button = event.target.closest('button[data-component]');
      if (!button || !state) return;
      const component = state.components[button.dataset.component];
      sendAction(component.running ? 'stop' : 'start', button.dataset.component);
    });
    refresh();
    setInterval(refresh, 900);
  </script>
</body>
</html>
"""


def _recording_service_succeeded(result: ActionResult) -> bool:
    if result.returncode != 0:
        return False
    normalized = "".join(result.output.lower().split())
    return "success=false" not in normalized and "success:false" not in normalized


def _extract_episode(output: str) -> int | None:
    for pattern in (
        r"[\"']episode[\"']\s*:\s*(-?\d+)",
        r"\bepisode\s*[=:]\s*(-?\d+)",
    ):
        match = re.search(pattern, output, flags=re.IGNORECASE)
        if match:
            value = int(match.group(1))
            return value if value >= 0 else None
    return None


class WebControlCenter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.lock = threading.RLock()
        self.action_done = threading.Event()
        self.action_done.set()
        self.busy = False
        self.action = ""
        self.last_action = ""
        self.master_ready = False
        self.component_states = {name: False for name, _ in AGX_COMPONENTS}
        self.recording_active = False
        self.recording_episode: int | None = None
        self.left_enable = bool(args.left_enable)
        self.right_enable = bool(args.right_enable)
        self.logs: deque[str] = deque(maxlen=220)
        self.server_xr = LocalManagedProcess(
            Path(args.server_state_dir).expanduser(), "server_xr"
        )
        self.agx = RemoteAgxManager(
            f"{args.agx_user}@{args.agx_host}",
            agx_root=args.agx_root,
            data_dir=args.agx_data_dir,
            task=args.task,
            domain_id=args.domain_id,
            password=os.environ.get("AMMR_AGX_PASSWORD") or DEFAULT_AGX_PASSWORD,
        )
        auth_mode = "password authentication (default credential)"
        if os.environ.get("AMMR_AGX_PASSWORD"):
            auth_mode = "password authentication (environment override)"
        self._log(f"Web control center started ({auth_mode}); click AGX Master first.")
        # Match the former desktop panel: discover an already-running AGX
        # session shortly after the UI opens, without blocking the HTTP
        # listener or the first browser render on an SSH round trip.
        threading.Thread(
            target=self._initial_status,
            name="ammr-control-initial-status",
            daemon=True,
        ).start()

    def _initial_status(self) -> None:
        accepted, _ = self.request("status")
        if not accepted:
            self._log("Initial AGX status check was skipped: another action is running.")

    @property
    def agx_target(self) -> str:
        return f"{self.args.agx_user}@{self.args.agx_host}"

    def _log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.logs.append(f"{timestamp} {message}")

    def _server_command(self) -> str:
        root = shlex.quote(str(PROJECT_ROOT))
        lifting = str(Path(configured_path("ros_ws")) / "install/local_setup.bash")
        arm_selection = " ".join(
            (
                "--left-enable" if self.left_enable else "--no-left-enable",
                "--right-enable" if self.right_enable else "--no-right-enable",
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

    def snapshot(self) -> dict:
        with self.lock:
            component_states = dict(self.component_states)
            return {
                "agx_target": self.agx_target,
                "domain_id": self.args.domain_id,
                "task_display": self.args.task if str(self.args.task).isascii() else "Custom UTF-8 task",
                "master_ready": self.master_ready,
                "components": {
                    name: {"label": label, "running": bool(component_states[name])}
                    for name, label in AGX_COMPONENTS
                },
                "server_xr_running": self.server_xr.running(),
                "left_enable": self.left_enable,
                "right_enable": self.right_enable,
                "recording_active": self.recording_active,
                "recording_episode": self.recording_episode,
                "busy": self.busy,
                "action": self.action,
                "last_action": self.last_action,
                "logs": list(self.logs),
            }

    def request(
        self,
        action: str,
        component: str = "",
        *,
        left_enable: bool | None = None,
        right_enable: bool | None = None,
    ) -> tuple[bool, str]:
        valid_components = {name for name, _ in AGX_COMPONENTS}
        if action in {"start", "stop"} and component not in valid_components:
            return False, f"Unknown component: {component}"
        if action not in {
            "master_start", "master_stop", "cleanup", "status",
            "start", "stop", "record_start", "record_stop",
            "server_start", "server_stop",
        }:
            return False, f"Unknown action: {action}"
        with self.lock:
            if self.busy:
                return False, f"Another action is still running: {self.action}"
            if action == "server_start":
                selected_left = self.left_enable if left_enable is None else left_enable
                selected_right = self.right_enable if right_enable is None else right_enable
                if not isinstance(selected_left, bool) or not isinstance(selected_right, bool):
                    return False, "left_enable and right_enable must be booleans"
                if not selected_left and not selected_right:
                    return False, "Select Left enable and/or Right enable."
                self.left_enable = selected_left
                self.right_enable = selected_right
            if action == "record_start" and (
                not self.master_ready or not self.component_states.get("recorder", False)
            ):
                return False, "Start AGX Master and AGX Recorder first."
            self.busy = True
            self.action = action if not component else f"{action} {component}"
            self.last_action = self.action
            self.action_done.clear()
        if action == "server_start":
            sides = ",".join(
                side for side, selected in (
                    ("left", self.left_enable), ("right", self.right_enable)
                ) if selected
            )
            self._log(f"REQUEST {self.action} teleop_sides={sides}")
        else:
            self._log(f"REQUEST {self.action}")
        threading.Thread(
            target=self._execute,
            args=(action, component),
            name="ammr-control-action",
            daemon=True,
        ).start()
        return True, "accepted"

    def _finish(self) -> None:
        with self.lock:
            self.busy = False
            self.action = ""
        self.action_done.set()

    def _execute(self, action: str, component: str) -> None:
        try:
            if action in {"server_start", "server_stop"}:
                self._execute_server(action)
            else:
                self._execute_remote(action, component)
        except Exception as error:  # defensive boundary for the HTTP worker
            self._log(f"FAILED: {action} {component} ({error})")
        finally:
            self._finish()

    def _execute_server(self, action: str) -> None:
        if action == "server_start":
            if self.server_xr.running():
                self._log("Server XR is already running.")
            else:
                message = self.server_xr.start(self._server_command(), PROJECT_ROOT)
                self._log(message)
        else:
            self._log(self.server_xr.stop())

    def _execute_remote(self, action: str, component: str) -> None:
        result = self.agx.run(action, component)
        if result.output:
            self._log(result.output)
        if result.returncode != 0:
            self._log(f"FAILED: {action} {component} (returncode={result.returncode})")
            return
        if action == "master_start":
            with self.lock:
                self.master_ready = True
        elif action in {"master_stop", "cleanup"}:
            with self.lock:
                self.master_ready = False
                self.component_states = {name: False for name, _ in AGX_COMPONENTS}
                self.recording_active = False
                self.recording_episode = None
        elif action == "start" and component:
            with self.lock:
                self.component_states[component] = True
                if component == "recorder":
                    self.recording_active = False
                    self.recording_episode = None
        elif action == "stop" and component:
            with self.lock:
                self.component_states[component] = False
                if component == "recorder":
                    self.recording_active = False
                    self.recording_episode = None
        elif action == "record_start":
            if not _recording_service_succeeded(result):
                self._log("Recording request rejected; check the AGX Recorder log and /ammr/recording/status.")
            else:
                with self.lock:
                    self.recording_active = True
                    self.recording_episode = _extract_episode(result.output)
        elif action == "record_stop":
            if not _recording_service_succeeded(result):
                self._log("Recording stop was rejected; the episode may still be active.")
            else:
                with self.lock:
                    self.recording_active = False
                    self.recording_episode = None
        elif action == "status":
            self._parse_status(result.output)

    def _parse_status(self, output: str) -> None:
        with self.lock:
            for line in output.splitlines():
                fields = line.split()
                if fields[:2] == ["MASTER", "READY"]:
                    self.master_ready = True
                elif fields[:2] in (["MASTER", "STOPPED"], ["MASTER", "DEGRADED"]):
                    self.master_ready = False
                elif len(fields) >= 3 and fields[0] == "STATUS" and fields[1] in self.component_states:
                    self.component_states[fields[1]] = fields[2] == "RUNNING"

    def shutdown(self) -> None:
        # Do not abandon an in-flight SSH start/stop request when the HTTP
        # process receives Ctrl-C.  Waiting is bounded by the manager's SSH
        # timeout, after which the normal cleanup request is still attempted.
        if not self.action_done.wait(timeout=60.0):
            self._log("Shutdown timeout while waiting for the active action; forcing cleanup.")
        running = self.server_xr.running() or self.master_ready or any(self.component_states.values())
        if not running:
            return
        self._log("Shutdown requested; stopping Server XR and AGX components.")
        self.server_xr.stop()
        try:
            result = self.agx.run("master_stop")
            if result.output:
                self._log(result.output)
        except Exception as error:
            self._log(f"Shutdown cleanup failed: {error}")


class ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, center: WebControlCenter):
        self.center = center
        super().__init__(address, handler)


class ControlRequestHandler(BaseHTTPRequestHandler):
    server: ControlHTTPServer

    def log_message(self, format: str, *args) -> None:
        # The browser polls state frequently; keep the terminal log focused on
        # control actions rather than printing one line per GET request.
        return

    def _send_json(self, payload: dict, status: int = 200) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 8192:
            raise ValueError("request body is empty or too large")
        body = self.rfile.read(length)
        value = json.loads(body.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    def do_GET(self) -> None:
        if self.path == "/" or self.path == "/index.html":
            encoded = WEB_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
            return
        if self.path == "/api/state":
            self._send_json(self.server.center.snapshot())
            return
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        self._send_json({"error": "not found"}, 404)

    def do_POST(self) -> None:
        if self.path != "/api/action":
            self._send_json({"error": "not found"}, 404)
            return
        try:
            body = self._read_json()
            action = str(body.get("action", ""))
            component = str(body.get("component", ""))
            accepted, message = self.server.center.request(
                action,
                component,
                left_enable=body.get("left_enable"),
                right_enable=body.get("right_enable"),
            )
            self._send_json(
                {"accepted": accepted, "message": message},
                202 if accepted else 409,
            )
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self._send_json({"error": str(error)}, 400)


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
        help="XR display mode used by the server teleoperation process.",
    )
    parser.add_argument("--server-state-dir", default="/tmp/ammr_xr_control_center")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP listen address")
    parser.add_argument("--port", type=int, default=8787, help="HTTP listen port")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    center = WebControlCenter(args)
    server = ControlHTTPServer((args.host, args.port), ControlRequestHandler, center)
    print(f"AMMR XR web control center: http://{args.host}:{args.port}", flush=True)
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        print("WARNING: HTTP control is unauthenticated; bind only on a trusted network.", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nStopping control center...", flush=True)
    finally:
        server.shutdown()
        server.server_close()
        center.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
