#!/usr/bin/env python3
"""XR teleoperation entry point for the AMMR real robot."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from sshkeyboard import listen_keyboard, stop_listening

from ammr_teleop.common.kinematics import DualArmKinematics, IKRegularizationConfig
from ammr_teleop.common.event_logging import configure_event_logger
from ammr_teleop.common.paths import configured_path
from ammr_teleop.common.pose_mapper import (
    FixedAbsoluteWristMapper,
    PoseMapperConfig,
    RelativeWristMapper,
    calibrated_translation_scale,
    fixed_xr_neutral_poses,
)
from ammr_teleop.xr.frontend import XRInputFrontend, controller_open_fraction

from ammr_teleop.common.data_contract import (
    LEFT_ARM_SLICE,
    LEFT_GRIPPER_INDEX,
    RIGHT_ARM_SLICE,
    RIGHT_GRIPPER_INDEX,
    arms_from_sim_contract,
    to_sim_contract,
)
from ammr_teleop.common.control_profiles import available_profiles, load_profile
from ammr_teleop.real.hardware_interface import AmmrRealHardwareNode
from ammr_teleop.real.jitter_diagnostics import JitterCsvLogger
from ammr_teleop.real.safety import (
    ChassisLimits,
    apply_deadband,
    chassis_command,
    feedback_ik_seed,
    feedback_is_fresh,
    jerk_limit_command,
)


class Edge:
    def __init__(self) -> None:
        self.previous = False

    def rising(self, value: bool) -> bool:
        current = bool(value)
        result = current and not self.previous
        self.previous = current
        return result


class KeyboardCommands:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.running = threading.Event()
        self.running.set()
        self.toggle_control = threading.Event()
        self.toggle_record = threading.Event()
        self.reset_arms = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if not self.enabled:
            return

        def on_press(key: str) -> None:
            if key == "r":
                self.toggle_control.set()
            elif key == "s":
                self.toggle_record.set()
            elif key == "y":
                self.reset_arms.set()
            elif key == "q":
                self.running.clear()
                stop_listening()

        self.thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": on_press, "until": None, "sequential": False},
            daemon=True,
        )
        self.thread.start()

    def close(self) -> None:
        self.running.clear()
        if self.enabled:
            try:
                stop_listening()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    profiles = available_profiles()
    profile_probe = argparse.ArgumentParser(add_help=False)
    profile_probe.add_argument("--profile", choices=profiles, default=None)
    selected, _ = profile_probe.parse_known_args(argv)
    profile_document = load_profile(selected.profile) if selected.profile else None

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=profiles,
        default=None,
        help="Versioned real-robot control profile; explicit CLI values override it.",
    )
    parser.add_argument(
        "--xr-frequency",
        "--frequency",
        dest="xr_frequency",
        type=float,
        default=50.0,
        help="XR target sampling and IK rate in Hz; --frequency is a compatibility alias.",
    )
    parser.add_argument(
        "--command-frequency",
        type=float,
        default=100.0,
        help="Jerk-limited arm trajectory and ROS command publication rate in Hz.",
    )
    parser.add_argument(
        "--display-frequency",
        type=float,
        default=30.0,
        help="Head-camera XR rendering rate in Hz.",
    )
    parser.add_argument(
        "--recording-frequency",
        type=float,
        default=30.0,
        help="Episode image/joint sampling rate in Hz.",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help=(
            "Enable AGX episode recording controls. Left X/[s] starts an "
            "episode; press it again to stop and save."
        ),
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=0.25,
        help="Maximum arm-feedback age in seconds when feedback timeout protection is enabled.",
    )
    parser.add_argument(
        "--feedback-timeout",
        dest="feedback_timeout",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Automatically pause and emergency-hold when required arm feedback is stale. "
            "Disabled by default for the current remote-DDS deployment; use "
            "--feedback-timeout to enable it."
        ),
    )
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument(
        "--operator-arm-length",
        type=float,
        default=0.65,
        help="Measured shoulder-to-controller reach in meters.",
    )
    parser.add_argument(
        "--robot-teleop-reach",
        type=float,
        default=0.13,
        help="Robot wrist travel produced by one operator arm length, in meters.",
    )
    parser.add_argument(
        "--translation-scale",
        type=float,
        default=None,
        help=(
            "Direct dimensionless translation gain; overrides arm-length "
            "calibration and is preferred by versioned control profiles."
        ),
    )
    parser.add_argument(
        "--wrist-rotation-gain",
        "--rotation-scale",
        dest="rotation_scale",
        type=float,
        default=0.35,
        help="Dimensionless XR wrist-angle gain before task-space IK.",
    )
    parser.add_argument("--ik-motion-regularization", type=float, default=0.20)
    parser.add_argument("--ik-posture-regularization", type=float, default=0.01)
    parser.add_argument("--max-wrist-translation", type=float, default=0.25)
    parser.add_argument("--pose-filter-alpha", type=float, default=0.25)
    parser.add_argument(
        "--pose-filter",
        choices=("ema", "one_euro"),
        default="one_euro",
        help="Task-space pose filter; real-robot default is one_euro.",
    )
    parser.add_argument(
        "--one-euro-min-cutoff",
        type=float,
        default=0.8,
        help="1€ minimum cutoff in Hz; lower values suppress more still-hand noise.",
    )
    parser.add_argument(
        "--one-euro-beta",
        type=float,
        default=5.0,
        help="1€ speed coefficient; higher values reduce motion lag.",
    )
    parser.add_argument(
        "--one-euro-d-cutoff",
        type=float,
        default=0.9,
        help="1€ derivative cutoff in Hz.",
    )
    parser.add_argument(
        "--wrist-mapping",
        choices=("incremental", "absolute"),
        default="incremental",
        help=(
            "incremental calibrates at enable/Right B; absolute uses fixed "
            "TeleVuer neutral and AMMR zero-pose references."
        ),
    )
    parser.add_argument(
        "--arm-reset-duration",
        type=float,
        default=2.5,
        help="Seconds used for the Left Y smooth return to arm zero pose.",
    )
    parser.add_argument("--arm-velocity", type=float, default=0.35)
    parser.add_argument(
        "--arm-acceleration",
        type=float,
        default=0.8,
        help="Maximum arm-joint command acceleration in rad/s^2.",
    )
    parser.add_argument(
        "--arm-jerk",
        type=float,
        default=4.0,
        help="Maximum arm-joint command jerk in rad/s^3.",
    )
    parser.add_argument(
        "--ik-feedback-seed-max-error",
        type=float,
        default=0.12,
        help=(
            "Maximum per-joint command-feedback correction (rad) accepted "
            "when using real feedback as the IK warm start."
        ),
    )
    parser.add_argument("--gripper-velocity", type=float, default=0.10)
    parser.add_argument(
        "--gripper-acceleration",
        type=float,
        default=0.4,
        help="Maximum gripper-width command acceleration in m/s^2.",
    )
    parser.add_argument(
        "--gripper-jerk",
        type=float,
        default=2.0,
        help="Maximum gripper-width command jerk in m/s^3.",
    )
    parser.add_argument("--gripper-effort", type=float, default=1.0)
    parser.add_argument("--max-linear-x", type=float, default=0.20)
    parser.add_argument("--max-linear-y", type=float, default=0.15)
    parser.add_argument("--max-angular-z", type=float, default=0.30)
    parser.add_argument("--stick-deadband", type=float, default=0.08)
    parser.add_argument("--lift-speed", type=int, default=40, help="Maximum mode=-4 lift speed in mm/s")
    parser.add_argument("--lift-command-period", type=float, default=0.20)
    parser.add_argument("--camera-max-age", type=float, default=0.20)
    parser.add_argument("--display-mode", choices=("pass-through", "immersive", "ego"), default="pass-through")
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument(
        "--image-stream",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Subscribe to all three camera streams (head and both wrists). "
            "The default ZMQ path keeps only the newest frame per camera."
        ),
    )
    parser.add_argument(
        "--image-transport",
        choices=("raw", "compressed", "zmq"),
        default="zmq",
        help="Camera transport used when an image stream is enabled.",
    )
    parser.add_argument(
        "--head-image-stream",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Compatibility option: subscribe to the head camera only and render it "
            "in XR. --image-transport selects raw or compressed transport."
        ),
    )
    parser.add_argument("--head-topic", default="/head/color/image_raw")
    parser.add_argument("--left-wrist-topic", default="/left/color/image_raw")
    parser.add_argument("--right-wrist-topic", default="/right/color/image_raw")
    parser.add_argument("--head-zmq-endpoint", default="tcp://192.168.3.99:55555")
    parser.add_argument("--left-wrist-zmq-endpoint", default="tcp://192.168.3.99:55556")
    parser.add_argument("--right-wrist-zmq-endpoint", default="tcp://192.168.3.99:55557")
    parser.add_argument(
        "--data-dir",
        default=str(Path(configured_path("data_root")) / "ammr_real_xr"),
        help=(
            "Compatibility option only; the AGX recorder launch command owns "
            "the real dataset path."
        ),
    )
    parser.add_argument(
        "--recorder-service",
        default="/ammr/recording/set_enabled",
        help="AGX-side std_srvs/SetBool episode recorder service.",
    )
    parser.add_argument(
        "--recorder-status-topic",
        default="/ammr/recording/status",
        help="AGX-side JSON recording status topic.",
    )
    parser.add_argument(
        "--recorder-service-timeout",
        type=float,
        default=40.0,
        help="Maximum time to wait for an AGX recorder start/save response.",
    )
    parser.add_argument(
        "--jitter-log",
        type=Path,
        default=None,
        help=(
            "Optional CSV path for raw XR, task targets, IK, command, and "
            "feedback telemetry. It does not change control behavior."
        ),
    )
    parser.add_argument("--task", default="Operate AMMR dual arms with XR.")
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "4")))
    parser.add_argument(
        "--enable-hardware-output",
        action="store_true",
        help="Actually publish commands. Without this flag the process is read-only/dry-run.",
    )
    parser.add_argument(
        "--hardware-components",
        nargs="+",
        choices=("left_arm", "right_arm", "chassis", "lift"),
        default=[],
        help="Hardware outputs to enable; requires --enable-hardware-output.",
    )
    parser.add_argument(
        "--left-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Allow the left arm to follow the left XR controller. Use "
            "--no-left-enable for right-only teleoperation; when left_arm is "
            "still a hardware component, it is commanded smoothly to zero and "
            "continues publishing state/action for recording."
        ),
    )
    parser.add_argument(
        "--right-enable",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Allow the right arm to follow the right XR controller. Use "
            "--no-right-enable for left-only teleoperation; when right_arm is "
            "still a hardware component, it is commanded smoothly to zero and "
            "continues publishing state/action for recording."
        ),
    )
    parser.add_argument(
        "--swap-controller-poses",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Swap the left/right controller pose sources before arm mapping. "
            "This affects only poses, never triggers or buttons."
        ),
    )
    parser.add_argument(
        "--diagnose-controller-poses",
        action="store_true",
        help=(
            "Log which TeleVuer pose stream is moving. Run without hardware "
            "output, move one physical controller at a time, then use the "
            "result to decide whether --swap-controller-poses is needed."
        ),
    )
    parser.add_argument(
        "--keyboard",
        action=argparse.BooleanOptionalAction,
        default=sys.stdin.isatty(),
    )
    if profile_document is not None:
        profile_parameters = profile_document["parameters"]
        known_destinations = {action.dest for action in parser._actions}
        unknown = sorted(set(profile_parameters) - known_destinations)
        if unknown:
            raise ValueError(
                f"control profile {selected.profile} contains unknown parameters: {unknown}"
            )
        parser.set_defaults(**profile_parameters)
    args = parser.parse_args(argv)
    args.profile_version = (
        profile_document.get("version") if profile_document is not None else None
    )
    args.profile_description = (
        profile_document.get("description") if profile_document is not None else None
    )
    return args


def validate_args(args: argparse.Namespace) -> None:
    positive = (
        "xr_frequency", "command_frequency", "display_frequency",
        "recording_frequency", "state_timeout", "startup_timeout", "operator_arm_length",
        "robot_teleop_reach", "rotation_scale",
        "max_wrist_translation", "pose_filter_alpha", "arm_velocity",
        "arm_acceleration", "gripper_acceleration", "arm_jerk", "gripper_jerk",
        "ik_feedback_seed_max_error",
        "one_euro_min_cutoff", "one_euro_d_cutoff",
        "gripper_velocity", "gripper_effort", "max_linear_x", "max_linear_y",
        "max_angular_z", "lift_speed", "lift_command_period", "camera_max_age",
        "recorder_service_timeout",
        "image_width", "image_height", "arm_reset_duration",
    )
    for name in positive:
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if args.command_frequency < args.xr_frequency:
        raise ValueError("command_frequency must be greater than or equal to xr_frequency")
    if args.translation_scale is not None and (
        not math.isfinite(args.translation_scale) or args.translation_scale <= 0.0
    ):
        raise ValueError("translation_scale must be finite and positive")
    for name in ("ik_motion_regularization", "ik_posture_regularization"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    if not 0.0 < args.pose_filter_alpha <= 1.0:
        raise ValueError("pose_filter_alpha must be in (0, 1]")
    if not math.isfinite(args.one_euro_beta) or args.one_euro_beta < 0.0:
        raise ValueError("one_euro_beta must be finite and non-negative")
    if not 0.0 <= args.stick_deadband < 1.0:
        raise ValueError("stick_deadband must be in [0, 1)")
    if args.enable_hardware_output and not args.hardware_components:
        raise ValueError(
            "--enable-hardware-output requires at least one --hardware-components value"
        )
    if args.hardware_components and not args.enable_hardware_output:
        raise ValueError("--hardware-components requires --enable-hardware-output")
    hardware_arm_sides = {
        side for side in ("left", "right")
        if f"{side}_arm" in args.hardware_components
    }
    requested_arm_sides = {
        side for side, selected in (
            ("left", args.left_enable),
            ("right", args.right_enable),
        )
        if selected
    }
    if hardware_arm_sides and not hardware_arm_sides.intersection(requested_arm_sides):
        raise ValueError(
            "at least one selected hardware arm must be teleoperated; enable "
            "--left-enable and/or --right-enable"
        )
    if args.diagnose_controller_poses and args.enable_hardware_output:
        raise ValueError("--diagnose-controller-poses is read-only and cannot enable hardware output")


def _metadata_value(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _metadata_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_metadata_value(item) for item in value]
    return value


def _git_state() -> dict[str, str | bool | None]:
    project_root = Path(__file__).resolve().parents[3]
    result: dict[str, str | bool | None] = {"revision": None, "dirty": None}
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        result["revision"] = revision.stdout.strip()
        result["dirty"] = bool(status.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return result


def _run_metadata(args: argparse.Namespace, translation_scale: float) -> dict:
    return {
        "schema_version": 1,
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "profile": {
            "name": args.profile,
            "version": args.profile_version,
            "description": args.profile_description,
        },
        "effective_translation_gain": float(translation_scale),
        "resolved_parameters": {
            key: _metadata_value(value)
            for key, value in vars(args).items()
            if key not in {"profile_version", "profile_description"}
        },
        "command_line": [sys.executable, *sys.argv],
        "git": _git_state(),
    }


def main() -> int:
    args = parse_args()
    validate_args(args)
    image_stream_enabled = bool(args.image_stream or args.head_image_stream)
    translation_scale = calibrated_translation_scale(
        args.operator_arm_length,
        args.robot_teleop_reach,
        args.translation_scale,
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    target_mode = os.environ.get("AMMR_TARGET_MODE", "real").strip().lower()
    sim_target_hold = None
    if target_mode == "sim":
        from ammr_teleop.common.sim_target_hold import SimTargetHold
        sim_target_hold = SimTargetHold()
    events = configure_event_logger(target_mode)
    if sim_target_hold is not None:
        events.info(
            "SIM_ARM_POLICY",
            "Legacy simulation arm policy: command seed, per-wrist target hold; "
            "ik-feedback-seed-max-error is not used in simulation",
        )
    events.info(
        "SESSION_START",
        "XR teleoperation starting",
        session=events.session_id,
        pid=os.getpid(),
        ros_domain_id=args.domain_id,
        log=events.path,
    )
    events.info(
        "CONFIG",
        profile=args.profile or "none",
        mapping=args.wrist_mapping,
        hardware=args.enable_hardware_output,
        components=",".join(args.hardware_components) or "none",
        teleop_sides=",".join(
            side for side, selected in (
                ("left", args.left_enable),
                ("right", args.right_enable),
            )
            if selected
        ) or "none",
        translation_gain=translation_scale,
        rotation_gain=args.rotation_scale,
        xr_hz=args.xr_frequency,
        command_hz=args.command_frequency,
        recording_hz=args.recording_frequency,
        recording_control="enabled" if args.record else "disabled",
        recorder="agx",
        recorder_service=args.recorder_service,
        feedback_timeout=args.feedback_timeout,
        state_timeout=args.state_timeout,
        image_stream=args.image_stream,
        head_image_stream=args.head_image_stream,
        image_transport=args.image_transport if image_stream_enabled else "off",
    )
    rclpy.init()
    hardware_arm_sides = {
        side
        for side in ("left", "right")
        if f"{side}_arm" in args.hardware_components
    }
    requested_arm_sides = {
        side for side, selected in (
            ("left", args.left_enable),
            ("right", args.right_enable),
        )
        if selected
    }
    # Hardware components describe which physical/virtual arm drivers are
    # online.  The independent enable flags only select which arm follows XR.
    # Keeping both hardware arms required and published is what lets a
    # single-arm episode retain synchronized 14-D state/action samples.
    required_arm_sides = hardware_arm_sides or (
        requested_arm_sides if not args.enable_hardware_output else set()
    )
    control_arm_sides = required_arm_sides.intersection(requested_arm_sides)
    arm_teleop_enabled = bool(control_arm_sides)
    node = AmmrRealHardwareNode(
        hardware_output=args.enable_hardware_output,
        enabled_components=set(args.hardware_components),
        required_arm_sides=required_arm_sides,
        head_topic=args.head_topic,
        left_wrist_topic=args.left_wrist_topic,
        right_wrist_topic=args.right_wrist_topic,
        head_zmq_endpoint=args.head_zmq_endpoint,
        left_wrist_zmq_endpoint=args.left_wrist_zmq_endpoint,
        right_wrist_zmq_endpoint=args.right_wrist_zmq_endpoint,
        subscribe_head_camera=image_stream_enabled,
        subscribe_wrist_cameras=args.image_stream,
        image_transport=args.image_transport if image_stream_enabled else "raw",
        recorder_service=args.recorder_service,
        recorder_status_topic=args.recorder_status_topic,
    )
    if image_stream_enabled and args.image_transport == "raw":
        node.get_logger().warning(
            "Raw three-camera image streaming is enabled; use "
            "--image-transport compressed to reduce DDS bandwidth"
        )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin, daemon=True)
    executor_thread.start()
    keyboard = KeyboardCommands(args.keyboard)
    jitter_logger: JitterCsvLogger | None = None
    xr: XRInputFrontend | None = None

    def request_stop(_signum=None, _frame=None) -> None:
        keyboard.running.clear()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        if args.diagnose_controller_poses:
            xr = XRInputFrontend(
                input_mode="controller",
                display_mode=args.display_mode,
                image_shape=(args.image_height, args.image_width),
                display_fps=args.display_frequency,
            )
            keyboard.start()
            node.get_logger().info(
                "Pose diagnosis active (read-only): move one physical controller at a time; "
                "Right A or q exits"
            )
            previous_left: np.ndarray | None = None
            previous_right: np.ndarray | None = None
            last_log = 0.0
            period = 1.0 / args.xr_frequency
            quit_edge = Edge()
            while keyboard.running.is_set() and rclpy.ok():
                started = time.monotonic()
                tele = xr.sample()
                left = tele.left_wrist_pose[:3, 3].copy()
                right = tele.right_wrist_pose[:3, 3].copy()
                if previous_left is not None and started - last_log >= 0.2:
                    left_delta = float(np.linalg.norm(left - previous_left))
                    right_delta = float(np.linalg.norm(right - previous_right))
                    if max(left_delta, right_delta) >= 0.005:
                        node.get_logger().info(
                            "Controller pose activity: "
                            f"TeleVuer left Δ={left_delta:.3f}m, right Δ={right_delta:.3f}m"
                        )
                        last_log = started
                previous_left, previous_right = left, right
                if quit_edge.rising(tele.right_ctrl_aButton):
                    keyboard.running.clear()
                time.sleep(max(0.0, period - (time.monotonic() - started)))
            return 0
        if (
            args.enable_hardware_output
            and "lift" in args.hardware_components
            and node.lift_client is None
        ):
            events.error("LIFT_UNAVAILABLE", "lifting_msg_pkg is not available")
            node.get_logger().error(
                "Lift output requested but lifting_msg_pkg cannot be imported; "
                "load the installed real-robot ROS environment first"
            )
            return 3
        if arm_teleop_enabled:
            events.info(
                "ROS_WAIT",
                "waiting for complete arm/gripper feedback",
                sides=",".join(sorted(required_arm_sides)),
            )
            node.get_logger().info(
                "Waiting for complete arm/gripper feedback from "
                + ", ".join(sorted(required_arm_sides))
                + "..."
            )
            deadline = time.monotonic() + args.startup_timeout
            state = None
            while time.monotonic() < deadline and rclpy.ok():
                state, _ = node.state()
                if state is not None:
                    break
                time.sleep(0.05)
            if state is None:
                events.error(
                    "ROS_TIMEOUT",
                    "complete real-robot feedback did not arrive",
                    timeout_s=args.startup_timeout,
                )
                node.get_logger().error("Timed out waiting for complete 14-D real-robot feedback")
                return 2

            events.info("ROS_READY", "complete real-robot feedback received", dimensions=14)
            ik_load_started = time.monotonic()
            events.info("IK_LOADING", "loading dual-arm kinematics")
            kinematics = DualArmKinematics(
                IKRegularizationConfig(
                    posture_weight=args.ik_posture_regularization,
                    motion_weight=args.ik_motion_regularization,
                ),
            )
            events.info(
                "IK_READY",
                "dual-arm kinematics ready",
                load_ms=(time.monotonic() - ik_load_started) * 1000.0,
            )
            mapper_config = PoseMapperConfig(
                translation_scale=translation_scale,
                rotation_scale=args.rotation_scale,
                max_translation=args.max_wrist_translation,
                filter_alpha=args.pose_filter_alpha,
                filter_mode=args.pose_filter,
                one_euro_min_cutoff=args.one_euro_min_cutoff,
                one_euro_beta=args.one_euro_beta,
                one_euro_d_cutoff=args.one_euro_d_cutoff,
                translation_deadband=0.01,
                rotation_deadband=math.radians(1.5),
            )
            if args.wrist_mapping == "incremental":
                mapper = RelativeWristMapper(mapper_config)
            else:
                # Keep these references independent of feedback and Right B so
                # absolute mode remains reproducible between operator sessions.
                zero_real = np.zeros_like(state)
                zero_robot = kinematics.forward(to_sim_contract(zero_real))
                mapper = FixedAbsoluteWristMapper(
                    xr_reference=fixed_xr_neutral_poses("controller"),
                    robot_reference=zero_robot,
                    config=mapper_config,
                )
        else:
            state = np.zeros(14, dtype=np.float64)
            kinematics = None
            mapper = None
            node.get_logger().info(
                "No arm output selected; starting chassis/lift XR control without arm feedback"
            )
        xr = XRInputFrontend(
            input_mode="controller",
            display_mode=args.display_mode,
            image_shape=(args.image_height, args.image_width),
            display_fps=args.display_frequency,
        )
        keyboard.start()
        if args.jitter_log is not None:
            if not arm_teleop_enabled:
                node.get_logger().warning(
                    "--jitter-log requested without arm output; only arm-enabled cycles contain IK telemetry"
                )
            jitter_logger = JitterCsvLogger(args.jitter_log)
            node.get_logger().info(f"Jitter diagnostic CSV: {jitter_logger.path}")
            metadata_path = jitter_logger.write_metadata(
                _run_metadata(args, translation_scale)
            )
            node.get_logger().info(f"Run metadata JSON: {metadata_path}")
        node.get_logger().info(
            "Ready: Right B/[r] enable-pause; "
            f"Left X/[s]={'record' if args.record else 'record-disabled'}; "
            "Left Y/[y] returns enabled arms to zero; "
            "both stick buttons hold; Right A/[q] exit; "
            f"profile={args.profile or 'none'}; "
            f"wrist-mapping={args.wrist_mapping}; "
            f"translation-gain={translation_scale:.3f} "
            f"({'direct override' if args.translation_scale is not None else f'{args.robot_teleop_reach:.3f}m/{args.operator_arm_length:.3f}m'}); "
            f"wrist-rotation-gain={args.rotation_scale:.3f}; "
            f"IK regularization motion={args.ik_motion_regularization:.3f}, "
            f"posture={args.ik_posture_regularization:.3f}; "
            f"feedback-timeout={'on' if args.feedback_timeout else 'off'} "
            f"({args.state_timeout:.3f}s); "
            f"rates XR/IK={args.xr_frequency:.1f}Hz, "
            f"command={args.command_frequency:.1f}Hz, "
            f"display={args.display_frequency:.1f}Hz, "
            f"recording={args.recording_frequency:.1f}Hz; "
            f"image-stream={'all' if args.image_stream else ('head' if args.head_image_stream else 'off')}"
            f"/{args.image_transport if image_stream_enabled else 'off'}; "
            f"teleop-arms={','.join(sorted(control_arm_sides)) or 'none'}; "
            f"hardware-arms={','.join(sorted(hardware_arm_sides)) or 'none'}; "
            f"pose-filter={args.pose_filter}; "
            f"arm-jerk={args.arm_jerk:.3f}; "
            f"controller poses={'swapped' if args.swap_controller_poses else 'direct'}"
        )
        events.info(
            "READY",
            (
                "Right-B=control Left-X=record Left-Y=reset Right-A=exit"
                if args.record
                else "Right-B=control Left-Y=reset Right-A=exit (recording disabled)"
            ),
            output="hardware" if args.enable_hardware_output else "dry-run",
            recorder="enabled" if args.record else "disabled",
        )

        command = state.copy()
        desired = command.copy()
        command_velocity = np.zeros(14, dtype=np.float64)
        command_acceleration = np.zeros(14, dtype=np.float64)
        enabled = False
        recording = False
        recorder_future = None
        recorder_target: bool | None = None
        recorder_request_started = 0.0
        deferred_arm_reset = False

        def begin_recorder_request(target: bool) -> bool:
            nonlocal recorder_future, recorder_target, recorder_request_started
            if recorder_future is not None:
                return False
            try:
                recorder_future = node.request_recording(target)
            except RuntimeError as error:
                node.get_logger().warning(str(error))
                events.warning("REC_REJECT", str(error))
                return False
            recorder_target = bool(target)
            recorder_request_started = time.monotonic()
            return True

        enable_edge, record_edge, quit_edge, hold_edge, arm_reset_edge = (
            Edge(), Edge(), Edge(), Edge(), Edge()
        )
        chassis_limits = ChassisLimits(
            args.max_linear_x, args.max_linear_y, args.max_angular_z, args.stick_deadband
        )
        period = 1.0 / args.command_frequency
        ik_period = 1.0 / args.xr_frequency
        display_period = 1.0 / args.display_frequency
        previous_time = time.monotonic()
        last_xr_sample = -math.inf
        last_ik_update = -math.inf
        next_display = previous_time
        target_raw: tuple[np.ndarray, np.ndarray] | None = None
        target_filtered: tuple[np.ndarray, np.ndarray] | None = None
        ik_seed: np.ndarray | None = None
        ik_solution: np.ndarray | None = None
        tele = xr.sample()
        events.info("XR_CONNECTED", "first XR sample received", input="controller")
        last_lift_send = 0.0
        last_lift_speed: int | None = None
        consecutive_ik_failures = 0
        inactive_left_target: np.ndarray | None = None
        inactive_right_target: np.ndarray | None = None
        arm_reset_active = False
        arm_reset_start_time: float | None = None
        arm_reset_start_positions: np.ndarray | None = None

        while keyboard.running.is_set() and rclpy.ok():
            started = time.monotonic()
            dt = min(max(started - previous_time, 1e-4), 0.1)
            previous_time = started

            if recorder_future is not None:
                if recorder_future.done():
                    target = bool(recorder_target)
                    try:
                        response = recorder_future.result()
                    except Exception as error:
                        response = None
                        node.get_logger().error(
                            f"AGX recorder service call failed: {error}"
                        )
                        events.error("REC_SERVICE_FAILED", str(error))
                    payload = {}
                    if response is not None:
                        try:
                            decoded = json.loads(response.message)
                            if isinstance(decoded, dict):
                                payload = decoded
                        except (TypeError, ValueError):
                            payload = {"message": response.message}
                    if response is not None and response.success:
                        if target:
                            recording = True
                            episode = int(payload.get("episode", -1))
                            remote_fps = float(
                                payload.get("fps", args.recording_frequency)
                            )
                            node.get_logger().info(
                                f"AGX recording episode_{episode:06d}; "
                                "base and lift locked"
                            )
                            events.info(
                                "REC_START",
                                "AGX recorder active; base and lift locked",
                                episode=f"{episode:06d}",
                                fps=remote_fps,
                                recorder="agx",
                            )
                        else:
                            recording = False
                            path = payload.get("path") or "unknown"
                            node.get_logger().info(f"AGX recording saved: {path}")
                            events.info(
                                "REC_SAVED",
                                "AGX recording saved",
                                episode=payload.get("episode"),
                                frames=payload.get("frames"),
                                path=path,
                            )
                    elif response is not None:
                        message = payload.get("error") or response.message
                        node.get_logger().warning(
                            f"AGX recorder request rejected: {message}"
                        )
                        events.warning("REC_REJECT", str(message))
                        if not target and payload.get("state") != "recording":
                            recording = False
                    recorder_future = None
                    recorder_target = None
                elif started - recorder_request_started > args.recorder_service_timeout:
                    status = node.recorder_status()
                    recording = status.get("state") == "recording"
                    node.get_logger().error(
                        "Timed out waiting for AGX recorder response; "
                        f"reported state={status.get('state', 'unknown')}"
                    )
                    events.error(
                        "REC_TIMEOUT",
                        "AGX recorder service response timed out",
                        state=status.get("state", "unknown"),
                    )
                    recorder_future = None
                    recorder_target = None

            state, timestamps = node.state()
            if arm_teleop_enabled and args.feedback_timeout and (
                state is None
                or not feedback_is_fresh(started, timestamps, args.state_timeout)
            ):
                if enabled:
                    node.get_logger().error("Arm feedback timeout; entering emergency hold")
                    events.error("FEEDBACK_TIMEOUT", "arm feedback stale; entering hold")
                    enabled = False
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    mapper.clear()
                    node.set_control_gate(False)
                    node.emergency_hold()
                if recording and recorder_future is None:
                    if begin_recorder_request(False):
                        node.get_logger().warning(
                            "Stopping AGX recording after feedback timeout"
                        )
                        events.warning(
                            "REC_STOP",
                            "AGX recording stop requested after feedback timeout",
                        )
                time.sleep(period)
                continue

            xr_sample_due = started - last_xr_sample >= ik_period
            if xr_sample_due:
                tele = xr.sample()
                last_xr_sample = started
            # In the deployed TeleVuer controller stream, its pose labels are
            # independent from buttons/triggers.  Keep this mapping explicit:
            # some WebXR runtimes report the physical controller identities
            # correctly, while others have been observed to reverse pose IDs.
            if args.swap_controller_poses:
                left_pose, right_pose = tele.right_wrist_pose, tele.left_wrist_pose
            else:
                left_pose, right_pose = tele.left_wrist_pose, tele.right_wrist_pose
            toggle = keyboard.toggle_control.is_set() or enable_edge.rising(tele.right_ctrl_bButton)
            keyboard.toggle_control.clear()
            toggle_record = keyboard.toggle_record.is_set() or record_edge.rising(tele.left_ctrl_aButton)
            keyboard.toggle_record.clear()
            reset_requested = keyboard.reset_arms.is_set() or arm_reset_edge.rising(
                tele.left_ctrl_bButton
            )
            keyboard.reset_arms.clear()
            if deferred_arm_reset and recording and recorder_future is None:
                begin_recorder_request(False)
            if deferred_arm_reset and not recording and recorder_future is None:
                reset_requested = True
            if quit_edge.rising(tele.right_ctrl_aButton):
                events.info("QUIT_REQUEST", "Right A requested shutdown")
                keyboard.running.clear()

            if hold_edge.rising(
                tele.left_ctrl_thumbstick and tele.right_ctrl_thumbstick
            ):
                if enabled:
                    node.get_logger().warning("Emergency hold requested by both stick buttons")
                events.warning("EMERGENCY_HOLD", "both stick buttons pressed")
                enabled = False
                command_velocity.fill(0.0)
                command_acceleration.fill(0.0)
                if mapper is not None:
                    mapper.clear()
                arm_reset_active = False
                node.set_control_gate(False)
                node.emergency_hold()

            if toggle:
                if enabled:
                    enabled = False
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    if mapper is not None:
                        mapper.clear()
                    arm_reset_active = False
                    node.set_control_gate(False)
                    node.emergency_hold()
                    node.get_logger().info("XR real-robot control PAUSED")
                    events.info("CONTROL_PAUSE", "XR control paused")
                else:
                    command = command.copy() if sim_target_hold is not None else state.copy()
                    desired = command.copy()
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    last_ik_update = -math.inf
                    if arm_teleop_enabled:
                        assert kinematics is not None
                        assert mapper is not None
                        sim_state = to_sim_contract(command)
                        kinematics.reset_solver_history(sim_state)
                        robot_left, robot_right = kinematics.forward(sim_state)
                        if args.wrist_mapping == "incremental":
                            mapper.calibrate(
                                left_pose, right_pose, robot_left, robot_right
                            )
                        else:
                            # FixedAbsoluteWristMapper.clear only drops filter
                            # history; its neutral references stay unchanged.
                            mapper.clear()
                        inactive_left_target = robot_left.copy()
                        inactive_right_target = robot_right.copy()
                        if control_arm_sides != {"left", "right"}:
                            zero_left, zero_right = kinematics.forward(
                                to_sim_contract(np.zeros(14, dtype=np.float64))
                            )
                            if "left" not in control_arm_sides:
                                inactive_left_target = zero_left
                            if "right" not in control_arm_sides:
                                inactive_right_target = zero_right
                        mapping = (
                            "calibrated from feedback"
                            if args.wrist_mapping == "incremental"
                            else "with fixed absolute wrist mapping"
                        )
                    else:
                        mapping = "for chassis/lift (no arm feedback required)"
                    node.set_control_gate(True)
                    enabled = True
                    arm_reset_active = False
                    consecutive_ik_failures = 0
                    mode = "hardware" if args.enable_hardware_output else "dry-run"
                    node.get_logger().info(f"XR control ENABLED, {mapping} ({mode})")
                    events.info(
                        "CONTROL_ENABLE",
                        "XR control enabled",
                        mapping=args.wrist_mapping,
                        output=mode,
                        components=",".join(args.hardware_components) or "none",
                        teleop_sides=",".join(sorted(control_arm_sides)),
                    )

            if reset_requested:
                if not arm_teleop_enabled:
                    node.get_logger().warning("Left Y arm reset ignored: no arm output is enabled")
                    events.warning("ARM_RESET_REJECT", "no arm output is enabled")
                elif not enabled:
                    node.get_logger().warning("Left Y arm reset ignored while XR control is paused")
                    events.warning("ARM_RESET_REJECT", "XR control is paused")
                elif recording or recorder_future is not None:
                    deferred_arm_reset = True
                    if recording and recorder_future is None:
                        begin_recorder_request(False)
                    node.get_logger().warning(
                        "Left Y arm reset deferred until AGX recording is saved"
                    )
                    events.warning(
                        "ARM_RESET_DEFER",
                        "waiting for AGX recorder to finish saving",
                    )
                else:
                    deferred_arm_reset = False
                    reset_command = command.copy()
                    for side, arm_slice in (
                        ("left", LEFT_ARM_SLICE),
                        ("right", RIGHT_ARM_SLICE),
                    ):
                        if side in control_arm_sides:
                            reset_command[arm_slice] = 0.0
                    kinematics.reset_solver_history(to_sim_contract(reset_command))
                    if args.wrist_mapping == "incremental":
                        reset_left, reset_right = kinematics.forward(
                            to_sim_contract(reset_command)
                        )
                        mapper.calibrate(left_pose, right_pose, reset_left, reset_right)
                    else:
                        mapper.clear()
                    arm_reset_active = True
                    arm_reset_start_time = started
                    arm_reset_start_positions = command.copy()
                    desired = command.copy()
                    consecutive_ik_failures = 0
                    node.get_logger().info(
                        "Arm reset requested by Left Y; moving enabled arms to zero "
                        f"pose over {args.arm_reset_duration:.1f}s"
                    )
                    events.info(
                        "ARM_RESET_START",
                        "Left Y requested enabled arms to zero",
                        duration_s=args.arm_reset_duration,
                    )

            if toggle_record and not args.record:
                node.get_logger().warning(
                    "Recording is disabled; restart teleoperation with --record"
                )
                events.warning(
                    "REC_DISABLED",
                    "recording is disabled; restart teleoperation with --record",
                )
                toggle_record = False

            if toggle_record:
                events.info("REC_TOGGLE", "Left X/[s] requested recorder toggle")
                if recorder_future is not None:
                    node.get_logger().warning(
                        "AGX recorder request is already in progress"
                    )
                    events.warning("REC_BUSY", "AGX recorder request in progress")
                elif recording:
                    if begin_recorder_request(False):
                        node.get_logger().info(
                            "Stopping AGX recording; waiting for episode save"
                        )
                        events.info(
                            "REC_STOP",
                            "AGX recorder is draining and saving the episode",
                        )
                elif not enabled:
                    node.get_logger().warning("Recording requires enabled XR arm control")
                    events.warning("REC_REJECT", "XR arm control is not enabled")
                elif args.enable_hardware_output and not {
                    "left_arm", "right_arm"
                }.issubset(args.hardware_components):
                    node.get_logger().warning(
                        "Hardware recording requires both left_arm and right_arm outputs"
                    )
                    events.warning(
                        "REC_REJECT",
                        "hardware recording requires both arm outputs",
                    )
                elif not node.recorder_service_ready():
                    node.get_logger().warning(
                        f"AGX recorder service unavailable: {args.recorder_service}"
                    )
                    events.warning(
                        "REC_REJECT",
                        "AGX recorder service unavailable",
                        service=args.recorder_service,
                    )
                else:
                    if begin_recorder_request(True):
                        node.publish_chassis(np.zeros(3))
                        node.stop_lift()
                        node.get_logger().info(
                            "Starting AGX recording; base and lift locked"
                        )
                        events.info(
                            "REC_STARTING",
                            "waiting for AGX recorder; base and lift locked",
                        )

            if sim_target_hold is not None and (not enabled or arm_reset_active):
                sim_target_hold.clear()
            if enabled and arm_teleop_enabled:
                assert kinematics is not None
                assert mapper is not None
                ik_success: bool | None = None
                ik_solve_ms: float | None = None
                if arm_reset_active:
                    assert arm_reset_start_time is not None
                    assert arm_reset_start_positions is not None
                    progress = min(
                        max((started - arm_reset_start_time) / args.arm_reset_duration, 0.0),
                        1.0,
                    )
                    smooth = progress * progress * (3.0 - 2.0 * progress)
                    for side, arm_slice in (
                        ("left", LEFT_ARM_SLICE),
                        ("right", RIGHT_ARM_SLICE),
                    ):
                        if side in control_arm_sides:
                            desired[arm_slice] = (
                                arm_reset_start_positions[arm_slice] * (1.0 - smooth)
                            )
                elif xr_sample_due or started - last_ik_update >= ik_period:
                    last_ik_update = started
                    left_target, right_target = mapper.map(left_pose, right_pose)
                    target_raw = mapper.last_raw_targets
                    target_filtered = (left_target.copy(), right_target.copy())
                    if "left" not in control_arm_sides:
                        assert inactive_left_target is not None
                        left_target = inactive_left_target
                    if "right" not in control_arm_sides:
                        assert inactive_right_target is not None
                        right_target = inactive_right_target
                    ik_seed = command.copy() if sim_target_hold is not None else feedback_ik_seed(
                        command,
                        state,
                        control_arm_sides,
                        args.ik_feedback_seed_max_error,
                    )
                    held = (False, False) if sim_target_hold is None else sim_target_hold.held(
                        left_target, right_target
                    )
                    if all(held):
                        solved = to_sim_contract(desired)
                    else:
                        solve_started = time.monotonic()
                        solved = kinematics.solve(
                            left_target, right_target, to_sim_contract(ik_seed)
                        )
                        ik_solve_ms = (time.monotonic() - solve_started) * 1000.0
                    if solved is None:
                        ik_success = False
                        consecutive_ik_failures += 1
                        if consecutive_ik_failures == 1 or consecutive_ik_failures % 30 == 0:
                            node.get_logger().warning(
                                f"IK rejected target ({consecutive_ik_failures} consecutive): {kinematics.last_failure_reason}"
                            )
                        if consecutive_ik_failures >= math.ceil(3.0 * args.xr_frequency):
                            enabled = False
                            command_velocity.fill(0.0)
                            command_acceleration.fill(0.0)
                            mapper.clear()
                            node.set_control_gate(False)
                            node.emergency_hold()
                            node.get_logger().error("IK failure threshold reached; control paused")
                            events.error(
                                "IK_HOLD",
                                "IK failure threshold reached; control paused",
                                consecutive=consecutive_ik_failures,
                            )
                    else:
                        next_desired = arms_from_sim_contract(solved, desired)
                        for side_held, arm_slice in zip(held, (LEFT_ARM_SLICE, RIGHT_ARM_SLICE)):
                            if side_held:
                                next_desired[arm_slice] = desired[arm_slice]
                        # A non-teleoperated hardware arm is not merely held at
                        # its starting pose: drive its six joints to the agreed
                        # zero pose through the same jerk-limited command path.
                        if "left" not in control_arm_sides:
                            next_desired[LEFT_ARM_SLICE] = 0.0
                        if "right" not in control_arm_sides:
                            next_desired[RIGHT_ARM_SLICE] = 0.0
                        desired = next_desired
                        if sim_target_hold is not None:
                            sim_target_hold.accept(left_target, right_target)
                        ik_solution = desired.copy()
                        ik_success = None if all(held) else True
                        consecutive_ik_failures = 0

                # Enforce the inactive side every command cycle, including an
                # arm-reset cycle before the first IK update.
                if "left" not in control_arm_sides:
                    desired[LEFT_ARM_SLICE] = 0.0
                if "right" not in control_arm_sides:
                    desired[RIGHT_ARM_SLICE] = 0.0
                desired[LEFT_GRIPPER_INDEX] = (
                    0.1 * controller_open_fraction(tele.left_ctrl_triggerValue)
                    if "left" in control_arm_sides else 0.0
                )
                desired[RIGHT_GRIPPER_INDEX] = (
                    0.1 * controller_open_fraction(tele.right_ctrl_triggerValue)
                    if "right" in control_arm_sides else 0.0
                )
                command, command_velocity, command_acceleration = jerk_limit_command(
                    desired,
                    command,
                    command_velocity,
                    command_acceleration,
                    dt,
                    args.arm_velocity,
                    args.gripper_velocity,
                    args.arm_acceleration,
                    args.gripper_acceleration,
                    args.arm_jerk,
                    args.gripper_jerk,
                )
                node.publish_arm_command(command, args.gripper_effort)
                if jitter_logger is not None:
                    jitter_logger.write(
                        timestamp=started,
                        enabled=enabled,
                        reset_active=arm_reset_active,
                        ik_success=ik_success,
                        ik_solve_ms=ik_solve_ms,
                        xr_left=left_pose,
                        xr_right=right_pose,
                        target_raw=target_raw,
                        target_filtered=target_filtered,
                        ik_seed=ik_seed,
                        ik_solution=ik_solution,
                        command=command,
                        command_velocity=command_velocity,
                        command_acceleration=command_acceleration,
                        feedback=state,
                    )

                reset_slices = tuple(
                    arm_slice
                    for side, arm_slice in (("left", LEFT_ARM_SLICE), ("right", RIGHT_ARM_SLICE))
                    if side in required_arm_sides
                )
                if arm_reset_active and max(
                    float(np.max(np.abs(command[arm_slice]))) for arm_slice in reset_slices
                ) <= 1e-3:
                    arm_reset_active = False
                    arm_reset_start_time = None
                    arm_reset_start_positions = None
                    enabled = False
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    mapper.clear()
                    node.set_control_gate(False)
                    node.emergency_hold()
                    node.get_logger().info(
                        "Arm reset complete; XR control PAUSED at zero pose. "
                        "Press Right B to resume"
                    )
                    events.info(
                        "ARM_RESET_DONE",
                        "online arms reached zero; control paused",
                    )

                if recording or recorder_future is not None:
                    node.publish_chassis(np.zeros(3))
                    if last_lift_speed != 0 or started - last_lift_send >= 1.0:
                        node.stop_lift()
                        last_lift_speed = 0
                        last_lift_send = started
                else:
                    velocity = chassis_command(
                        tele.left_ctrl_thumbstickValue,
                        tele.right_ctrl_thumbstickValue,
                        chassis_limits,
                    )
                    node.publish_chassis(velocity)
                    lift_axis = -apply_deadband(float(tele.right_ctrl_thumbstickValue[1]), args.stick_deadband)
                    lift_speed = int(round(lift_axis * args.lift_speed))
                    if (
                        last_lift_speed != lift_speed
                        or started - last_lift_send >= args.lift_command_period
                    ):
                        if lift_speed == 0:
                            node.stop_lift()
                        else:
                            node.command_lift_speed(lift_speed)
                        last_lift_speed = lift_speed
                        last_lift_send = started
            elif enabled:
                # Pure chassis/lift stage: no arm feedback, IK, or arm
                # command publication is involved.
                velocity = chassis_command(
                    tele.left_ctrl_thumbstickValue,
                    tele.right_ctrl_thumbstickValue,
                    chassis_limits,
                )
                node.publish_chassis(velocity)
                lift_axis = -apply_deadband(
                    float(tele.right_ctrl_thumbstickValue[1]), args.stick_deadband
                )
                lift_speed = int(round(lift_axis * args.lift_speed))
                if (
                    last_lift_speed != lift_speed
                    or started - last_lift_send >= args.lift_command_period
                ):
                    if lift_speed == 0:
                        node.stop_lift()
                    else:
                        node.command_lift_speed(lift_speed)
                    last_lift_speed = lift_speed
                    last_lift_send = started
            else:
                node.publish_chassis(np.zeros(3))

            if recording and not enabled and recorder_future is None:
                if begin_recorder_request(False):
                    node.get_logger().warning(
                        "XR control is no longer enabled; stopping AGX recording"
                    )
                    events.warning(
                        "REC_STOP",
                        "AGX recording stop requested because control is disabled",
                    )

            if (
                image_stream_enabled
                and args.display_mode != "pass-through"
                and started >= next_display
            ):
                next_display += display_period * (
                    math.floor((started - next_display) / display_period) + 1
                )
                head_image = node.camera_sample("head")
                if head_image is not None:
                    xr.render(head_image.image)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        events.info(
            "SHUTDOWN",
            "XR teleoperation stopping",
            duration_s=events.elapsed,
            control_enabled=locals().get("enabled", False),
            recording=locals().get("recording", False),
        )
        node.safe_shutdown()
        if jitter_logger is not None:
            jitter_logger.close()

        shutdown_recording = bool(locals().get("recording", False))
        pending_future = locals().get("recorder_future")
        pending_target = locals().get("recorder_target")
        if pending_future is not None:
            deadline = time.monotonic() + args.recorder_service_timeout
            while not pending_future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            try:
                if pending_future.done():
                    pending_response = pending_future.result()
                    if pending_response is not None and pending_response.success:
                        shutdown_recording = bool(pending_target)
            except Exception as error:
                events.error("REC_SERVICE_FAILED", str(error))

        if shutdown_recording and node.recorder_service_ready():
            try:
                stop_future = node.request_recording(False)
                deadline = time.monotonic() + args.recorder_service_timeout
                while not stop_future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                if not stop_future.done():
                    raise TimeoutError("AGX recorder stop timed out during shutdown")
                response = stop_future.result()
                if response is None or not response.success:
                    raise RuntimeError(
                        response.message if response is not None else "empty response"
                    )
                payload = json.loads(response.message)
                path = payload.get("path", "unknown")
                node.get_logger().info(
                    f"AGX recording saved during shutdown: {path}"
                )
                events.info(
                    "REC_SAVED",
                    "AGX recording saved during shutdown",
                    path=path,
                )
            except Exception as error:
                node.get_logger().error(
                    f"Failed to save AGX recording during shutdown: {error}"
                )
                events.error("REC_SAVE_FAILED", str(error))
        keyboard.close()
        if xr is not None:
            xr.close()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)
        events.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
