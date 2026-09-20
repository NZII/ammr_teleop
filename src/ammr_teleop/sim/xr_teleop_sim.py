#!/usr/bin/env python3
"""Control the AMMR Isaac simulation from XR through the ROS 2 bridge."""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, JointState
from sshkeyboard import listen_keyboard, stop_listening
from std_msgs.msg import Bool, Int32, String

from ammr_teleop.common.control_profiles import available_profiles, load_profile

from ammr_teleop.common.event_logging import AmmrEventLogger, configure_event_logger
from ammr_teleop.common.contract import (
    ARM_SLICE,
    LEFT_GRIPPER_SLICE,
    LEFT_ARM_SLICE,
    LIFTER_SLICE,
    RIGHT_GRIPPER_SLICE,
    RIGHT_ARM_SLICE,
    ROBOT_JOINTS,
    clamp_sim_positions,
    gripper_pair,
    jerk_limit_positions,
    ordered_positions,
)
from ammr_teleop.common.kinematics import DualArmKinematics, IKRegularizationConfig
from ammr_teleop.common.pose_mapper import (
    FixedAbsoluteWristMapper,
    PoseMapperConfig,
    RelativeWristMapper,
    calibrated_translation_scale,
    fixed_xr_neutral_poses,
)
from ammr_teleop.xr.frontend import (
    XRInputFrontend,
    controller_open_fraction,
    hand_open_fraction,
)


def reliable_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def image_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class AmmrXrRosNode(Node):
    def __init__(
        self,
        subscribe_head_image: bool = False,
        event_logger: AmmrEventLogger | None = None,
    ):
        super().__init__("ammr_xr_teleop_sim")
        self.event_logger = event_logger
        self.command_publisher = self.create_publisher(
            JointState, "/ammr/joint_command", reliable_qos()
        )
        self.record_publisher = self.create_publisher(
            Bool, "/ammr/record_enable", reliable_qos()
        )
        self.create_subscription(
            JointState, "/ammr/joint_states", self._on_joint_state, reliable_qos()
        )
        self.create_subscription(
            Bool, "/ammr/record_status", self._on_record_status, reliable_qos()
        )
        self.create_subscription(
            Int32, "/ammr/record_episode", self._on_record_episode, reliable_qos()
        )
        self.create_subscription(
            Int32,
            "/ammr/record_saved_episode",
            self._on_record_saved_episode,
            reliable_qos(),
        )
        self.create_subscription(
            String,
            "/ammr/record_saved_path",
            self._on_record_saved_path,
            reliable_qos(),
        )
        self.image_subscription = None
        if subscribe_head_image:
            self.image_subscription = self.create_subscription(
                Image, "/ammr/camera/head/image_raw", self._on_image, image_qos()
            )
        self.lock = threading.RLock()
        self.initialized = threading.Event()
        self.latest_state: np.ndarray | None = None
        self.latest_image: np.ndarray | None = None
        self.image_sequence = 0
        self.recording = False
        self.record_episode = -1
        self.saved_episode = -1
        self.saved_path = ""

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            state = ordered_positions(msg.name, msg.position)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid AMMR state: {error}")
            return
        with self.lock:
            self.latest_state = state
            self.initialized.set()

    def _on_record_status(self, msg: Bool) -> None:
        changed = False
        with self.lock:
            value = bool(msg.data)
            changed = value != self.recording
            self.recording = value
        if changed:
            if self.event_logger is not None:
                self.event_logger.info(
                    "REC_START" if value else "REC_STOP",
                    "recording active" if value else "recording stopped; saving",
                    episode=(
                        f"{self.record_episode:06d}"
                        if self.record_episode >= 0
                        else "pending"
                    ),
                )

    def _on_record_episode(self, msg: Int32) -> None:
        with self.lock:
            self.record_episode = int(msg.data)

    def _on_record_saved_episode(self, msg: Int32) -> None:
        episode = int(msg.data)
        with self.lock:
            if episode >= 0:
                self.saved_episode = episode

    def _on_record_saved_path(self, msg: String) -> None:
        path = str(msg.data).strip()
        if not path:
            return
        with self.lock:
            if path == self.saved_path:
                return
            self.saved_path = path
            episode = self.saved_episode
        if self.event_logger is not None:
            self.event_logger.info(
                "REC_SAVED",
                "episode data and metadata saved",
                episode=f"{episode:06d}" if episode >= 0 else "unknown",
            )
            self.event_logger.info("DATA_PATH", path)

    def _on_image(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning(
                f"Ignoring unsupported image encoding {msg.encoding!r}"
            )
            return
        expected_row = int(msg.width) * 3
        if msg.height <= 0 or msg.width <= 0 or msg.step < expected_row:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.step) * int(msg.height)
        if raw.size < expected_size:
            return
        rows = raw[:expected_size].reshape(int(msg.height), int(msg.step))
        image = rows[:, :expected_row].reshape(int(msg.height), int(msg.width), 3)
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        with self.lock:
            self.latest_image = np.ascontiguousarray(image)
            self.image_sequence += 1

    def state(self) -> np.ndarray | None:
        with self.lock:
            return None if self.latest_state is None else self.latest_state.copy()

    def image(self, after_sequence: int) -> tuple[np.ndarray | None, int]:
        with self.lock:
            if self.image_sequence == after_sequence or self.latest_image is None:
                return None, self.image_sequence
            return self.latest_image.copy(), self.image_sequence

    def publish_command(self, positions: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ROBOT_JOINTS)
        msg.position = positions.tolist()
        self.command_publisher.publish(msg)

    def toggle_recording(self) -> None:
        with self.lock:
            desired = not self.recording
        self.record_publisher.publish(Bool(data=desired))

    def stop_recording(self) -> None:
        with self.lock:
            active = self.recording
        if active:
            self.record_publisher.publish(Bool(data=False))


class Edge:
    def __init__(self):
        self.previous = False

    def rising(self, value: bool) -> bool:
        current = bool(value)
        result = current and not self.previous
        self.previous = current
        return result


class KeyboardCommands:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        self.running = threading.Event()
        self.running.set()
        self.toggle_control = threading.Event()
        self.toggle_record = threading.Event()
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if not self.enabled:
            return

        def on_press(key: str) -> None:
            if key == "r":
                self.toggle_control.set()
            elif key == "s":
                self.toggle_record.set()
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
        help="Shared simulation/real-robot control profile; CLI values override it.",
    )
    parser.add_argument(
        "--xr-frequency",
        "--frequency",
        dest="xr_frequency",
        type=float,
        default=50.0,
        help="XR sampling and IK rate; --frequency is a compatibility alias.",
    )
    parser.add_argument(
        "--command-frequency",
        type=float,
        default=100.0,
        help="Trajectory limiting and ROS command publication rate in Hz.",
    )
    parser.add_argument("--display-frequency", type=float, default=30.0)
    parser.add_argument(
        "--recording-frequency",
        type=float,
        default=30.0,
        help="Expected simulator recorder rate (configured by the Isaac launch).",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help=(
            "Enable episode recording controls. Left X/[s] starts an episode; "
            "press it again to save. Isaac prepares the three cameras at startup."
        ),
    )
    parser.add_argument("--input-mode", choices=("controller", "hand"), default="controller")
    parser.add_argument(
        "--wrist-mapping",
        choices=("incremental", "absolute"),
        default="incremental",
        help=(
            "Wrist target mapping: incremental calibrates at enable/B; "
            "absolute uses fixed TeleVuer neutral and AMMR-zero references."
        ),
    )
    parser.add_argument(
        "--display-mode",
        choices=("pass-through", "immersive", "ego"),
        default="pass-through",
    )
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--state-timeout", type=float, default=30.0)
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
        help="Legacy direct translation gain; overrides arm-length calibration.",
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
    parser.add_argument("--max-wrist-translation", type=float, default=0.45)
    parser.add_argument("--linear-speed", type=float, default=0.30)
    parser.add_argument("--yaw-speed", type=float, default=0.60)
    parser.add_argument(
        "--lifter-speed",
        type=float,
        default=0.10,
        help="Lifter joint speed commanded by the right thumbstick Y axis.",
    )
    parser.add_argument(
        "--arm-reset-duration",
        type=float,
        default=2.5,
        help="Seconds used to smoothly return both arms to the zero pose.",
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.35,
        help="Legacy chassis/lifter URDF velocity scale.",
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
        "--gripper-velocity",
        type=float,
        default=0.10,
        help="Maximum total gripper-width velocity in joint-units/s.",
    )
    parser.add_argument(
        "--gripper-acceleration",
        type=float,
        default=0.4,
        help="Maximum total gripper-width acceleration in joint-units/s^2.",
    )
    parser.add_argument(
        "--gripper-jerk",
        type=float,
        default=2.0,
        help="Maximum total gripper-width jerk in joint-units/s^3.",
    )
    parser.add_argument(
        "--pose-filter-alpha",
        type=float,
        default=0.35,
        help="Per-arm wrist target low-pass factor in (0, 1].",
    )
    parser.add_argument(
        "--pose-filter",
        choices=("ema", "one_euro"),
        default="one_euro",
    )
    parser.add_argument("--one-euro-min-cutoff", type=float, default=0.8)
    parser.add_argument("--one-euro-beta", type=float, default=5.0)
    parser.add_argument("--one-euro-d-cutoff", type=float, default=0.9)
    parser.add_argument(
        "--wrist-translation-deadband",
        type=float,
        default=0.01,
        help="Ignore wrist translation changes smaller than this many meters.",
    )
    parser.add_argument(
        "--wrist-rotation-deadband-deg",
        type=float,
        default=1.5,
        help="Ignore wrist rotation changes smaller than this many degrees.",
    )
    parser.add_argument(
        "--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0"))
    )
    parser.add_argument(
        "--keyboard",
        action=argparse.BooleanOptionalAction,
        default=sys.stdin.isatty(),
        help="Enable r/s/q fallback (defaults to interactive terminals only).",
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
    return args


def validate_args(args: argparse.Namespace) -> None:
    positive = (
        "xr_frequency",
        "command_frequency",
        "display_frequency",
        "recording_frequency",
        "image_width",
        "image_height",
        "state_timeout",
        "operator_arm_length",
        "robot_teleop_reach",
        "rotation_scale",
        "max_wrist_translation",
        "linear_speed",
        "yaw_speed",
        "lifter_speed",
        "arm_reset_duration",
        "velocity_scale",
        "arm_velocity",
        "arm_acceleration",
        "gripper_acceleration",
        "arm_jerk",
        "gripper_jerk",
        "gripper_velocity",
        "one_euro_min_cutoff",
        "one_euro_d_cutoff",
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
    if not math.isfinite(args.pose_filter_alpha) or not 0.0 < args.pose_filter_alpha <= 1.0:
        raise ValueError("pose_filter_alpha must be finite and in (0, 1]")
    if not math.isfinite(args.one_euro_beta) or args.one_euro_beta < 0.0:
        raise ValueError("one_euro_beta must be finite and non-negative")
    if not math.isfinite(args.wrist_translation_deadband) or args.wrist_translation_deadband < 0.0:
        raise ValueError("wrist_translation_deadband must be finite and non-negative")
    if not math.isfinite(args.wrist_rotation_deadband_deg) or args.wrist_rotation_deadband_deg < 0.0:
        raise ValueError("wrist_rotation_deadband_deg must be finite and non-negative")

def main() -> int:
    args = parse_args()
    validate_args(args)
    translation_scale = calibrated_translation_scale(
        args.operator_arm_length,
        args.robot_teleop_reach,
        args.translation_scale,
    )
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    events = configure_event_logger("sim")
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
        input=args.input_mode,
        translation_gain=translation_scale,
        rotation_gain=args.rotation_scale,
        xr_hz=args.xr_frequency,
        command_hz=args.command_frequency,
        recording_hz=args.recording_frequency,
    )

    rclpy.init()
    node = AmmrXrRosNode(
        subscribe_head_image=args.display_mode != "pass-through",
        event_logger=events,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_executor() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()
    keyboard = KeyboardCommands(args.keyboard)
    xr: XRInputFrontend | None = None

    def stop(_signum=None, _frame=None) -> None:
        keyboard.running.clear()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        events.info("ROS_WAIT", "waiting for simulator joint state", topic="/ammr/joint_states")
        if not node.initialized.wait(args.state_timeout):
            events.error("ROS_TIMEOUT", "simulator joint state did not arrive")
            return 2
        events.info("ROS_READY", "simulator joint state received", dimensions=21)

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
        command = node.state()
        assert command is not None
        command_velocity = np.zeros_like(command)
        command_acceleration = np.zeros_like(command)
        mapper_config = PoseMapperConfig(
            translation_scale=translation_scale,
            rotation_scale=args.rotation_scale,
            max_translation=args.max_wrist_translation,
            filter_alpha=args.pose_filter_alpha,
            filter_mode=args.pose_filter,
            one_euro_min_cutoff=args.one_euro_min_cutoff,
            one_euro_beta=args.one_euro_beta,
            one_euro_d_cutoff=args.one_euro_d_cutoff,
            translation_deadband=args.wrist_translation_deadband,
            rotation_deadband=math.radians(args.wrist_rotation_deadband_deg),
        )
        if args.wrist_mapping == "incremental":
            mapper = RelativeWristMapper(mapper_config)
        else:
            xr_neutral = fixed_xr_neutral_poses(args.input_mode)
            ammr_zero = np.zeros_like(command)
            robot_neutral = kinematics.forward(ammr_zero)
            mapper = FixedAbsoluteWristMapper(
                xr_reference=xr_neutral,
                robot_reference=robot_neutral,
                config=mapper_config,
            )
        xr = XRInputFrontend(
            input_mode=args.input_mode,
            display_mode=args.display_mode,
            image_shape=(args.image_height, args.image_width),
            display_fps=args.display_frequency,
        )
        keyboard.start()
        events.info(
            "READY",
            (
                "Right-B=control Left-X=record Left-Y=reset Right-A=exit"
                if args.record
                else "Right-B=control Left-Y=reset Right-A=exit (recording disabled)"
            ),
            display=args.display_mode,
            recorder="enabled" if args.record else "disabled",
        )
        enabled = False
        enable_edge = Edge()
        record_edge = Edge()
        quit_edge = Edge()
        arm_reset_edge = Edge()
        last_image_sequence = 0
        last_wrist_targets: tuple[np.ndarray, np.ndarray] | None = None
        latched_arm_command: np.ndarray | None = None
        arm_reset_active = False
        arm_reset_start_time: float | None = None
        arm_reset_start_positions: np.ndarray | None = None
        ik_failures = 0
        period = 1.0 / args.command_frequency
        ik_period = 1.0 / args.xr_frequency
        display_period = 1.0 / args.display_frequency
        previous_time = time.monotonic()
        last_xr_sample = -math.inf
        last_ik_update = -math.inf
        next_display = previous_time
        tele = xr.sample()
        events.info("XR_CONNECTED", "first XR sample received", input=args.input_mode)

        while keyboard.running.is_set() and rclpy.ok():
            started = time.monotonic()
            dt = min(max(started - previous_time, 1e-4), 0.1)
            previous_time = started
            state = node.state()
            if state is None:
                time.sleep(period)
                continue

            # Use the last command as the IK seed. The ROS joint-state
            # callback is asynchronous and can still contain the pose from
            # before a pause/reset; using it as the seed would reintroduce
            # that old pose during the first solve after resuming.
            ik_state = command.copy()

            # TeleVuer keeps the last valid shared state when an XR event is
            # malformed or temporarily absent, matching Unitree's original
            # continuous-control behavior.
            xr_sample_due = started - last_xr_sample >= ik_period
            if xr_sample_due:
                tele = xr.sample()
                last_xr_sample = started
            toggle_requested = keyboard.toggle_control.is_set()
            keyboard.toggle_control.clear()
            record_requested = keyboard.toggle_record.is_set()
            keyboard.toggle_record.clear()

            if args.input_mode == "controller":
                toggle_requested |= enable_edge.rising(tele.right_ctrl_bButton)
                record_requested |= record_edge.rising(tele.left_ctrl_aButton)
                if quit_edge.rising(tele.right_ctrl_aButton):
                    events.info("QUIT_REQUEST", "Right A requested shutdown")
                    keyboard.running.clear()
                if arm_reset_edge.rising(tele.left_ctrl_bButton):
                    if enabled:
                        reset_command = state.copy()
                        reset_command[ARM_SLICE] = 0.0
                        kinematics.reset_solver_history(reset_command)
                        if args.wrist_mapping == "incremental":
                            reset_left, reset_right = kinematics.forward(reset_command)
                            mapper.calibrate(
                                tele.left_wrist_pose,
                                tele.right_wrist_pose,
                                reset_left,
                                reset_right,
                            )
                        latched_arm_command = reset_command
                        last_wrist_targets = None
                        arm_reset_active = True
                        arm_reset_start_time = started
                        arm_reset_start_positions = command[ARM_SLICE].copy()
                        ik_failures = 0
                        events.info(
                            "ARM_RESET_START",
                            "Left Y requested arm reset",
                            duration_s=args.arm_reset_duration,
                        )
                    else:
                        events.warning(
                            "ARM_RESET_REJECT", "XR control is paused"
                        )
                if (
                    enabled
                    and tele.left_ctrl_thumbstick
                    and tele.right_ctrl_thumbstick
                ):
                    enabled = False
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    mapper.clear()
                    last_wrist_targets = None
                    latched_arm_command = None
                    arm_reset_active = False
                    arm_reset_start_time = None
                    arm_reset_start_positions = None
                    events.warning("EMERGENCY_HOLD", "both stick buttons pressed")

                if toggle_requested:
                    if enabled:
                        enabled = False
                        command_velocity.fill(0.0)
                        command_acceleration.fill(0.0)
                        mapper.clear()
                        last_wrist_targets = None
                        latched_arm_command = None
                        arm_reset_active = False
                        arm_reset_start_time = None
                        arm_reset_start_positions = None
                        events.info("CONTROL_PAUSE", "XR control paused")
                    else:
                        # Resume from the last published target, not from
                        # the asynchronously received joint state. This is
                        # important after Left Y, when command[ARM_SLICE] is
                        # the zero pose while state may still be stale.
                        resume_command = command.copy()
                        kinematics.reset_solver_history(resume_command)
                        command = resume_command
                        command_velocity.fill(0.0)
                        command_acceleration.fill(0.0)
                        latched_arm_command = resume_command.copy()
                        ik_state = resume_command
                        if args.wrist_mapping == "incremental":
                            robot_left, robot_right = kinematics.forward(resume_command)
                            mapper.calibrate(
                                tele.left_wrist_pose,
                                tele.right_wrist_pose,
                                robot_left,
                                robot_right,
                            )
                            initial_left_target, initial_right_target = mapper.map(
                                tele.left_wrist_pose, tele.right_wrist_pose
                            )
                            # Hold the calibrated target for the first
                            # enabled cycle. This prevents even a tiny XR
                            # sample change on B from invoking IK immediately.
                            last_wrist_targets = (
                                initial_left_target.copy(),
                                initial_right_target.copy(),
                            )
                        else:
                            # Absolute mode has fixed references and must
                            # solve the current absolute XR target on resume.
                            last_wrist_targets = None
                        arm_reset_active = False
                        arm_reset_start_time = None
                        arm_reset_start_positions = None
                        enabled = True
                        events.info(
                            "CONTROL_ENABLE",
                            "XR control enabled",
                            mapping=args.wrist_mapping,
                        )

            if record_requested:
                if args.record:
                    events.info("REC_TOGGLE", "Left X/[s] requested recorder toggle")
                    node.toggle_recording()
                else:
                    events.warning(
                        "REC_DISABLED",
                        "recording is disabled; restart teleop with --record",
                    )

            if enabled:
                desired = command.copy()
                assert latched_arm_command is not None
                if arm_reset_active:
                    # Keep solving out of the reset transition.  Otherwise
                    # the current XR pose would immediately overwrite the
                    # requested zero arm target on the next cycle.
                    assert arm_reset_start_time is not None
                    assert arm_reset_start_positions is not None
                    reset_progress = min(
                        max(
                            (started - arm_reset_start_time)
                            / args.arm_reset_duration,
                            0.0,
                        ),
                        1.0,
                    )
                    # Smoothstep gives zero velocity at both ends; the normal
                    # rate limiter remains as a second safety boundary.
                    smooth_progress = reset_progress * reset_progress * (
                        3.0 - 2.0 * reset_progress
                    )
                    desired[ARM_SLICE] = arm_reset_start_positions * (
                        1.0 - smooth_progress
                    )
                elif xr_sample_due or started - last_ik_update >= ik_period:
                    last_ik_update = started
                    left_target, right_target = mapper.map(
                        tele.left_wrist_pose, tele.right_wrist_pose
                    )
                    left_target_held = (
                        last_wrist_targets is not None
                        and np.array_equal(left_target, last_wrist_targets[0])
                    )
                    right_target_held = (
                        last_wrist_targets is not None
                        and np.array_equal(right_target, last_wrist_targets[1])
                    )
                    last_wrist_targets = (left_target.copy(), right_target.copy())
                    if not (left_target_held and right_target_held):
                        solved = kinematics.solve(left_target, right_target, ik_state)
                        if solved is not None:
                            # A held wrist keeps its previous IK target even when
                            # the other arm moves and the dual-arm optimizer finds
                            # a slightly different equivalent solution.
                            if not left_target_held:
                                latched_arm_command[LEFT_ARM_SLICE] = solved[
                                    LEFT_ARM_SLICE
                                ]
                            if not right_target_held:
                                latched_arm_command[RIGHT_ARM_SLICE] = solved[
                                    RIGHT_ARM_SLICE
                                ]
                            ik_failures = 0
                        else:
                            ik_failures += 1
                            if ik_failures == 1 or ik_failures % 30 == 0:
                                reason = getattr(kinematics, "last_failure_reason", None)
                                suffix = f": {reason}" if reason else ""
                                node.get_logger().warning(
                                    "Dual-arm IK rejected target "
                                    f"({ik_failures} consecutive){suffix}"
                                )

                if not arm_reset_active:
                    desired[LEFT_ARM_SLICE] = latched_arm_command[LEFT_ARM_SLICE]
                    desired[RIGHT_ARM_SLICE] = latched_arm_command[RIGHT_ARM_SLICE]

                if args.input_mode == "controller":
                    left_open = controller_open_fraction(tele.left_ctrl_triggerValue)
                    right_open = controller_open_fraction(tele.right_ctrl_triggerValue)
                    left_stick = np.asarray(tele.left_ctrl_thumbstickValue, dtype=float)
                    right_stick = np.asarray(tele.right_ctrl_thumbstickValue, dtype=float)
                    local_x = -left_stick[1] * args.linear_speed * dt
                    local_y = -left_stick[0] * args.linear_speed * dt
                    yaw = float(state[2])
                    desired[0] += local_x * math.cos(yaw) - local_y * math.sin(yaw)
                    desired[1] += local_x * math.sin(yaw) + local_y * math.cos(yaw)
                    desired[2] += -right_stick[0] * args.yaw_speed * dt
                    lifter_input = -float(right_stick[1])
                    if abs(lifter_input) < 0.05:
                        lifter_input = 0.0
                    desired[LIFTER_SLICE] += lifter_input * args.lifter_speed * dt
                else:
                    left_open = hand_open_fraction(tele.left_hand_pinchValue)
                    right_open = hand_open_fraction(tele.right_hand_pinchValue)

                desired[LEFT_GRIPPER_SLICE] = gripper_pair(left_open)
                desired[RIGHT_GRIPPER_SLICE] = gripper_pair(right_open)
                desired = clamp_sim_positions(desired)
                command, command_velocity, command_acceleration = jerk_limit_positions(
                    desired,
                    command,
                    command_velocity,
                    command_acceleration,
                    dt,
                    velocity_scale=args.velocity_scale,
                    arm_acceleration=args.arm_acceleration,
                    # The simulator represents one physical gripper width by
                    # two opposing finger joints.  Profile limits use the
                    # real-driver width convention, so each finger receives
                    # half of the configured width limit.
                    gripper_acceleration=0.5 * args.gripper_acceleration,
                    arm_jerk=args.arm_jerk,
                    gripper_jerk=0.5 * args.gripper_jerk,
                    enforce_position_limits=False,
                    arm_velocity=args.arm_velocity,
                    gripper_velocity=0.5 * args.gripper_velocity,
                )
                if arm_reset_active and np.max(np.abs(command[ARM_SLICE])) <= 1e-3:
                    arm_reset_active = False
                    arm_reset_start_time = None
                    arm_reset_start_positions = None
                    enabled = False
                    mapper.clear()
                    last_wrist_targets = None
                    latched_arm_command = None
                    command[ARM_SLICE] = 0.0
                    command_velocity.fill(0.0)
                    command_acceleration.fill(0.0)
                    events.info(
                        "ARM_RESET_DONE",
                        "arms reached zero pose; control paused",
                    )
            # While paused, keep publishing the target latched at startup,
            # pause, or emergency hold.  Following the
            # latest measured state here would accept small PhysX drift as the
            # next target on every cycle and make an idle arm/base creep.

            node.publish_command(command)
            if started >= next_display:
                next_display += display_period * (
                    math.floor((started - next_display) / display_period) + 1
                )
                image, last_image_sequence = node.image(last_image_sequence)
                if image is not None:
                    xr.render(image)
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period - elapsed))
    finally:
        events.info(
            "SHUTDOWN",
            "XR teleoperation stopping",
            duration_s=events.elapsed,
            control_enabled=locals().get("enabled", False),
            recording=node.recording,
        )
        if args.record:
            node.stop_recording()
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
