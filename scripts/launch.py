#!/usr/bin/env python3
"""Unified AMMR XR launcher for a separate simulator or the real robot."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ammr_teleop.common.paths import configured_path, project_root


def parse(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument(
        "command",
        choices=(
            "teleop",
            "recorder",
            "replay",
            "inspect-dataset",
            "control-center",
            "check",
        ),
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--sim",
        action="store_true",
        help="Use the separately running ../ammr_sim ROS 2 bridge.",
    )
    mode.add_argument(
        "--real",
        action="store_true",
        help="Use the real robot (default; retained as an explicit safety marker).",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Enable Left-X/[s] episode start/stop for sim or real teleoperation.",
    )
    parser.add_argument("--plan", action="store_true")
    parser.add_argument("--config")
    for option in ("log-root", "ros-ws", "teleop-python"):
        parser.add_argument("--" + option)
    parser.add_argument("--domain-id", type=int, default=None)
    args, extra = parser.parse_known_args(argv)
    if args.domain_id is None:
        args.domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0" if args.sim else "101"))
    if args.command not in ("teleop", "replay", "check") and (args.sim or args.real):
        parser.error(f"{args.command} does not accept --sim/--real")
    if args.record and args.command != "teleop":
        parser.error("--record is valid only with the teleop command")
    if args.sim and "--enable-hardware-output" in extra:
        parser.error("hardware output is not valid with --sim")
    if not 0 <= args.domain_id <= 232:
        parser.error("ROS domain must be in [0, 232]")
    return args, extra


def commands(args, extra):
    root = project_root()
    backends = root / "scripts/backends"
    mode = "sim" if args.sim else "real"
    if args.command == "teleop":
        forwarded = (["--record"] if args.record else []) + list(extra)
        return [[
            "bash",
            str(backends / f"run_xr_teleop_ammr_{mode}_ros2.sh"),
            *forwarded,
        ]]
    if args.command == "replay":
        return [[
            "bash",
            str(backends / f"run_replay_ammr_{mode}_ros2.sh"),
            *extra,
        ]]
    if args.command == "check":
        if args.sim:
            return [["bash", str(root.parent / "ammr_sim/scripts/ammr_sim.sh"), "check", *extra]]
        return [[sys.executable, str(root / "scripts/check_paths.py"), "--mode", "server", *extra]]
    if args.command == "inspect-dataset":
        return [["bash", str(backends / "run_dataset_inspector.sh"), *extra]]
    script = (
        "run_ammr_real_recorder_agx.sh"
        if args.command == "recorder"
        else "run_ammr_control_center.sh"
    )
    return [["bash", str(backends / script), *extra]]


def main(argv=None) -> int:
    args, extra = parse(argv)
    if args.config:
        os.environ["AMMR_CONFIG"] = args.config
    for option, variable in (
        ("log_root", "AMMR_LOG_ROOT"),
        ("ros_ws", "AMMR_ROS_WS"),
        ("teleop_python", "AMMR_TELEOP_PYTHON"),
    ):
        value = getattr(args, option)
        if value:
            os.environ[variable] = value
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain_id)
    env["PYTHONUNBUFFERED"] = "1"
    configured_path("log_root")
    command = commands(args, extra)[0]
    if args.plan:
        print(
            json.dumps(
                {
                    "mode": "sim" if args.sim else "real",
                    "ros_domain_id": args.domain_id,
                    "record": args.record,
                    "command": command,
                },
                indent=2,
            )
        )
        return 0
    os.execvpe(command[0], command, env)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, FileNotFoundError) as error:
        print(f"[AMMR] {error}", file=sys.stderr)
        raise SystemExit(2)
