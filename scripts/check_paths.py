#!/usr/bin/env python3
"""Read-only checks for server and AGX real-robot deployments."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ammr_teleop.common.paths import configured_path, project_root


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("server", "agx"), required=True)
    args = parser.parse_args()
    checks: list[tuple[str, str, bool]] = []

    def check(label: str, path: str | Path) -> None:
        candidate = Path(path)
        checks.append((label, str(candidate), candidate.exists()))

    check("project", project_root() / "src/ammr_teleop")
    if args.mode == "server":
        check("XR Python", configured_path("teleop_python"))
        check("IK source", project_root() / "src/ammr_teleop/common/robot_ik.py")
        check("IK URDF", configured_path("kinematics_urdf"))
    else:
        check("AGX recorder", project_root() / "src/ammr_teleop/real/agx_recorder_node.py")
        check("lift ROS interface", project_root() / "ros2_ws/src/lifting_msg_pkg/package.xml")
    for label, path, exists in checks:
        print(f'{"OK" if exists else "MISSING"} {label}: {path}')
    print(json.dumps({key: configured_path(key) for key in ("data_root", "log_root", "ros_ws")}, indent=2))
    print(f"Checked {len(checks)} paths; missing={sum(not item[2] for item in checks)}")
    return int(any(not item[2] for item in checks))


if __name__ == "__main__":
    raise SystemExit(main())
