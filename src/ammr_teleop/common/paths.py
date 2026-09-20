"""Portable project paths for the real-robot teleoperation package.

Resolution order is explicit value > environment > config > default. Local
relative paths are anchored at the installed project root, never the caller's
working directory. ``agx_*`` values stay as remote strings and are resolved
against the AGX user's HOME by ``agx_process_manager.sh``.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex


DEFAULTS = {
    "kinematics_urdf": "models/kinematics/c2436.urdf",
    "data_root": "datasets",
    "log_root": "~/.cache/ammr_teleop/logs",
    "ros_ws": "ros2_ws",
    "teleop_python": "~/miniconda3/envs/tv/bin/python",
    "agx_root": "ammr_teleop",
    "agx_data_dir": "ammr_teleop/datasets/ammr_real_xr",
    "agx_overlay_setup": "",
    "agx_camera_setup": "",
}

ENV = {
    "kinematics_urdf": "AMMR_URDF",
    "data_root": "AMMR_DATA_ROOT",
    "log_root": "AMMR_LOG_ROOT",
    "ros_ws": "AMMR_ROS_WS",
    "teleop_python": "AMMR_TELEOP_PYTHON",
    "agx_root": "AMMR_AGX_ROOT",
    "agx_data_dir": "AMMR_AGX_DATA_DIR",
    "agx_overlay_setup": "AMMR_AGX_OVERLAY_SETUP",
    "agx_camera_setup": "AMMR_AGX_CAMERA_SETUP",
}

OPTIONAL_KEYS = {"agx_overlay_setup", "agx_camera_setup"}


def project_root() -> Path:
    configured = os.environ.get("AMMR_PROJECT_ROOT")
    return Path(configured or Path(__file__).resolve().parents[3]).expanduser().resolve()


def settings() -> dict[str, str]:
    filename = Path(
        os.environ.get("AMMR_CONFIG", project_root() / "configs/paths.yaml")
    ).expanduser()
    if not filename.is_absolute():
        filename = project_root() / filename
    if not filename.is_file():
        if "AMMR_CONFIG" in os.environ:
            raise FileNotFoundError(f"AMMR_CONFIG not found: {filename}")
        return {}
    text = filename.read_text(encoding="utf-8")
    try:
        result = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as error:
            raise ValueError(
                "Use JSON syntax in paths.yaml or install PyYAML for full YAML"
            ) from error
        result = yaml.safe_load(text)
    if not isinstance(result, dict) or set(result) - set(DEFAULTS):
        raise ValueError(f"Invalid or unknown path configuration keys: {filename}")
    for key, value in result.items():
        if not isinstance(value, str) or (not value.strip() and key not in OPTIONAL_KEYS):
            raise ValueError(f"Invalid path configuration value for {key}: {filename}")
    return result


def configured_path(key: str, explicit: str | None = None) -> str:
    if key not in DEFAULTS:
        raise KeyError(f"Unknown path setting: {key}")
    value = explicit if explicit is not None else os.environ.get(ENV[key])
    if value is None and key == "teleop_python":
        legacy = os.environ.get("TELEOP_ENV") or os.environ.get("XR_TELEOP_ENV")
        value = str(Path(legacy) / "bin/python") if legacy else None
    if value is None:
        value = settings().get(key, DEFAULTS[key])
    if key.startswith("agx_"):
        return value
    result = Path(value).expanduser()
    return str((result if result.is_absolute() else project_root() / result).resolve())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shell", action="store_true")
    args = parser.parse_args()
    values = {ENV[key]: configured_path(key) for key in DEFAULTS}
    values["AMMR_PROJECT_ROOT"] = str(project_root())
    values["XR_TELEOP_ROOT"] = os.environ.get("XR_TELEOP_ROOT", str(project_root()))
    if args.shell:
        for key, value in values.items():
            print(f"export {key}={shlex.quote(value)}")
    else:
        print(json.dumps(values, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
