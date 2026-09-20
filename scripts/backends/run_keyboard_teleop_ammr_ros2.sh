#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
source "${PROJECT_ROOT}/scripts/path_env.sh"
PYTHON_BIN="$AMMR_TELEOP_PYTHON"
export ROS_DISTRO=humble
export ROS_LOG_DIR="${ROS_LOG_DIR:-${AMMR_LOG_ROOT}/sim_keyboard}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
PYTHON_VERSION="$("$PYTHON_BIN" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
for candidate in "/opt/ros/humble/lib/python${PYTHON_VERSION}/site-packages" "/opt/ros/humble/local/lib/python${PYTHON_VERSION}/dist-packages"; do
    [[ -d "$candidate" ]] && export PYTHONPATH="$candidate${PYTHONPATH:+:$PYTHONPATH}"
done
mkdir -p "$ROS_LOG_DIR"
exec "$PYTHON_BIN" -m ammr_teleop.sim.keyboard "$@"
