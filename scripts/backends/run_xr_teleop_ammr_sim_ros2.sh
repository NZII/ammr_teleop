#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
source "${PROJECT_ROOT}/scripts/path_env.sh"
if [[ " $* " != *" --help "* ]]; then
    python3 "${PROJECT_ROOT}/scripts/check_paths.py" --mode server
fi
PYTHON_BIN="${AMMR_TELEOP_PYTHON}"
if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "XR teleoperation Python is unavailable: $PYTHON_BIN" >&2
    exit 1
fi
export ROS_DISTRO=humble
export ROS_LOG_DIR="${ROS_LOG_DIR:-${AMMR_LOG_ROOT}/sim}"
export AMMR_EVENT_LOG_DIR="${AMMR_EVENT_LOG_DIR:-$ROS_LOG_DIR}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export AMMR_TARGET_MODE=sim
export LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
PYTHON_VERSION="$("$PYTHON_BIN" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
ROS_PYTHON_PATH=""
for candidate in "/opt/ros/humble/lib/python${PYTHON_VERSION}/site-packages" "/opt/ros/humble/local/lib/python${PYTHON_VERSION}/dist-packages"; do
    [[ -d "$candidate" ]] && ROS_PYTHON_PATH="${ROS_PYTHON_PATH:+${ROS_PYTHON_PATH}:}$candidate"
done
export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}/third_party/televuer/src${ROS_PYTHON_PATH:+:${ROS_PYTHON_PATH}}${PYTHONPATH:+:${PYTHONPATH}}"
mkdir -p "$ROS_LOG_DIR"
if [[ -f "${AMMR_ROS_WS}/install/local_setup.bash" ]]; then
    set +u
    source "${AMMR_ROS_WS}/install/local_setup.bash"
    set -u
fi
exec "$PYTHON_BIN" -m ammr_teleop.real.xr_teleop_real \
    --enable-hardware-output \
    --hardware-components left_arm right_arm chassis lift \
    --image-stream \
    --image-transport zmq \
    --head-zmq-endpoint tcp://127.0.0.1:56555 \
    --left-wrist-zmq-endpoint tcp://127.0.0.1:56556 \
    --right-wrist-zmq-endpoint tcp://127.0.0.1:56557 \
    "$@"
