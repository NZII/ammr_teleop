#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
source "${PROJECT_ROOT}/scripts/path_env.sh"

if [[ -n "${AMMR_REPLAY_PYTHON:-}" ]]; then
    PYTHON_BIN="${AMMR_REPLAY_PYTHON}"
elif [[ -n "${TELEOP_ENV:-}" && -x "${TELEOP_ENV}/bin/python" ]]; then
    PYTHON_BIN="${AMMR_TELEOP_PYTHON}"
elif [[ -n "${XR_TELEOP_ENV:-}" && -x "${XR_TELEOP_ENV}/bin/python" ]]; then
    PYTHON_BIN="${XR_TELEOP_ENV}/bin/python"
elif [[ -x "${AMMR_TELEOP_PYTHON}" ]]; then
    PYTHON_BIN="${AMMR_TELEOP_PYTHON}"
else
    PYTHON_BIN="$(command -v python3 || true)"
fi

if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
    echo "No usable replay Python found. Set AMMR_REPLAY_PYTHON explicitly." >&2
    exit 1
fi

export ROS_DISTRO=humble
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ammr_real_replay_ros2_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"
export LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

PYTHON_VERSION="$("${PYTHON_BIN}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
ROS_PYTHON_PATH=""
for ROS_PYTHON_DIR in \
    "/opt/ros/humble/lib/python${PYTHON_VERSION}/site-packages" \
    "/opt/ros/humble/local/lib/python${PYTHON_VERSION}/dist-packages"; do
    if [[ -d "${ROS_PYTHON_DIR}" ]]; then
        ROS_PYTHON_PATH="${ROS_PYTHON_PATH:+${ROS_PYTHON_PATH}:}${ROS_PYTHON_DIR}"
    fi
done

export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}${ROS_PYTHON_PATH:+:${ROS_PYTHON_PATH}}${PYTHONPATH:+:${PYTHONPATH}}"
mkdir -p "${ROS_LOG_DIR}"

"${PYTHON_BIN}" -c "import numpy, pandas, pyarrow, rclpy, sensor_msgs.msg, sshkeyboard, std_srvs.srv" || {
    echo "Missing ROS2/replay dependencies for ${PYTHON_BIN}." >&2
    exit 1
}

cd "${PROJECT_ROOT}"
exec "${PYTHON_BIN}" -m ammr_teleop.real.replay_real "$@"
