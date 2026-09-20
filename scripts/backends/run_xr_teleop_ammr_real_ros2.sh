#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
source "${PROJECT_ROOT}/scripts/path_env.sh"
if [[ " $* " != *" --help "* ]]; then
    python3 "${PROJECT_ROOT}/scripts/check_paths.py" --mode server
fi

PYTHON_BIN="${AMMR_TELEOP_PYTHON}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "XR teleoperation Python is unavailable: ${PYTHON_BIN}" >&2
    echo "Set AMMR_TELEOP_PYTHON or edit configs/paths.yaml." >&2
    exit 1
fi

export ROS_DISTRO=humble
export ROS_LOG_DIR="${ROS_LOG_DIR:-${AMMR_LOG_ROOT}}"
export AMMR_EVENT_LOG_DIR="${AMMR_EVENT_LOG_DIR:-${ROS_LOG_DIR}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"
export AMMR_TARGET_MODE=real
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

export PYTHONPATH="${PROJECT_ROOT}/src:${PROJECT_ROOT}:${PROJECT_ROOT}/third_party/televuer/src${ROS_PYTHON_PATH:+:${ROS_PYTHON_PATH}}${PYTHONPATH:+:${PYTHONPATH}}"
mkdir -p "${ROS_LOG_DIR}"
if [[ -f "${AMMR_ROS_WS}/install/local_setup.bash" ]]; then
    set +u
    source "${AMMR_ROS_WS}/install/local_setup.bash"
    set -u
fi

"${PYTHON_BIN}" -c "import casadi, cv2, numpy, pinocchio, zmq, rclpy, sensor_msgs.msg, sshkeyboard, televuer" || {
    echo "Missing XR/ROS2/IK dependencies for ${PYTHON_BIN}." >&2
    exit 1
}

cd "${PROJECT_ROOT}"
exec "${PYTHON_BIN}" -m ammr_teleop.real.xr_teleop_real "$@"
