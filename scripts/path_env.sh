#!/usr/bin/env bash
# Sourced by launchers; all local relative paths resolve from this project.
AMMR_PATH_PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AMMR_PATH_ARGS=()
while (( $# )); do
    case "$1" in
        --config|--log-root|--ros-ws|--teleop-python)
            if (( $# < 2 )) || [[ -z "$2" ]]; then
                printf 'Missing value for %s\n' "$1" >&2
                return 2
            fi
            case "$1" in
                --config) export AMMR_CONFIG="$2" ;;
                --log-root) export AMMR_LOG_ROOT="$2" ;;
                --ros-ws) export AMMR_ROS_WS="$2" ;;
                --teleop-python) export AMMR_TELEOP_PYTHON="$2" ;;
            esac
            shift 2 ;;
        *) AMMR_PATH_ARGS+=("$1"); shift ;;
    esac
done
set -- "${AMMR_PATH_ARGS[@]}"
unset AMMR_PATH_ARGS
AMMR_PATH_EXPORTS="$(python3 "${AMMR_PATH_PROJECT}/src/ammr_teleop/common/paths.py" --shell)" || return 1
eval "$AMMR_PATH_EXPORTS"
unset AMMR_PATH_EXPORTS AMMR_PATH_PROJECT
export PYTHONPATH="${AMMR_PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${AMMR_LOG_ROOT}}"
printf '[AMMR paths] project=%s data=%s logs=%s ros_ws=%s\n' \
    "$AMMR_PROJECT_ROOT" "$AMMR_DATA_ROOT" "$AMMR_LOG_ROOT" "$AMMR_ROS_WS" >&2
