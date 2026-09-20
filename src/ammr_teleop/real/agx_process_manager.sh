#!/usr/bin/env bash

# This file is sent over SSH and executed on the AGX by the control center
# backend (control_center_web.py; control_center.py remains a legacy desktop
# frontend).
# It deliberately does not use `set -e`: a stale process or an already-UP CAN
# interface must be reported and cleaned up rather than aborting halfway.
# Camera components use the vendor SDK directly and publish JPEG frames on
# three independent ZMQ ports. The recorder subscribes over loopback while the
# server may subscribe over the LAN; remote slowness never blocks acquisition.

ACTION="${1:-}"
COMPONENT="${2:-}"
if [[ -n "${AMMR_AGX_ROOT:-}" ]]; then
    AGX_ROOT="$AMMR_AGX_ROOT"
else
    # Direct invocation from the checked-out script should work without an
    # extra environment variable.  The control center sends this script over
    # stdin and always supplies AMMR_AGX_ROOT, so that path remains explicit.
    SCRIPT_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." 2>/dev/null && pwd)"
    if [[ -x "$SCRIPT_PROJECT_ROOT/scripts/ammr.sh" ]]; then
        AGX_ROOT="$SCRIPT_PROJECT_ROOT"
    else
        AGX_ROOT="ammr_teleop"
    fi
fi
AGX_OVERLAY_SETUP="${AMMR_AGX_OVERLAY_SETUP:-}"
AGX_CAMERA_SETUP="${AMMR_AGX_CAMERA_SETUP:-}"
STATE_DIR="${AMMR_STATE_DIR:-$HOME/.cache/ammr_xr_ui}"
LOG_DIR="$STATE_DIR/logs"
TASK_NAME="${AMMR_TASK:-XR双臂遥操作数据采集}"
DATA_DIR="${AMMR_DATA_DIR:-$AGX_ROOT/datasets/ammr_real_xr}"
DOMAIN_ID="${AMMR_DOMAIN_ID:-101}"
# Resolve remote-relative values here, on the AGX, never on the server.
remote_path() {
    case "$1" in
        '~/'*) printf '%s/%s' "$HOME" "${1:2}" ;;
        /*) printf '%s' "$1" ;;
        *) printf '%s/%s' "$HOME" "$1" ;;
    esac
}
AGX_ROOT="$(remote_path "$AGX_ROOT")"
DATA_DIR="$(remote_path "$DATA_DIR")"
if [[ -n "$AGX_OVERLAY_SETUP" ]]; then
    AGX_OVERLAY_SETUP="$(remote_path "$AGX_OVERLAY_SETUP")"
fi
if [[ -n "$AGX_CAMERA_SETUP" ]]; then
    AGX_CAMERA_SETUP="$(remote_path "$AGX_CAMERA_SETUP")"
fi
# The deployment currently uses this AGX account password for both SSH and
# sudo. It is supplied by the control center in the manager's short-lived
# shell environment; AMMR_SUDO_PASSWORD can override it for a future setup.
AMMR_SUDO_PASSWORD="${AMMR_SUDO_PASSWORD:-1212}"

mkdir -p "$STATE_DIR" "$LOG_DIR"

pid_file() { printf '%s/%s.pid' "$STATE_DIR" "$1"; }
log_file() { printf '%s/%s.log' "$LOG_DIR" "$1"; }
shell_quote() { printf '%q' "$1"; }

sudo_run() {
    if [[ -n "${AMMR_SUDO_PASSWORD:-}" ]]; then
        printf '%s\n' "$AMMR_SUDO_PASSWORD" | sudo -S -p '' "$@"
    else
        sudo -n "$@"
    fi
}

component_command() {
    local overlay_source camera_source camera_binary camera_library_path
    overlay_source=''
    camera_source=''
    if [[ -n "$AGX_OVERLAY_SETUP" ]]; then
        overlay_source="source $(shell_quote "$AGX_OVERLAY_SETUP") 2>/dev/null || true;"
    fi
    if [[ -n "$AGX_CAMERA_SETUP" ]]; then
        camera_source="source $(shell_quote "$AGX_CAMERA_SETUP") 2>/dev/null || true;"
    else
        camera_source="$overlay_source"
    fi
    camera_binary="$AGX_ROOT/install/camera_zmq/bin/ammr_camera_zmq_publisher"
    camera_library_path="/home/lin/agilex_ws/install/orbbec_camera/lib"
    case "$1" in
        left_arm)
            printf '%s' "source /opt/ros/humble/setup.bash; $overlay_source export ROS_DOMAIN_ID=$DOMAIN_ID; ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can4 arm_type:=piper effector_type:=agx_gripper control:=false namespace:=left"
            ;;
        right_arm)
            printf '%s' "source /opt/ros/humble/setup.bash; $overlay_source export ROS_DOMAIN_ID=$DOMAIN_ID; ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can3 arm_type:=piper effector_type:=agx_gripper control:=false namespace:=right"
            ;;
        # Driver locations are external deployment concerns. An optional ROS
        # overlay can be supplied with AMMR_AGX_OVERLAY_SETUP.
        left_camera)
            printf '%s' "export LD_LIBRARY_PATH=$(shell_quote "$camera_library_path"):\${LD_LIBRARY_PATH:-}; test -x $(shell_quote "$camera_binary") || { echo 'Build camera publisher with scripts/build_camera_zmq_agx.sh'; exit 2; }; exec $(shell_quote "$camera_binary") --backend orbbec --camera-name left_wrist --serial CC1N16200TW --usb-port 1-4.1.1.2.4 --bind tcp://0.0.0.0:55556 --width 640 --height 480 --fps 30 --jpeg-quality 92"
            ;;
        right_camera)
            printf '%s' "export LD_LIBRARY_PATH=$(shell_quote "$camera_library_path"):\${LD_LIBRARY_PATH:-}; test -x $(shell_quote "$camera_binary") || { echo 'Build camera publisher with scripts/build_camera_zmq_agx.sh'; exit 2; }; exec $(shell_quote "$camera_binary") --backend orbbec --camera-name right_wrist --serial CC1WC5200R4 --usb-port 1-4.1.3.4 --bind tcp://0.0.0.0:55557 --width 640 --height 480 --fps 30 --jpeg-quality 92"
            ;;
        # RealSense declares serial_no as a string.  The nested quotes preserve
        # the string type when this command is evaluated by `bash -lc`.
        head_camera)
            printf '%s' "export LD_LIBRARY_PATH=/usr/local/lib:$(shell_quote "$camera_library_path"):\${LD_LIBRARY_PATH:-}; test -x $(shell_quote "$camera_binary") || { echo 'Build camera publisher with scripts/build_camera_zmq_agx.sh'; exit 2; }; exec $(shell_quote "$camera_binary") --backend realsense --camera-name head --serial 130222250419 --bind tcp://0.0.0.0:55555 --width 640 --height 480 --fps 30 --jpeg-quality 92"
            ;;
        recorder)
            printf '%s' "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$DOMAIN_ID; cd $(shell_quote "$AGX_ROOT"); exec bash scripts/ammr.sh recorder --data-dir $(shell_quote "$DATA_DIR") --task $(shell_quote "$TASK_NAME")"
            ;;
        chassis)
            printf '%s' "source /opt/ros/humble/setup.bash; $overlay_source export ROS_DOMAIN_ID=$DOMAIN_ID; ros2 launch agx_bringup robot.launch.py"
            ;;
        lift)
            printf '%s' "source /opt/ros/humble/setup.bash; $overlay_source export ROS_DOMAIN_ID=$DOMAIN_ID; ros2 run lifting_ctrl_ros2_pkg service_830abs"
            ;;
        *) return 1 ;;
    esac
}

component_pattern() {
    case "$1" in
        left_arm) printf '%s' 'agx_arm_ctrl_single.*__ns:=/left|start_single_agx_arm_rviz.launch.py.*namespace:=left' ;;
        right_arm) printf '%s' 'agx_arm_ctrl_single.*__ns:=/right|start_single_agx_arm_rviz.launch.py.*namespace:=right' ;;
        left_camera) printf '%s' 'ammr_camera_zmq_publisher.*CC1N16200TW' ;;
        right_camera) printf '%s' 'ammr_camera_zmq_publisher.*CC1WC5200R4' ;;
        head_camera) printf '%s' 'ammr_camera_zmq_publisher.*130222250419' ;;
        recorder) printf '%s' 'ammr_real.agx_recorder_node|ammr_teleop.real.agx_recorder_node|run_ammr_real_recorder_agx' ;;
        chassis) printf '%s' 'agx_bringup.*robot.launch.py' ;;
        lift) printf '%s' 'lifting_ctrl_ros2_pkg.*service_830abs' ;;
        *) return 1 ;;
    esac
}

kill_pid_group() {
    local pid="$1" pgid
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    local own_pgid
    own_pgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
    if [[ "$pgid" =~ ^[0-9]+$ && "$pgid" != "0" && "$pgid" != "$own_pgid" ]]; then
        kill -TERM -- "-$pgid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        sleep 0.4
        kill -KILL -- "-$pgid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    else
        kill -TERM "$pid" 2>/dev/null || true
        sleep 0.4
        kill -KILL "$pid" 2>/dev/null || true
    fi
}

kill_pattern() {
    local pattern="$1" pid self=$$ parent=$PPID
    while read -r pid; do
        [[ "$pid" =~ ^[0-9]+$ ]] || continue
        [[ "$pid" == "$self" || "$pid" == "$parent" ]] && continue
        kill -TERM "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
    sleep 0.4
    while read -r pid; do
        [[ "$pid" =~ ^[0-9]+$ ]] || continue
        [[ "$pid" == "$self" || "$pid" == "$parent" ]] && continue
        kill -KILL "$pid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

stop_component() {
    local name="$1" pid pattern
    pid="$(cat "$(pid_file "$name")" 2>/dev/null || true)"
    kill_pid_group "$pid"
    pattern="$(component_pattern "$name" 2>/dev/null || true)"
    [[ -n "$pattern" ]] && kill_pattern "$pattern"
    rm -f "$(pid_file "$name")"
    printf 'STOPPED %s\n' "$name"
}

recording_service_call() {
    local value="$1"
    local recorder_pid
    if [[ "$value" == "true" ]]; then
        recorder_pid="$(cat "$(pid_file recorder)" 2>/dev/null || true)"
        if ! [[ "$recorder_pid" =~ ^[0-9]+$ ]] \
            || ! kill -0 "$recorder_pid" 2>/dev/null; then
            printf 'RECORDING_ERROR AGX Recorder is not running\n'
            return 2
        fi
    fi
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    export ROS_DOMAIN_ID="$DOMAIN_ID"
    if ! command -v ros2 >/dev/null 2>&1; then
        printf 'RECORDING_ERROR ros2 command is unavailable\n'
        return 2
    fi
    # The recorder does not need sudo; do not leak the credential to it.
    # The recorder exposes a standard SetBool service, so this action does
    # not require the recorder's Python environment on the AGX.  Keep the
    # timeout bounded: a broken ROS graph must not block UI cleanup forever.
    # Recorder finalization includes closing three video writers and writing
    # Parquet (the node default stop timeout is 30 s), so leave enough margin
    # for a busy AGX instead of killing the service client early.
    env -u AMMR_SUDO_PASSWORD timeout 45s ros2 service call /ammr/recording/set_enabled \
        std_srvs/srv/SetBool "{data: $value}"
}

stop_recording_before_cleanup() {
    local pid response rc normalized proc_args
    pid="$(cat "$(pid_file recorder)" 2>/dev/null || true)"
    # Do not spend several seconds probing a recorder that is not running.
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    proc_args="$(ps -o args= -p "$pid" 2>/dev/null || true)"
    [[ "$proc_args" == *"ammr_real_recorder_agx"* || "$proc_args" == *"agx_recorder_node"* ]] || return 0
    response="$(recording_service_call false 2>&1)"
    rc=$?
    normalized="${response,,}"
    if (( rc == 0 )) \
        && [[ "$normalized" != *"success=false"* ]] \
        && [[ "$normalized" != *"success: false"* ]]; then
        printf 'RECORDING_STOP_REQUESTED before cleanup\n'
    else
        printf 'RECORDING_WARN stop request failed; continuing cleanup\n%s\n' "$response"
    fi
}

start_component() {
    local name="$1" cmd pid
    [[ -f "$STATE_DIR/master.active" ]] || {
        printf 'ERROR %s AGX master is not ready\n' "$name"
        return 2
    }
    cmd="$(component_command "$name" 2>/dev/null || true)"
    [[ -n "$cmd" ]] || {
        printf 'ERROR unknown component=%s\n' "$name"
        return 2
    }
    if [[ "$name" == "recorder" ]]; then
        stop_recording_before_cleanup
    fi
    stop_component "$name" >/dev/null
    # Components do not need sudo; keep the credential out of their process
    # environments and their /proc entries.
    unset AMMR_SUDO_PASSWORD
    nohup setsid bash -lc "$cmd" >"$(log_file "$name")" 2>&1 < /dev/null &
    pid=$!
    printf '%s\n' "$pid" >"$(pid_file "$name")"
    sleep 0.5
    if kill -0 "$pid" 2>/dev/null; then
        printf 'RUNNING %s PID=%s LOG=%s\n' "$name" "$pid" "$(log_file "$name")"
        return 0
    fi
    printf 'ERROR %s failed to start; LOG=%s\n' "$name" "$(log_file "$name")"
    return 1
}

all_components=(left_arm right_arm left_camera right_camera head_camera recorder chassis lift)

stop_all() {
    local name
    # Save an active episode before any cleanup, including a repeated
    # master_start.  On a fresh AGX there is no recorder PID, so this is
    # effectively free; on a stale process it prevents an in-progress file
    # from being killed without the recorder's normal finalization path.
    stop_recording_before_cleanup
    for name in "${all_components[@]}"; do
        stop_component "$name" >/dev/null
    done
    rm -f "$STATE_DIR/master.active"
}

setup_can() {
    local can_script="$HOME/pyAgxArm-master/pyAgxArm/scripts/ubuntu/setup_can2usb.bash"
    if ! lsmod | grep -q '^gs_usb '; then
        if [[ -f "$can_script" ]]; then
            (cd "$(dirname "$can_script")" && sudo_run insmod ./gs_usb.ko) || true
        else
            printf 'CAN_WARN setup script not found: %s\n' "$can_script"
        fi
    fi
    local retry present
    for retry in {1..20}; do
        present=1
        for name in can2 can3 can4; do
            ip link show "$name" >/dev/null 2>&1 || present=0
        done
        [[ "$present" == "1" ]] && break
        sleep 0.25
    done
    # Re-running `ip link set ... up type can ...` on an already-UP adapter
    # reports EBUSY. Only configure an interface that is missing or down.
    local name bitrate state
    for name in can2 can3 can4; do
        bitrate=1000000
        [[ "$name" == can2 ]] && bitrate=500000
        state="$(ip -br link show "$name" 2>/dev/null || true)"
        if [[ "$state" != *"UP"* ]]; then
            sudo_run ip link set "$name" up type can bitrate "$bitrate" || true
        fi
    done
}

master_start() {
    stop_all
    setup_can
    local name missing=0
    for name in can2 can3 can4; do
        if ! ip link show "$name" >/dev/null 2>&1; then
            printf 'CAN_ERROR missing %s\n' "$name"
            missing=1
        elif [[ "$(ip -br link show "$name" 2>/dev/null)" != *"UP"* ]]; then
            printf 'CAN_ERROR %s is not UP\n' "$name"
            missing=1
        fi
    done
    if (( missing )); then
        printf 'MASTER_ERROR CAN is not ready; run setup_can2usb.bash manually if sudo requires a password\n'
        return 2
    fi
    : >"$STATE_DIR/master.active"
    unset AMMR_SUDO_PASSWORD
    printf 'MASTER_READY host=%s domain=%s\n' "$(hostname)" "$DOMAIN_ID"
}

master_stop() {
    stop_all
    local name
    for name in can2 can3 can4; do
        sudo_run ip link set "$name" down 2>/dev/null || true
    done
    unset AMMR_SUDO_PASSWORD
    printf 'MASTER_STOPPED\n'
}

status_all() {
    local name pid can_ok=1
    for name in can2 can3 can4; do
        if [[ "$(ip -br link show "$name" 2>/dev/null || true)" != *"UP"* ]]; then
            can_ok=0
        fi
    done
    if [[ -f "$STATE_DIR/master.active" && "$can_ok" == "1" ]]; then
        printf 'MASTER READY\n'
    elif [[ -f "$STATE_DIR/master.active" ]]; then
        printf 'MASTER DEGRADED CAN_NOT_READY\n'
    else
        printf 'MASTER STOPPED\n'
    fi
    for name in "${all_components[@]}"; do
        pid="$(cat "$(pid_file "$name")" 2>/dev/null || true)"
        if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
            printf 'STATUS %s RUNNING PID=%s\n' "$name" "$pid"
        else
            printf 'STATUS %s STOPPED\n' "$name"
        fi
    done
}

case "$ACTION" in
    master_start) master_start ;;
    master_stop) master_stop ;;
    start) start_component "$COMPONENT" ;;
    stop)
        if [[ "$COMPONENT" == "recorder" ]]; then
            stop_recording_before_cleanup
        fi
        stop_component "$COMPONENT"
        ;;
    record_start) recording_service_call true ;;
    record_stop) recording_service_call false ;;
    status) status_all ;;
    cleanup) stop_all ;;
    *) printf 'ERROR usage: {master_start|master_stop|start|stop|record_start|record_stop|status|cleanup} [component]\n'; exit 2 ;;
esac
