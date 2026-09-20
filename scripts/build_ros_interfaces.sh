#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/path_env.sh"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$AMMR_ROS_WS"
exec /usr/bin/colcon build --packages-select lifting_msg_pkg --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
