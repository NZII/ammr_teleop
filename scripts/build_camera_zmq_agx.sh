#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_DIR="${PROJECT_ROOT}/native/camera_zmq"
BUILD_DIR="${PROJECT_ROOT}/build/camera_zmq"
INSTALL_DIR="${PROJECT_ROOT}/install/camera_zmq"

cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
cmake --build "${BUILD_DIR}" --parallel
cmake --install "${BUILD_DIR}"
echo "Installed ${INSTALL_DIR}/bin/ammr_camera_zmq_publisher"

