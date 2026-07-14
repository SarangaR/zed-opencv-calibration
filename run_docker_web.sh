#!/usr/bin/env bash
#
# Run the WEB UI for the whole calibration toolkit (mono/stereo calibration +
# checkers) inside a ZED SDK Docker container. No X11 needed: the GUI is a
# website served on port 30000 — open http://<jetson-ip>:30000 from the
# tethered machine.
#
# Works on x86_64 and Jetson Orin (aarch64, l4t base image + /tmp/argus_socket
# mount for GMSL cameras). Same host requirements as run_docker_zed.sh.
#
# Usage:
#   ./run_docker_web.sh [--port <p>]     # default 30000
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_docker_common.sh"

PORT=30000
if [ "${1:-}" = "--port" ] && [ -n "${2:-}" ]; then
  PORT="$2"
fi

ensure_zed_image

# Persist the SDK's camera settings/resources between runs so calibration
# files downloaded by the SDK are cached.
mkdir -p "${SCRIPT_DIR}/.zed-docker/settings" "${SCRIPT_DIR}/.zed-docker/resources"

docker run --rm "${TTY_ARGS[@]}" "${NVIDIA_ARGS[@]}" --privileged \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -p "${PORT}:${PORT}" \
  -v /dev:/dev \
  "${ARGUS_MOUNT[@]}" \
  -v "${SCRIPT_DIR}":/root/zed-opencv-calibration \
  -v "${SCRIPT_DIR}/.zed-docker/settings":/usr/local/zed/settings \
  -v "${SCRIPT_DIR}/.zed-docker/resources":/usr/local/zed/resources \
  zed-calib-stereo \
  bash -c "cd /root/zed-opencv-calibration \
    && cmake -S . -B build-zed-docker \
    && cmake --build build-zed-docker --target zed_calib_web -j\$(nproc) \
    && exec python3 web/server.py --port ${PORT} \
         --engine ./build-zed-docker/web_tool/zed_calib_web"
