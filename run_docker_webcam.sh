#!/usr/bin/env bash
#
# Build (if needed) the zed-calib-webcam Docker image and run the webcam-only
# monocular CALIBRATION GUI inside it. Works on x86_64 and Jetson (aarch64).
#
# Usage:
#   ./run_docker_webcam.sh [tool args...]     # default: --webcam 0 --charuco --mirror
#
# Examples:
#   ./run_docker_webcam.sh
#   ./run_docker_webcam.sh --webcam 1 --charuco --squares_x 15 --squares_y 11
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_docker_common.sh"

ensure_webcam_image
allow_x11
video_devices

ARGS=("$@")
if [ "${#ARGS[@]}" -eq 0 ]; then
  ARGS=(--webcam 0 --charuco --mirror)
fi

docker run --rm "${TTY_ARGS[@]}" "${NVIDIA_ARGS[@]}" \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${SCRIPT_DIR}":/root/zed-opencv-calibration \
  "${DEVICES[@]}" \
  zed-calib-webcam \
  bash -c "cd /root/zed-opencv-calibration && exec ./build_and_run.sh mono_calib --webcam-only -- $(printf '%q ' "${ARGS[@]}")"
