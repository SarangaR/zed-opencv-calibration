#!/usr/bin/env bash
#
# Run the STEREO CHECKER (live reprojection-error monitor) for a real ZED
# stereo camera inside a ZED SDK Docker container, validating an existing
# calibration. Works on x86_64 and Jetson Orin (aarch64, l4t base image +
# /tmp/argus_socket mount for GMSL cameras).
#
# Same host requirements as run_docker_zed.sh.
#
# Usage:
#   ./run_docker_zed_checker.sh [tool args...]   # default: --charuco
#
# Examples:
#   ./run_docker_zed_checker.sh
#   ./run_docker_zed_checker.sh --charuco --calib_opencv zed_calibration_SN12345678.yml
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_docker_common.sh"

ensure_zed_image
allow_x11

ARGS=("$@")
if [ "${#ARGS[@]}" -eq 0 ]; then
  ARGS=(--charuco --mirror)
fi

mkdir -p "${SCRIPT_DIR}/.zed-docker/settings" "${SCRIPT_DIR}/.zed-docker/resources"

docker run --rm "${TTY_ARGS[@]}" "${NVIDIA_ARGS[@]}" --privileged \
  -e DISPLAY="${DISPLAY}" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  "${ARGUS_MOUNT[@]}" \
  -v "${SCRIPT_DIR}":/root/zed-opencv-calibration \
  -v "${SCRIPT_DIR}/.zed-docker/settings":/usr/local/zed/settings \
  -v "${SCRIPT_DIR}/.zed-docker/resources":/usr/local/zed/resources \
  zed-calib-stereo \
  bash -c "cd /root/zed-opencv-calibration \
    && cmake -S . -B build-zed-docker \
    && cmake --build build-zed-docker --target zed_stereo_checker -j\$(nproc) \
    && exec ./build-zed-docker/stereo_checker/zed_stereo_checker $(printf '%q ' "${ARGS[@]}")"
