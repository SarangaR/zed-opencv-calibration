#!/usr/bin/env bash
#
# Run the STEREO CALIBRATION GUI for a real ZED stereo camera (e.g. ZED X Mini)
# inside a ZED SDK Docker container. Works on x86_64 and Jetson Orin (aarch64,
# l4t base image + /tmp/argus_socket mount for GMSL cameras).
#
# Requirements on the host:
#   - nvidia-container-toolkit (--runtime=nvidia)
#   - For GMSL cameras (ZED X / X Mini): ZED Link driver + ZED X daemon
#     running on the host.
#   - Override the SDK base image with ZED_DOCKER_BASE if your JetPack/CUDA
#     version differs (see _docker_common.sh for the defaults).
#
# Usage:
#   ./run_docker_zed.sh [tool args...]        # default: --charuco --mirror
#
# Examples:
#   ./run_docker_zed.sh
#   ./run_docker_zed.sh --charuco --squares_x 15 --squares_y 11 \
#                       --square_size 15 --marker_size 11
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_docker_common.sh"

ensure_zed_image
allow_x11

ARGS=("$@")
if [ "${#ARGS[@]}" -eq 0 ]; then
  ARGS=(--charuco --mirror)
fi

# Persist the SDK's camera settings/resources between runs so calibration
# files downloaded by the SDK are cached.
mkdir -p "${SCRIPT_DIR}/.zed-docker/settings" "${SCRIPT_DIR}/.zed-docker/resources"

# Stereo tools build in a separate in-container tree (build-zed-docker/).
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
    && cmake --build build-zed-docker --target zed_stereo_calibration -j\$(nproc) \
    && exec ./build-zed-docker/stereo_calibration/zed_stereo_calibration $(printf '%q ' "${ARGS[@]}")"
