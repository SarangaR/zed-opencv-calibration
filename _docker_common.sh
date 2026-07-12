# Shared helpers for the run_docker_*.sh wrappers. Source, don't execute.
#
# Handles: host architecture detection (x86_64 vs Jetson/aarch64), picking the
# right base image, building the local image when missing, NVIDIA runtime
# detection, and X11 access.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
ARCH="$(uname -m)"

# --runtime=nvidia only when the runtime is actually installed (webcam-only
# tools don't need a GPU; on Jetson the runtime is standard). The images set
# NVIDIA_VISIBLE_DEVICES, so --gpus is not required.
NVIDIA_ARGS=()
if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
  NVIDIA_ARGS=(--runtime=nvidia)
fi

# Interactive TTY only when one exists (lets the scripts run from CI/automation).
TTY_ARGS=()
if [ -t 0 ] && [ -t 1 ]; then
  TTY_ARGS=(-it)
fi

# Build a local image from a Dockerfile if it doesn't exist yet.
#   ensure_image <image_tag> <dockerfile> [build-args...]
ensure_image() {
  local image="$1" dockerfile="$2"
  shift 2
  if ! docker image inspect "${image}" >/dev/null 2>&1; then
    echo ">>> Building Docker image ${image} (one-time)"
    docker build -f "${SCRIPT_DIR}/${dockerfile}" -t "${image}" "$@" \
      "${SCRIPT_DIR}"
  fi
}

# Webcam-only test image (no ZED SDK). x86_64 reuses the borda OpenCV image
# as base; Jetson/aarch64 builds on plain Ubuntu (no CUDA needed).
ensure_webcam_image() {
  local base_args=()
  if [ "${ARCH}" = "aarch64" ]; then
    base_args=(--build-arg BASE_IMAGE=ubuntu:22.04)
  fi
  ensure_image zed-calib-webcam Dockerfile "${base_args[@]}"
}

# ZED SDK image. x86_64 uses the desktop CUDA tag; Jetson uses an l4t tag
# (JetPack 6 / L4T r36 on Orin Nano). Override with ZED_DOCKER_BASE if your
# JetPack or CUDA version differs.
ensure_zed_image() {
  local base
  if [ "${ARCH}" = "aarch64" ]; then
    base="${ZED_DOCKER_BASE:-stereolabs/zed:5.0-devel-l4t-r36.4}"
  else
    base="${ZED_DOCKER_BASE:-stereolabs/zed:5.2-gl-devel-cuda12.8-ubuntu22.04}"
  fi
  ensure_image zed-calib-stereo Dockerfile.zed --build-arg "ZED_BASE=${base}"
}

# Allow the container to talk to the host X server.
allow_x11() { xhost +local: >/dev/null; }

# Extra mounts needed by GMSL cameras (ZED X / X Mini) on Jetson.
ARGUS_MOUNT=()
if [ -S /tmp/argus_socket ]; then
  ARGUS_MOUNT=(-v /tmp/argus_socket:/tmp/argus_socket)
fi

# All /dev/video* nodes as --device args (webcam tools).
video_devices() {
  DEVICES=()
  local d
  for d in /dev/video*; do
    [ -e "$d" ] && DEVICES+=(--device "$d")
  done
}
