#!/usr/bin/env bash
#
# Build (if needed) and run one of the ZED calibration tools.
# Run with --help for full usage.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Build (if needed) and run one of the ZED calibration tools.

Usage:
  ./build_and_run.sh <tool> [--rebuild] [--webcam-only] [-- <tool arguments...>]

  <tool> is one of:
    mono_calib    -> zed_mono_calibration     (monocular_calibration)
    mono_check    -> zed_mono_checker         (monocular_checker)
    stereo_calib  -> zed_stereo_calibration   (stereo_calibration)
    stereo_check  -> zed_stereo_checker       (stereo_checker)
    viewer        -> zed_reprojection_viewer  (stereo_reprojection_viewer)

  --rebuild       Force a clean reconfigure + rebuild before running.
  --webcam-only   ZED-free test build (cmake -DWEBCAM_ONLY=ON, links OpenCV
                  only, no ZED SDK/CUDA/camera needed). Uses the separate
                  build-webcam/ directory. Only mono_calib and mono_check
                  exist in this mode, and they require a --webcam <id|path>
                  tool argument.
  Everything after the flags (or after a literal --) is passed straight
  through to the tool.

Examples:
  ./build_and_run.sh mono_calib --charuco
  ./build_and_run.sh stereo_calib -- --charuco --squares_x 15 --squares_y 11 \
                                     --square_size 15 --marker_size 11
  ./build_and_run.sh stereo_check --rebuild -- --calib_opencv zed_calibration_SN123.yml

  # Webcam test mode (e.g. inside an OpenCV-only Docker image):
  ./build_and_run.sh mono_calib --webcam-only -- --webcam 0 --charuco
  ./build_and_run.sh mono_calib --webcam-only -- --webcam clip.mp4 --charuco
  ./build_and_run.sh mono_check --webcam-only -- --webcam 0 --charuco \
                                   --calib_opencv mono_calibration_SN0.yml
EOF
  exit "${1:-0}"
}

if [ "$#" -lt 1 ]; then usage 1; fi

TOOL_ALIAS="$1"; shift

case "${TOOL_ALIAS}" in
  -h|--help|help) usage 0 ;;
esac

# Map tool alias -> (subdirectory, executable name)
case "${TOOL_ALIAS}" in
  mono_calib)   SUBDIR="monocular_calibration";       EXE="zed_mono_calibration" ;;
  mono_check)   SUBDIR="monocular_checker";           EXE="zed_mono_checker" ;;
  stereo_calib) SUBDIR="stereo_calibration";          EXE="zed_stereo_calibration" ;;
  stereo_check) SUBDIR="stereo_checker";              EXE="zed_stereo_checker" ;;
  viewer)       SUBDIR="stereo_reprojection_viewer";  EXE="zed_reprojection_viewer" ;;
  *)
    echo "Unknown tool '${TOOL_ALIAS}'." >&2
    echo "Valid tools: mono_calib, mono_check, stereo_calib, stereo_check, viewer" >&2
    exit 1
    ;;
esac

# Optional flags right after the tool name (any order).
REBUILD=0
WEBCAM_ONLY=0
while [ "$#" -ge 1 ]; do
  case "$1" in
    --rebuild)     REBUILD=1; shift ;;
    --webcam-only) WEBCAM_ONLY=1; shift ;;
    *) break ;;
  esac
done

# Drop an optional literal "--" separator before the passthrough arguments.
if [ "$#" -ge 1 ] && [ "$1" = "--" ]; then
  shift
fi

# Webcam-only mode: separate ZED-free build tree; mono tools only.
if [ "${WEBCAM_ONLY}" -eq 1 ]; then
  case "${TOOL_ALIAS}" in
    mono_calib|mono_check) ;;
    *)
      echo "--webcam-only builds only mono_calib and mono_check (stereo tools need a ZED)." >&2
      exit 1
      ;;
  esac
  BUILD_DIR="${SCRIPT_DIR}/build-webcam"
  CMAKE_EXTRA=(-DWEBCAM_ONLY=ON)
else
  BUILD_DIR="${SCRIPT_DIR}/build"
  CMAKE_EXTRA=()
fi

EXE_PATH="${BUILD_DIR}/${SUBDIR}/${EXE}"

need_build=0
if [ "${REBUILD}" -eq 1 ]; then
  need_build=1
elif [ ! -x "${EXE_PATH}" ]; then
  need_build=1
fi

if [ "${need_build}" -eq 1 ]; then
  if [ "${REBUILD}" -eq 1 ] && [ -d "${BUILD_DIR}" ]; then
    echo ">>> Removing existing build directory for a clean rebuild"
    rm -rf "${BUILD_DIR}"
  fi
  echo ">>> Configuring (cmake) in ${BUILD_DIR}"
  mkdir -p "${BUILD_DIR}"
  cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" ${CMAKE_EXTRA[@]+"${CMAKE_EXTRA[@]}"}

  # Build only the requested target for speed.
  JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"
  echo ">>> Building ${EXE} (-j${JOBS})"
  cmake --build "${BUILD_DIR}" --target "${EXE}" -j"${JOBS}"
else
  echo ">>> ${EXE} already built (use --rebuild to force). Skipping build."
fi

if [ ! -x "${EXE_PATH}" ]; then
  echo "Build did not produce '${EXE_PATH}'." >&2
  exit 1
fi

echo ">>> Running: ${EXE_PATH} $*"
echo
exec "${EXE_PATH}" "$@"
