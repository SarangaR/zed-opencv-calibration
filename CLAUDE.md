# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

C++17 camera-calibration toolkit for ZED cameras built on the ZED SDK (>= 5.1) and OpenCV 4.x. It produces five independent command-line executables, each in its own top-level directory with its own `CMakeLists.txt`. See `README.md` for the full user-facing manual (CLI flags, checkerboard workflow, quality-metric meanings, output-file usage in ZED SDK apps).

## Build

The root `CMakeLists.txt` `add_subdirectory`s all five tools; there is no shared library — each subproject compiles standalone.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)      # Release by default (set in every CMakeLists.txt)
```

Binaries land under `build/<subdir>/`:

| Directory | Executable | Purpose |
|---|---|---|
| `monocular_calibration` | `zed_mono_calibration` | Intrinsics for a single ZED X One |
| `monocular_checker` | `zed_mono_checker` | Live reprojection-error monitor (mono) |
| `stereo_calibration` | `zed_stereo_calibration` | Intrinsics + extrinsics for stereo / virtual-stereo rigs |
| `stereo_checker` | `zed_stereo_checker` | Live reprojection-error monitor (stereo) |
| `stereo_reprojection_viewer` | `zed_reprojection_viewer` | OpenGL point-cloud + reprojection overlay |

To rebuild a single tool, `make <project_name>` (e.g. `make zed_stereo_calibration`) from `build/`.

Dependencies: ZED SDK 5, OpenCV, CUDA (version pinned by `${ZED_CUDA_VERSION}`), plus GLEW/freeGLUT/OpenGL — the OpenGL libs are only needed by `stereo_reprojection_viewer`. There is no test suite; validation is done at runtime with the checker/viewer tools against a physical camera or an `.svo` file.

## Architecture

### Duplicated-not-shared code

`CameraCalib` / `StereoCalib` structs (`include/opencv_calibration.hpp`) and the `CalibrationChecker` class (`include/calibration_checker.hpp`) exist as **separate copies** inside `monocular_calibration/` and `stereo_calibration/`. The mono and stereo versions have diverged (e.g. mono `CameraCalib` carries its own `imageSize`; stereo keeps it on `StereoCalib`). **A change to calibration logic must be applied to each copy deliberately** — editing one does not affect the other.

### Two-phase calibration flow

The calibration tools split responsibilities between `main.cpp` and `opencv_calibration.cpp`:

1. **Acquisition (`src/main.cpp`)** — opens the ZED camera(s), runs the live OpenCV-window GUI, detects checkerboard corners each frame, and gates samples through `CalibrationChecker::testSample`. `CalibrationChecker` (`src/calibration_checker.cpp`) scores each accepted sample for X/Y coverage, size range, and skew range, and decides when enough diverse data exists. Frames are also rejected below a Laplacian-variance sharpness threshold. Accepted frames are written to disk (`zed-images/` by default).
2. **Computation (`opencv_calibration.cpp`)** — the free function `calibrate(...)` reloads saved images, re-detects corners, and calls the struct methods `mono_calibrate` / `stereo_calibrate`, then `saveCalibOpenCV` and `saveCalibZED`. Stereo calibrates left intrinsics, then right, then extrinsics (R, T); outlier rejection is applied after stereo calibration.

Acquisition auto-triggers computation when all quality metrics hit 100% at the minimum sample count, or when the maximum sample count is reached. Tuning constants (`min_samples`, `max_samples`, coverage/skew thresholds, `max_repr_error`, `min_sharpness`) are file-scope constants near the top of each `main.cpp`.

### Calibration target detection (`BoardDetector`)

Both a classic chessboard (default) and a ChArUco board (`--charuco`) are supported by the four calibration/checker tools. Detection is abstracted behind `board_detector.hpp/.cpp` — a **per-tool copy** exists in each of `monocular_calibration`, `stereo_calibration`, `monocular_checker`, `stereo_checker` (checkers keep both files in `src/` since they have no `include/` on the path). `BoardDetector::detect()` returns a `BoardDetection` with aligned `imagePoints`/`objectPoints` plus, for ChArUco, per-corner `ids`. All OpenCV version handling is isolated inside `board_detector.cpp` behind a `CV_VERSION` guard: the 4.7+ `cv::aruco::CharucoDetector` (core `objdetect`) vs the ≤4.6 `cv::aruco::interpolateCornersCharuco` (contrib `aruco`).

Key consequences of ChArUco's partial detection (things that differ from the chessboard's all-or-nothing dense grid):
- **Per-frame object points.** `calibrate()` builds object/image points per frame from the detected id subset instead of reusing one fixed `pattern_points` grid.
- **ID-intersection for stereo.** `stereo_calibration`/`stereo_checker` pair left/right corners by shared ChArUco id (not by dense row-major index lockstep).
- **`CalibrationChecker`** takes an `is_charuco` flag + per-corner `ids`; its coverage/skew geometry (`get_outside_corners`) picks the 4 extreme corners by id when the board is partial.
- **Full-resolution live detection.** ArUco marker detection is scale-sensitive, so ChArUco mode detects on the full-res frame (chessboard mode keeps the faster downscaled detect); corners are scaled down only for display.

Board parameters flow through a `BoardConfig` struct; `calibrate()` takes it instead of the old `h_edges/v_edges/square_size` args.

### Distortion models

Every `CameraCalib` chooses between two OpenCV backends based on `disto_model_RadTan`:

- **Radial-Tangential** (default): `cv::calibrateCamera` / `cv::stereoCalibrate`. With >= 8 distortion coefficients the `CALIB_RATIONAL_MODEL` flag is added (6 radial + 2 tangential).
- **Fisheye** (`--fisheye`): `cv::fisheye::*` with a 4-coefficient model (k1..k4).

`CameraCalib::setFrom(sl::CameraParameters)` infers the model from the SDK's `disto[]` array (fisheye is detected when `disto[2]==0 && disto[3]==0 && disto[4]!=0 && disto[5]!=0`, remapping k3/k4). Corner data must be `cv::Point3f`/`Point2f` — the OpenCV calibration APIs reject the `double` variants (noted in the headers).

### Output files

- OpenCV YAML: `mono_calibration_SN<sn>.yml` (mono) / `zed_calibration_SN<sn>.yml` (stereo) — load via `sl::InitParameters(One)::optional_opencv_calibration_file`.
- ZED `.conf`: `SN<sn>_mono.conf` / `SN<sn>.conf` — the `saveCalibZED` methods emit per-resolution intrinsic sections for every supported ZED X One output resolution (`is_4k` selects the 4K vs GS resolution set). Drop into the ZED settings folder (`/usr/local/zed/settings/` on Linux) for automatic pickup.

### Camera selection & virtual stereo

All tools accept a live camera (default first connected), a specific `--id`/`--sn`, or an `--svo` recording. Stereo tools additionally support `--virtual` (two ZED X One cameras as one rig, selected with `--left_*`/`--right_*`); the ZED SDK synthesizes a combined serial number for the virtual rig, which is the identifier to use when loading the resulting calibration.

### Reprojection viewer

`stereo_reprojection_viewer` is the only tool with an OpenGL frontend (`GLViewer.hpp/.cpp`, GLEW + freeGLUT). `main.cpp` computes depth and reprojects the 3D point cloud onto the unrectified left image (depth color-coded), shown alongside the rectified image and the 3D cloud for visual calibration inspection.
