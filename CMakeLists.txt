cmake_minimum_required(VERSION 3.5)
project(zed_opencv_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# WEBCAM_ONLY: build only the ZED-free monocular tools against plain OpenCV
# (cv::VideoCapture input via --webcam). Lets the ChArUco pipeline be tested in
# an environment without the ZED SDK, CUDA, or a ZED camera.
option(WEBCAM_ONLY "Build only ZED-free tools (webcam test mode)" OFF)

if(WEBCAM_ONLY)
  add_subdirectory(monocular_calibration)
  add_subdirectory(monocular_checker)
else()
  add_subdirectory(stereo_calibration)
  add_subdirectory(stereo_reprojection_viewer)
  add_subdirectory(monocular_calibration)
  add_subdirectory(stereo_checker)
  add_subdirectory(monocular_checker)
endif()