cmake_minimum_required(VERSION 3.5)
project(zed_mono_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${OpenCV_LIBRARY_DIRS})

if(WEBCAM_ONLY)
  # ZED-free test build: cv::VideoCapture input only (--webcam required).
  add_definitions(-DWEBCAM_ONLY)
else()
  find_package(ZED 5 REQUIRED)
  find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  include_directories(${ZED_INCLUDE_DIRS})
  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${CUDA_LIBRARY_DIRS})
endif()

FILE(GLOB_RECURSE SRC_FILES "src/*.cpp")
FILE(GLOB_RECURSE HDR_FILES "include/*.hpp")
ADD_EXECUTABLE(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

if(WEBCAM_ONLY)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBRARIES})
else()
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZED_LIBRARIES} ${CUDA_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
