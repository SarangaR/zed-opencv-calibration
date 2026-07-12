# Webcam-only calibration test image.
#
# The base image's OpenCV 4.13 was compiled with the `highgui` module
# disabled (no cv::imshow) and ships no C++ build tools, so the GUI
# calibration tools cannot be built or run against it directly. This image
# adds build tools and a minimal CPU-only OpenCV 4.13 build (with highgui/
# GTK3, V4L2 webcam capture, and objdetect for ChArUco) installed to
# /opt/opencv, leaving the base image's Python OpenCV untouched.
#
# Build:
#   docker build -t zed-calib-webcam .
#
# Run (from the repo root):
#   xhost +local:
#   docker run --rm -it --runtime=nvidia --gpus all \
#     -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v "$PWD":/root/zed-opencv-calibration \
#     --device /dev/video0 --device /dev/video1 \
#     zed-calib-webcam \
#     bash -c 'cd /root/zed-opencv-calibration && ./build_and_run.sh mono_calib --webcam-only --rebuild -- --webcam 0 --charuco'

# x86_64 default. On Jetson / aarch64 the webcam-only build needs no CUDA at
# all, so a plain Ubuntu base works: --build-arg BASE_IMAGE=ubuntu:22.04
# (run_docker_webcam*.sh selects this automatically).
ARG BASE_IMAGE=borda/docker_python-opencv-ffmpeg:gpu-py3.11-cv4.13.0
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake wget ca-certificates unzip pkg-config \
        libgtk-3-dev \
    && rm -rf /var/lib/apt/lists/*

# Minimal OpenCV build: only the modules the calibration tools need
# (BUILD_LIST pulls in required dependencies automatically).
RUN wget -q https://github.com/opencv/opencv/archive/refs/tags/4.13.0.tar.gz -O /tmp/opencv.tar.gz \
    && tar -xzf /tmp/opencv.tar.gz -C /tmp \
    && cmake -S /tmp/opencv-4.13.0 -B /tmp/opencv-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/opencv \
        -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,flann,objdetect \
        -DWITH_GTK=ON \
        -DWITH_QT=OFF \
        -DWITH_V4L=ON \
        -DWITH_CUDA=OFF \
        -DBUILD_opencv_python3=OFF \
        -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
        -DBUILD_opencv_apps=OFF \
    && cmake --build /tmp/opencv-build -j"$(nproc)" \
    && cmake --install /tmp/opencv-build \
    && rm -rf /tmp/opencv.tar.gz /tmp/opencv-4.13.0 /tmp/opencv-build

# Make the GUI-capable OpenCV win over the base image's headless one in
# /usr/local, both at configure time (find_package) and at run time.
RUN echo /opt/opencv/lib > /etc/ld.so.conf.d/00-opencv-gui.conf && ldconfig
ENV CMAKE_PREFIX_PATH=/opt/opencv

WORKDIR /root/zed-opencv-calibration
