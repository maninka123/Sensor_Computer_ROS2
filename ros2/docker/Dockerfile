FROM ros:jazzy-ros-base-noble@sha256:00b8d47a6e6384a2834e3d01fa87ae767876543baa574f2d64a75c04ea33c698
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config libapr1-dev libpcl-dev libopencv-dev \
    libeigen3-dev libboost-all-dev libusb-1.0-0-dev libyaml-cpp-dev \
    python3-colcon-common-extensions python3-venv python3-pip python3-opencv \
    python3-numpy python3-yaml python3-websocket python3-pytest \
    iproute2 iputils-ping usbutils procps curl ca-certificates \
    ros-jazzy-ament-cmake-auto ros-jazzy-cv-bridge ros-jazzy-pcl-conversions \
    ros-jazzy-camera-info-manager ros-jazzy-image-transport \
    ros-jazzy-diagnostic-updater ros-jazzy-rosbridge-server \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-tf2-ros \
    ros-jazzy-imu-complementary-filter \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/inference \
    && /opt/inference/bin/pip install --no-cache-dir \
       numpy==1.26.4 Pillow==11.1.0 torch==2.6.0 onnxruntime==1.20.1
ENV NODE_PC_INFERENCE_PYTHON=/opt/inference/bin/python
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED=1
ENV OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
WORKDIR /workspace/ros2
CMD ["bash"]
