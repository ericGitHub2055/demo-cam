FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates \
    python3-pip python3-colcon-common-extensions \
    v4l-utils \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-usb-cam \
    ros-jazzy-apriltag-ros \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /work/demo-cam
