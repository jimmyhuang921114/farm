FROM ros:humble-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-serial \
        ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /farm
