#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
    echo "This installer targets Ubuntu 22.04 / ROS 2 Humble." >&2
    exit 1
fi
sudo apt-get update
sudo apt-get install --no-install-recommends \
    build-essential cmake git iproute2 util-linux \
    python3-venv python3-colcon-common-extensions python3-yaml python3-numpy python3-pil \
    python3-empy python3-lark python3-catkin-pkg \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan \
    ros-humble-rviz2
