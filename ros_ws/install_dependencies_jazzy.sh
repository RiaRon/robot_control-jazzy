#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: source a ROS 2 Jazzy environment before installing dependencies" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"

sudo apt-get update
sudo apt-get install -y \
    libcli11-dev \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -r -y \
    --skip-keys "ign_ros2_control"
