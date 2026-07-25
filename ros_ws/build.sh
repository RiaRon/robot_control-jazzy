#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "error: source a ROS 2 Humble environment before building this branch" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
colcon build \
    --base-paths "$WORKSPACE/src" \
    --build-base "$WORKSPACE/build" \
    --install-base "$WORKSPACE/install" \
    --log-base "$WORKSPACE/log" \
    --symlink-install
