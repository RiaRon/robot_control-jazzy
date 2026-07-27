#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "error: source a ROS 2 Humble environment before building this branch" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
# --log-base is a global colcon option; colcon-core 0.21 rejects it after the verb.
colcon --log-base "$WORKSPACE/log" build \
    --base-paths "$WORKSPACE/src" \
    --build-base "$WORKSPACE/build" \
    --install-base "$WORKSPACE/install" \
    --symlink-install
