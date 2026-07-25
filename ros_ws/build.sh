#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: source a ROS 2 Jazzy environment before building this branch" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
SUPPORTED_PACKAGES_FILE="$WORKSPACE/supported-packages.txt"
mapfile -t SUPPORTED_PACKAGES < <(
    sed -e '/^[[:space:]]*#/d' -e '/^[[:space:]]*$/d' "$SUPPORTED_PACKAGES_FILE"
)

colcon --log-base "$WORKSPACE/log" build \
    --base-paths "$WORKSPACE/src" \
    --build-base "$WORKSPACE/build" \
    --install-base "$WORKSPACE/install" \
    --symlink-install \
    --packages-up-to "${SUPPORTED_PACKAGES[@]}"
