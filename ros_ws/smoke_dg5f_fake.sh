#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: source a ROS 2 Jazzy environment before running this smoke test" >&2
    exit 2
fi

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
REPOSITORY="$(cd "$WORKSPACE/.." && pwd)"
SETUP="$WORKSPACE/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "error: missing workspace setup: $SETUP" >&2
    exit 2
fi

# shellcheck disable=SC1090
source "$SETUP"

LAUNCH_PID=""
cleanup() {
    local status=$?
    trap - EXIT INT TERM HUP
    set +e
    if [[ -n "$LAUNCH_PID" ]]; then
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
        for ((attempt = 0; attempt < 50; attempt++)); do
            kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 -- "-$LAUNCH_PID" 2>/dev/null; then
            kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
        fi
        wait "$LAUNCH_PID" 2>/dev/null
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

setsid ros2 launch dg5f_gz dg5f_right_gz.launch.py \
    use_fake_hardware:=true \
    gui:=false &
LAUNCH_PID=$!

LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d '[:space:]')"
if [[ "$LAUNCH_PGID" != "$LAUNCH_PID" ]]; then
    echo "error: launch did not enter its own process group" >&2
    exit 1
fi
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "error: DG5F fake-hardware launch exited unexpectedly" >&2
    exit 1
fi

timeout --signal=TERM --kill-after=5s 55s \
    python3 "$REPOSITORY/tools/ros_smoke.py" \
    --robot dg5f \
    --controller-timeout 30 \
    --state-timeout 10 \
    --tolerance 0.02

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "error: DG5F fake-hardware launch exited unexpectedly" >&2
    exit 1
fi
echo "DG5F fake-hardware smoke test passed"
