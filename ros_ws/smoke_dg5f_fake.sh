#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
REPOSITORY="$(cd "$WORKSPACE/.." && pwd)"
SMOKE_NAME="DG5F fake-hardware"
SMOKE_WORKSPACE="$WORKSPACE"
SMOKE_REPOSITORY="$REPOSITORY"
SMOKE_LAUNCH_COMMAND=(ros2 launch dg5f_gz dg5f_right_gz.launch.py
    use_fake_hardware:=true \
    gui:=false)
SMOKE_VALIDATOR_COMMAND=(python3 "$REPOSITORY/tools/ros_smoke.py"
    --robot dg5f \
    --controller-timeout 30 \
    --state-timeout 10 \
    --tolerance 0.02)

# shellcheck source=smoke_harness.sh
source "$WORKSPACE/smoke_harness.sh"
run_smoke_harness
