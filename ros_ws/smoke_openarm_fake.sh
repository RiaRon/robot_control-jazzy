#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
REPOSITORY="$(cd "$WORKSPACE/.." && pwd)"
SMOKE_NAME="OpenArm fake-hardware"
SMOKE_WORKSPACE="$WORKSPACE"
SMOKE_REPOSITORY="$REPOSITORY"
SMOKE_LAUNCH_COMMAND=(ros2 launch openarm_bringup openarm.launch.py
    use_fake_hardware:=true \
    can_interface:=robot_control_fake_only)
SMOKE_VALIDATOR_COMMAND=(python3 "$REPOSITORY/tools/ros_smoke.py"
    --robot openarm \
    --controller-timeout 30 \
    --state-timeout 10 \
    --tolerance 0.02)

# shellcheck source=smoke_harness.sh
source "$WORKSPACE/smoke_harness.sh"
run_smoke_harness
