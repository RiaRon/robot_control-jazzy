#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
REPOSITORY="$(cd "$WORKSPACE/.." && pwd)"
SMOKE_NAME="OpenArm pose-setting"
SMOKE_WORKSPACE="$WORKSPACE"
SMOKE_REPOSITORY="$REPOSITORY"
# The same wrapper an operator uses, so this check would catch the wrapper
# regressing to the vendor's real-hardware default.
SMOKE_LAUNCH_COMMAND=("$WORKSPACE/pose_bringup.sh")
SMOKE_VALIDATOR_COMMAND=(python3 "$REPOSITORY/tools/pose_smoke.py"
    --group openarm_right_arm \
    --offset-z 0.03 \
    --tolerance 0.005 \
    --timeout 35)

# The validator imports robot_control and re-invokes robotctl. Prepending keeps
# the branch checkout ahead of any installed copy; the ROS setup files prepend
# their own paths afterwards without dropping this one.
export PYTHONPATH="$REPOSITORY/src:${PYTHONPATH:-}"

# shellcheck source=smoke_harness.sh
source "$WORKSPACE/smoke_harness.sh"
run_smoke_harness
