#!/usr/bin/env bash

# Bring up the OpenArm bimanual MoveIt stack for interactive pose setting.
#
# demo.launch.py defaults use_fake_hardware to false, so an operator who omits
# the argument opens can0 and can1 and drives the real robot. This wrapper
# inverts that: fake hardware is what you get by omission, and reaching
# hardware takes --real plus the buses named explicitly.

set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
# A name no CAN device can have, so a fake run that somehow tried to open a
# bus would fail loudly instead of finding a real one.
FAKE_CAN="robot_control_fake_only"

usage() {
    cat <<'USAGE'
usage: pose_bringup.sh [--real --right-can IFACE --left-can IFACE] [-- ARGS...]

  (no options)  fake hardware; nothing touches CAN
  --real        drive the real robot; requires --right-can and --left-can
  --right-can   CAN interface for the right arm, for example can0
  --left-can    CAN interface for the left arm, for example can1
  -- ARGS...    extra name:=value arguments passed through to ros2 launch

Then set poses with `robotctl pose`; see docs/cli.md.
USAGE
}

real=false
right_can=""
left_can=""
extra=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --real) real=true; shift ;;
        --right-can) right_can="${2:-}"; shift 2 ;;
        --left-can) left_can="${2:-}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --) shift; extra=("$@"); break ;;
        *) echo "error: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: source a ROS 2 Jazzy environment before running this bringup" >&2
    exit 2
fi

SETUP="$WORKSPACE/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
    echo "error: missing workspace setup.bash: $SETUP" >&2
    echo "       build the workspace first with ros_ws/build.sh" >&2
    exit 2
fi

if [[ "$real" == true ]]; then
    if [[ -z "$right_can" || -z "$left_can" ]]; then
        echo "error: --real needs both --right-can and --left-can named explicitly" >&2
        echo "       refusing to guess which bus the robot is on" >&2
        exit 2
    fi
    echo "WARNING: driving REAL hardware on $right_can and $left_can." >&2
    echo "         Clear the workspace and keep the E-stop within reach." >&2
    use_fake_hardware=false
else
    right_can="$FAKE_CAN"
    left_can="$FAKE_CAN"
    use_fake_hardware=true
fi

# Colcon and ament setup files read COLCON_TRACE and AMENT_TRACE_SETUP_FILES
# without a default, so `set -u` must be off while sourcing. Strict mode is
# restored immediately afterwards.
set +eu
# shellcheck disable=SC1090
source "$SETUP"
set -euo pipefail

# exec so Ctrl-C reaches the launch directly and its status is this script's.
exec ros2 launch openarm_bimanual_moveit_config demo.launch.py \
    "use_fake_hardware:=$use_fake_hardware" \
    "right_can_interface:=$right_can" \
    "left_can_interface:=$left_can" \
    ${extra[@]+"${extra[@]}"}
