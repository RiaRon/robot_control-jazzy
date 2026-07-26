#!/usr/bin/env bash

# Load the effort controllers into an already running stack.
#
# The vendored bringup claims the position interface only, so every MIT command
# carries tau = 0 and each joint holds position by sitting short of its command.
# These controllers claim the effort interfaces beside the trajectory
# controllers, which is what lets a gravity feedforward be published without the
# trajectory controllers giving up position.
#
# Loaded at runtime rather than merged into the vendor controller file, because
# demo.launch.py passes the controller manager exactly one parameter file:
# adding them there would mean patching the vendor snapshot or keeping a copy of
# the whole vendor file in step with it. This also leaves the effort interfaces
# unclaimed until compensation is actually asked for.

set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$WORKSPACE/config/effort_controllers.yaml"
TYPE="forward_command_controller/ForwardCommandController"

usage() {
    cat <<'USAGE'
usage: load_effort_controllers.sh [right|left|both]

  both  (default) load the effort controller for each arm

Requires a running stack; start one with pose_bringup.sh first. The controllers
are additive: the trajectory controllers keep the position interfaces.
USAGE
}

case "${1:-both}" in
    right) sides=(right) ;;
    left) sides=(left) ;;
    both) sides=(right left) ;;
    -h|--help) usage; exit 0 ;;
    *) echo "error: unknown argument: $1" >&2; usage >&2; exit 2 ;;
esac

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: source a ROS 2 Jazzy environment first" >&2
    exit 2
fi

if [[ ! -f "$CONFIG" ]]; then
    echo "error: missing controller parameters: $CONFIG" >&2
    exit 2
fi

if ! ros2 control list_controllers >/dev/null 2>&1; then
    echo "error: no controller manager is answering" >&2
    echo "       start the stack first: ros_ws/pose_bringup.sh" >&2
    exit 2
fi

for side in "${sides[@]}"; do
    name="${side}_forward_effort_controller"
    if ros2 control list_controllers | grep -q "^${name} "; then
        echo "$name is already loaded"
        continue
    fi
    # The controller manager reads a controller's type from its own parameters,
    # and it was started before this file existed, so the type has to be set on
    # the running node before the load can resolve it.
    ros2 param set /controller_manager "${name}.type" "$TYPE" >/dev/null
    ros2 control load_controller --set-state active "$name" "$CONFIG"
done

echo
echo "effort interfaces now claimed:"
ros2 control list_controllers | grep effort || echo "  none - check the log above"
