#!/usr/bin/env bash
# Load the effort controllers `r2s identify` publishes gravity feedforward on.
#
# They are declared in the bringup's controller configuration but deliberately
# not spawned with it: this puts a torque path onto a powered arm, and that
# should be something somebody ran on purpose. Run it when a sweep needs one,
# and `--unload` when the sweep is done.
#
# The trajectory controller keeps position throughout. The hardware sends both
# in one MIT-mode packet — tau = kp(q_des - q) + kd(qd_des - qd) + tau_ff — so
# the effort command is a feedforward term added to the position loop, not a
# replacement for it. Nothing here releases the arm.
#
# Usage:
#   ./ros_ws/load_effort_controllers.sh [right|left|both]   (default: both)
#   ./ros_ws/load_effort_controllers.sh --unload [right|left|both]

set -euo pipefail

if ! command -v ros2 > /dev/null 2>&1; then
    echo "error: ros2 is not on PATH; source /opt/ros/humble/setup.bash and this" >&2
    echo "       workspace's install/setup.bash first" >&2
    exit 2
fi

UNLOAD=false
if [[ "${1:-}" == "--unload" ]]; then
    UNLOAD=true
    shift
fi

case "${1:-both}" in
    right) CONTROLLERS=(right_forward_effort_controller) ;;
    left) CONTROLLERS=(left_forward_effort_controller) ;;
    both) CONTROLLERS=(right_forward_effort_controller left_forward_effort_controller) ;;
    *)
        echo "error: expected right, left, or both; got '${1}'" >&2
        exit 2
        ;;
esac

if ! ros2 control list_controllers > /dev/null 2>&1; then
    echo "error: no controller_manager is responding; is the bringup running?" >&2
    exit 2
fi

for controller in "${CONTROLLERS[@]}"; do
    if [[ "$UNLOAD" == true ]]; then
        # Deactivate before unloading so the interface is released with a zero
        # command rather than whatever was last written to it.
        ros2 control set_controller_state "$controller" inactive || true
        ros2 control unload_controller "$controller"
        echo "unloaded $controller"
    else
        ros2 control load_controller --set-state active "$controller"
        echo "loaded and activated $controller"
    fi
done

ros2 control list_controllers | grep -E "forward_effort|joint_trajectory" || true
