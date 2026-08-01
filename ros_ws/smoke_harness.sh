#!/usr/bin/env bash

# Shared lifecycle for the thin smoke-test entrypoints. This file is sourced;
# callers provide SMOKE_NAME, SMOKE_WORKSPACE, SMOKE_REPOSITORY,
# SMOKE_LAUNCH_COMMAND, and SMOKE_VALIDATOR_COMMAND.

_SMOKE_LAUNCH_PID=""
_SMOKE_VALIDATOR_PID=""
_SMOKE_CLEANED=0

_smoke_signal_groups() {
    local signal_name="$1"
    local pid
    for pid in "$_SMOKE_VALIDATOR_PID" "$_SMOKE_LAUNCH_PID"; do
        if [[ -n "$pid" ]]; then
            kill "-$signal_name" -- "-$pid" 2>/dev/null || true
        fi
    done
}

_smoke_groups_alive() {
    local pid
    for pid in "$_SMOKE_VALIDATOR_PID" "$_SMOKE_LAUNCH_PID"; do
        if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
            return 0
        fi
    done
    return 1
}

_smoke_stop_all() {
    if [[ "$_SMOKE_CLEANED" -eq 1 ]]; then
        return
    fi
    _SMOKE_CLEANED=1
    set +e

    # Signal both groups before waiting for either one, so launch teardown is
    # never deferred behind validator teardown.
    _smoke_signal_groups TERM
    for ((attempt = 0; attempt < 10; attempt++)); do
        _smoke_groups_alive || break
        sleep 0.1
    done
    if _smoke_groups_alive; then
        _smoke_signal_groups KILL
    fi

    if [[ -n "$_SMOKE_VALIDATOR_PID" ]]; then
        wait "$_SMOKE_VALIDATOR_PID" 2>/dev/null
    fi
    if [[ -n "$_SMOKE_LAUNCH_PID" ]]; then
        wait "$_SMOKE_LAUNCH_PID" 2>/dev/null
    fi
}

_smoke_on_signal() {
    local status="$1"
    trap - INT TERM HUP
    _smoke_stop_all
    exit "$status"
}

_smoke_on_exit() {
    local status="$1"
    trap - EXIT INT TERM HUP
    _smoke_stop_all
    exit "$status"
}

_smoke_start_group() {
    local output_variable="$1"
    shift

    setsid "$@" &
    local pid=$!
    printf -v "$output_variable" '%s' "$pid"

    local pgid
    pgid="$(ps -o pgid= -p "$pid" | tr -d '[:space:]')"
    if [[ "$pgid" != "$pid" ]]; then
        echo "error: child did not enter its own process group: $*" >&2
        return 1
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "error: child exited unexpectedly: $*" >&2
        return 1
    fi
}

run_smoke_harness() {
    if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
        echo "error: source a ROS 2 Jazzy environment before running this smoke test" >&2
        exit 2
    fi

    local setup="$SMOKE_WORKSPACE/install/setup.bash"
    if [[ ! -f "$setup" ]]; then
        echo "error: missing workspace setup: $setup" >&2
        exit 2
    fi

    # Colcon and ament setup files read COLCON_TRACE and
    # AMENT_TRACE_SETUP_FILES without a default, so `set -u` must be off while
    # sourcing. Strict mode is restored immediately afterwards.
    set +eu
    # shellcheck disable=SC1090
    source "$setup"
    set -euo pipefail

    trap '_smoke_on_exit $?' EXIT
    trap '_smoke_on_signal 130' INT
    trap '_smoke_on_signal 143' TERM
    trap '_smoke_on_signal 129' HUP

    _smoke_start_group _SMOKE_LAUNCH_PID "${SMOKE_LAUNCH_COMMAND[@]}"
    _smoke_start_group _SMOKE_VALIDATOR_PID \
        timeout --signal=TERM --kill-after=5s 55s \
        "${SMOKE_VALIDATOR_COMMAND[@]}"

    local validator_status
    set +e
    wait "$_SMOKE_VALIDATOR_PID"
    validator_status=$?
    set -e
    if [[ "$validator_status" -ne 0 ]]; then
        echo "error: $SMOKE_NAME validator exited with status $validator_status" >&2
        exit "$validator_status"
    fi

    if ! kill -0 "$_SMOKE_LAUNCH_PID" 2>/dev/null; then
        echo "error: $SMOKE_NAME launch exited unexpectedly" >&2
        exit 1
    fi
    echo "$SMOKE_NAME smoke test passed"
}
