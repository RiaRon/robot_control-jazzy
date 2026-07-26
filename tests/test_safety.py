import numpy as np
import pytest

from robot_control.safety import CommandGate, SafetyError


def test_publish_is_blocked_without_execute():
    gate = CommandGate(
        execute=False,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([2.0]),
        command_period_sec=0.1,
    )
    with pytest.raises(SafetyError, match="--execute"):
        gate.authorize(np.array([0.0]), now_sec=0.0)


def test_command_gate_enforces_position_and_velocity_and_watchdog():
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([2.0]),
        command_period_sec=0.1,
        watchdog_sec=0.25,
    )
    assert gate.authorize(np.array([0.0]), now_sec=0.0).tolist() == [0.0]
    with pytest.raises(SafetyError, match="velocity"):
        gate.authorize(np.array([0.5]), now_sec=0.1)
    with pytest.raises(SafetyError, match="position"):
        gate.authorize(np.array([1.1]), now_sec=0.2)
    with pytest.raises(SafetyError, match="watchdog"):
        gate.authorize(np.array([0.0]), now_sec=0.3)


def _trajectory_gate(**overrides) -> CommandGate:
    settings = dict(
        execute=True,
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
        velocity=np.array([1.0, 1.0]),
        command_period_sec=0.1,
    )
    settings.update(overrides)
    return CommandGate(**settings)


def test_authorize_trajectory_accepts_every_in_bounds_waypoint():
    gate = _trajectory_gate()

    points = [np.array([0.0, 0.0]), np.array([0.1, -0.1]), np.array([0.2, -0.2])]
    authorized = gate.authorize_trajectory(points, start_time_sec=0.0, period_sec=0.2)

    assert [point.tolist() for point in authorized] == [
        [0.0, 0.0],
        [0.1, -0.1],
        [0.2, -0.2],
    ]
    # The gate now stands at the final waypoint, at the time it is reached:
    # three waypoints one 0.2 s period apart, so 0.6 s after dispatch.
    assert gate.hold_pose().tolist() == [0.2, -0.2]
    assert gate.authorize(np.array([0.25, -0.25]), now_sec=0.65).tolist() == [0.25, -0.25]
    with pytest.raises(SafetyError, match="watchdog"):
        gate.authorize(np.array([0.25, -0.25]), now_sec=1.5)


def test_authorize_trajectory_rejection_leaves_gate_state_untouched():
    gate = _trajectory_gate()
    gate.authorize(np.array([0.0, 0.0]), now_sec=0.0)

    points = [np.array([0.05, 0.0]), np.array([0.1, 0.0]), np.array([1.5, 0.0])]
    with pytest.raises(SafetyError, match="position limit exceeded at waypoint 2"):
        gate.authorize_trajectory(points, start_time_sec=0.0, period_sec=0.2)

    # No prefix was committed, so the pre-trajectory pose is still the reference.
    assert gate.hold_pose().tolist() == [0.0, 0.0]
    assert gate.authorize(np.array([0.05, 0.0]), now_sec=0.05).tolist() == [0.05, 0.0]


def test_authorize_trajectory_budgets_the_first_waypoint_over_one_period():
    """Dispatching immediately after reading the current pose must be allowed.

    That is exactly what the pose CLI does: seed the gate with the measured
    state, then hand over a trajectory starting now. Charging the first
    waypoint against the zero-length gap since the seed would leave it a budget
    of one command period and reject every real move.
    """
    gate = _trajectory_gate()
    gate.authorize(np.array([0.0, 0.0]), now_sec=0.0)

    authorized = gate.authorize_trajectory(
        [np.array([0.4, 0.0])], start_time_sec=0.0, period_sec=0.5
    )

    assert authorized[0].tolist() == [0.4, 0.0]
    # The budget is still one period, not unlimited.
    with pytest.raises(SafetyError, match="velocity limit exceeded at waypoint 0"):
        gate.authorize_trajectory(
            [np.array([0.9, 0.0])], start_time_sec=0.5, period_sec=0.1
        )


def test_authorize_trajectory_rejects_velocity_between_adjacent_waypoints():
    gate = _trajectory_gate()

    points = [np.array([0.0, 0.0]), np.array([0.5, 0.0])]
    with pytest.raises(SafetyError, match="velocity limit exceeded at waypoint 1"):
        gate.authorize_trajectory(points, start_time_sec=0.0, period_sec=0.1)


def test_authorize_trajectory_rejects_an_empty_trajectory():
    gate = _trajectory_gate()

    with pytest.raises(SafetyError, match="no waypoints"):
        gate.authorize_trajectory([], start_time_sec=0.0, period_sec=0.2)


def test_authorize_trajectory_rejects_a_non_positive_period():
    gate = _trajectory_gate()

    with pytest.raises(SafetyError, match="period"):
        gate.authorize_trajectory(
            [np.array([0.0, 0.0])], start_time_sec=0.0, period_sec=0.0
        )


def test_authorize_trajectory_keeps_execute_and_estop_refusals():
    blocked = _trajectory_gate(execute=False)
    with pytest.raises(SafetyError, match="--execute"):
        blocked.authorize_trajectory(
            [np.array([0.0, 0.0])], start_time_sec=0.0, period_sec=0.2
        )

    gate = _trajectory_gate()
    gate.estop()
    with pytest.raises(SafetyError, match="E-stop"):
        gate.authorize_trajectory(
            [np.array([0.0, 0.0])], start_time_sec=0.0, period_sec=0.2
        )


def test_authorize_trajectory_applies_the_watchdog_only_to_the_leading_gap():
    gate = _trajectory_gate(watchdog_sec=0.25)
    gate.authorize(np.array([0.0, 0.0]), now_sec=0.0)

    # Waypoints spaced beyond the watchdog are a planned motion handed to the
    # controller as one goal, not a stalled command stream.
    points = [np.array([0.0, 0.0]), np.array([0.5, 0.0])]
    authorized = gate.authorize_trajectory(points, start_time_sec=0.1, period_sec=1.0)
    assert len(authorized) == 2

    # An idle gap before the trajectory is still a stalled stream.
    stale = _trajectory_gate(watchdog_sec=0.25)
    stale.authorize(np.array([0.0, 0.0]), now_sec=0.0)
    with pytest.raises(SafetyError, match="watchdog"):
        stale.authorize_trajectory(points, start_time_sec=5.0, period_sec=1.0)


def test_estop_returns_last_safe_pose_hold():
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([10.0]),
        command_period_sec=0.1,
    )
    gate.authorize(np.array([0.25]), now_sec=0.0)
    gate.estop()
    assert gate.hold_pose().tolist() == [0.25]
    with pytest.raises(SafetyError, match="E-stop"):
        gate.authorize(np.array([0.0]), now_sec=0.1)
