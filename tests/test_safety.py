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


def _streaming_gate(**overrides) -> CommandGate:
    """A two-joint gate for the streaming tests."""
    settings = dict(
        execute=True,
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
        velocity=np.array([2.0, 2.0]),
        effort=np.array([10.0, 10.0]),
        command_period_sec=0.01,
    )
    settings.update(overrides)
    return CommandGate(**settings)


def test_follow_clamps_a_fast_target_instead_of_refusing_it():
    """Servoing cannot refuse: an operator flicking the marker faster than the
    arm can move would abort the session on the first quick drag. So follow()
    steps as far as the velocity limit allows and says that it did."""
    gate = _streaming_gate()
    gate.follow(np.array([0.0, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01)

    command, limited = gate.follow(
        np.array([1.0, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01
    )

    # 2 rad/s for 10 ms is 0.02 rad, not the whole 1.0 asked for.
    np.testing.assert_allclose(command, [0.02, 0.0])
    assert limited is not None and "velocity" in limited


def test_follow_passes_a_reachable_target_through_unchanged():
    gate = _streaming_gate()
    gate.follow(np.array([0.0, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01)

    command, limited = gate.follow(
        np.array([0.01, -0.01]), np.array([0.0, 0.0]), elapsed_sec=0.01
    )

    np.testing.assert_allclose(command, [0.01, -0.01])
    assert limited is None


def test_follow_clamps_into_the_position_limits():
    """Stopping at the limit is what an operator dragging past it should see;
    refusing would end the session mid-drag instead."""
    gate = _streaming_gate()

    command, limited = gate.follow(
        np.array([5.0, 0.0]), np.array([0.99, 0.0]), elapsed_sec=1.0
    )

    np.testing.assert_allclose(command, [1.0, 0.0])
    assert limited is not None and "position" in limited


def test_follow_still_refuses_without_execute_and_after_estop():
    gate = _streaming_gate(execute=False)
    with pytest.raises(SafetyError, match="--execute"):
        gate.follow(np.array([0.0, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01)

    gate = _streaming_gate()
    gate.estop()
    with pytest.raises(SafetyError, match="E-stop"):
        gate.follow(np.array([0.0, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01)


def test_follow_rejects_a_target_that_is_not_a_number():
    gate = _streaming_gate()

    with pytest.raises(SafetyError, match="invalid"):
        gate.follow(np.array([np.nan, 0.0]), np.array([0.0, 0.0]), elapsed_sec=0.01)


def test_follow_measures_velocity_from_where_the_arm_is():
    """The step has to be bounded against the measured pose, not the last
    command. A drooping arm sits behind its command, so budgeting from the
    command would let the real step exceed the limit."""
    gate = _streaming_gate()

    command, _limited = gate.follow(
        np.array([1.0, 0.0]), np.array([0.5, 0.0]), elapsed_sec=0.01
    )

    np.testing.assert_allclose(command, [0.52, 0.0])


def test_authorize_effort_bounds_torque_by_the_profile():
    """Effort is feedforward torque: too much accelerates the arm rather than
    misposing it, so it is refused rather than clamped."""
    gate = _streaming_gate()

    np.testing.assert_allclose(
        gate.authorize_effort(np.array([9.0, -9.0])), [9.0, -9.0]
    )
    with pytest.raises(SafetyError, match="effort limit exceeded"):
        gate.authorize_effort(np.array([11.0, 0.0]))
    with pytest.raises(SafetyError, match="effort limit exceeded"):
        gate.authorize_effort(np.array([0.0, -11.0]))


def test_authorize_effort_refuses_without_execute_and_after_estop():
    with pytest.raises(SafetyError, match="--execute"):
        _streaming_gate(execute=False).authorize_effort(np.array([1.0, 1.0]))

    gate = _streaming_gate()
    gate.estop()
    with pytest.raises(SafetyError, match="E-stop"):
        gate.authorize_effort(np.array([1.0, 1.0]))


def test_effort_limits_are_optional_so_existing_gates_keep_working():
    """Position-only callers build a gate with no effort bound at all."""
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([2.0]),
        command_period_sec=0.01,
    )

    assert gate.authorize(np.array([0.0]), now_sec=0.0).tolist() == [0.0]
    with pytest.raises(SafetyError, match="no effort limit"):
        gate.authorize_effort(np.array([1.0]))
