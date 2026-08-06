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
        max_lead=np.array([0.5, 0.5]),
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

def test_follow_records_lower_position_limit_direction():
    """최솟값 아래로 가려는 관절이 position_lower로 기록되는지 확인한다."""

    # 두 관절의 이름과 허용 범위를 가진 가짜 안전 게이트를 만든다.
    gate = _streaming_gate(names=("joint_a", "joint_b"))

    # joint_a는 현재 -0.99 rad에 있고, -5.0 rad로 가려고 한다.
    # 허용 최솟값은 -1.0 rad이므로 아래쪽 위치 제한에 걸려야 한다.
    command, limited = gate.follow(
        target=np.array([-5.0, 0.0]),
        measured=np.array([-0.99, 0.0]),
        elapsed_sec=1.0,
    )

    # 실제 명령은 안전한 최솟값 -1.0 rad에서 멈춰야 한다.
    np.testing.assert_allclose(command, [-1.0, 0.0])

    # 기존 종합 결과에도 position 제한이 표시되어야 한다.
    assert limited is not None and "position" in limited

    # 새 진단 정보에는 joint_a가 아래쪽 한계에 걸렸다고 기록되어야 한다.
    assert gate.last_follow_limits["position_lower"] == ("joint_a",)

    # 위쪽 한계에는 걸리지 않았으므로 position_upper가 없어야 한다.
    assert "position_upper" not in gate.last_follow_limits

def test_follow_records_the_joints_limited_by_each_safety_rule():
    """follow()가 제한 종류뿐 아니라 해당 관절 이름도 기록해야 한다."""

    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
        velocity=np.array([2.0, 2.0]),
        command_period_sec=0.01,
        max_lead=np.array([0.1, 0.1]),
        names=("joint_a", "joint_b"),
    )

    # joint_a는 속도·lead·위치 제한에 모두 걸리도록 만든다.
    # joint_b는 lead 제한에만 걸리도록 만든다.
    command, limited = gate.follow(
        target=np.array([5.0, 1.0]),
        measured=np.array([0.99, 0.0]),
        elapsed_sec=1.0,
    )

    # 기존 반환값과 최종 안전 명령이 그대로 유지되는지 확인한다.
    np.testing.assert_allclose(command, [1.0, 0.1])
    assert limited == "velocity and lead and position limit"

    # 새로 추가한 관절별 제한 정보가 정확한지 확인한다.
    assert gate.last_follow_limits == {
        "velocity": ("joint_a",),
        "lead": ("joint_a", "joint_b"),
        "position": ("joint_a",),
        "position_upper": ("joint_a",),
    }

    # 다음 호출에서 아무 제한도 발생하지 않으면
    # 이전 호출의 제한 정보가 남아 있지 않아야 한다.
    command, limited = gate.follow(
        target=np.array([0.99, 0.0]),
        measured=np.array([0.99, 0.0]),
        elapsed_sec=1.0,
    )

    np.testing.assert_allclose(command, [0.99, 0.0])
    assert limited is None
    assert gate.last_follow_limits == {}

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


def test_follow_rate_limits_from_the_previous_command_not_the_measured_pose():
    """The bug this pins cost a real-hardware session.

    These arms sit behind their command by the droop their impedance control
    needs to hold position. Budgeting each step from the measured pose caps the
    command at one period's travel ahead of where the arm is — which is less
    than the standing droop, so the command lands behind the equilibrium and the
    arm never advances. Fake hardware has no droop, so it tracked perfectly and
    hid this entirely.
    """
    gate = _streaming_gate(max_lead=np.array([0.5, 0.5]))
    droop = 0.03  # larger than one period's budget of 2.0 * 0.01 = 0.02

    measured = np.array([0.0, 0.0])
    for _ in range(5):
        command, _limited = gate.follow(
            np.array([1.0, 0.0]), measured, elapsed_sec=0.01
        )
        # The arm follows its command, lagging by the droop it needs to hold.
        measured = np.array([max(0.0, command[0] - droop), 0.0])

    # Five samples at 0.02 rad each: the command has to have advanced.
    assert command[0] > 0.09, f"command stalled at {command[0]}"
    assert measured[0] > 0.0, "the arm never moved"


def test_follow_bounds_how_far_the_command_may_lead_the_measured_pose():
    """A blocked arm must not let the command wind away from it.

    Rate-limiting from the previous command is what makes progress possible; on
    its own it also means a joint held still by an obstacle would accumulate
    command indefinitely, and with it the torque its stiffness produces.
    """
    gate = _streaming_gate(max_lead=np.array([0.1, 0.1]))
    stuck = np.array([0.0, 0.0])

    for _ in range(50):
        command, limited = gate.follow(np.array([1.0, 0.0]), stuck, elapsed_sec=0.01)

    assert command[0] == pytest.approx(0.1)
    assert limited is not None and "lead" in limited


def test_follow_still_bounds_the_first_sample_against_the_measured_pose():
    """With no previous command there is nothing to rate-limit from, so the
    measured pose is the only honest starting point."""
    gate = _streaming_gate(max_lead=np.array([0.5, 0.5]))

    command, limited = gate.follow(
        np.array([1.0, 0.0]), np.array([0.5, 0.0]), elapsed_sec=0.01
    )

    np.testing.assert_allclose(command, [0.52, 0.0])
    assert limited is not None and "velocity" in limited


def _named_gate() -> CommandGate:
    return CommandGate(
        execute=True,
        lower=np.array([-0.1745, 0.0]),
        upper=np.array([3.3161, 2.4435]),
        velocity=np.array([2.0, 2.0]),
        command_period_sec=0.01,
        names=["r_aj_2", "r_aj_4"],
    )


def test_a_refused_position_names_the_joint_the_value_and_the_bound():
    """What the operator needs is which joint, not that some joint.

    An arm parked outside the profile's bounds is refused at the seed, before
    any waypoint exists, so the message is the only evidence of what happened.
    Over seven joints "position limit exceeded" cannot distinguish an arm in a
    surprising pose from a profile whose limits are wrong for this robot, which
    is the whole question.
    """
    gate = _named_gate()

    with pytest.raises(SafetyError) as refusal:
        gate.authorize(np.array([-0.9, 0.8]), now_sec=0.0)

    message = str(refusal.value)
    assert "r_aj_2=-0.9000" in message
    assert "[-0.1745, +3.3161]" in message
    # The joint that was in range is not accused.
    assert "r_aj_4" not in message


def test_a_refused_position_names_every_joint_that_broke_a_bound():
    gate = _named_gate()

    with pytest.raises(SafetyError) as refusal:
        gate.authorize(np.array([-0.9, -0.5]), now_sec=0.0)

    message = str(refusal.value)
    assert "r_aj_2" in message and "r_aj_4" in message


def test_a_refused_velocity_names_the_joint_and_the_budget_it_broke():
    gate = _named_gate()
    gate.authorize(np.array([0.0, 0.5]), now_sec=0.0)

    with pytest.raises(SafetyError) as refusal:
        gate.authorize(np.array([0.0, 1.5]), now_sec=0.01)

    message = str(refusal.value)
    assert "r_aj_4" in message
    assert "1.0000 rad" in message and "0.0200" in message


def test_an_unnamed_gate_still_says_which_index_broke_the_bound():
    """names is optional; every other construction site must keep working."""
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0, -1.0]),
        upper=np.array([1.0, 1.0]),
        velocity=np.array([2.0, 2.0]),
        command_period_sec=0.1,
    )

    with pytest.raises(SafetyError, match=r"joint 1=\+2\.0000"):
        gate.authorize(np.array([0.0, 2.0]), now_sec=0.0)
