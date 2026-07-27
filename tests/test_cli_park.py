"""`pose ready` and `pose rest`: the slow setup move the operator asked for.

Bringing the arms up is a fixed ritual — from wherever the arm is to the
initial working state (elbow raised, everything else at zero) — and it was
being typed as a seven-value `pose joints` each time. The speed matters more
than the destination: these run at PARK_SPEED_RAD_PER_SEC so the operator can
reach the E-stop mid-move, which a default-duration move did not allow.

The seed tests live here too because parking exposed the problem: an arm at
rest sits ON its lower stop, and impedance droop reads a hair past it. The
measured pose is evidence, not a command — refusing to start a move because
the arm is 3 mrad outside a bound it is resting against would make the rest
pose unrecoverable.
"""

import sys

import numpy as np
import pytest

from robot_control.cli import (
    ELBOW_INDEX,
    PARK_SPEED_RAD_PER_SEC,
    READY_ELBOW_RAD,
    READY_WRIST_RAD,
    SEED_SLACK_RAD,
    WRIST_INDEX,
    main,
)

ARM_JOINTS = 7


@pytest.fixture
def no_ros(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)


class ParkableArm:
    """Records every trajectory; its state follows the last waypoint."""

    def __init__(self, state):
        self.joints = np.asarray(state, dtype=float)
        self.trajectories = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def send_trajectory(self, points, period_sec):
        self.trajectories.append(
            [np.asarray(point, dtype=float).copy() for point in points]
        )
        self.joints = np.asarray(points[-1], dtype=float).copy()
        self.period_sec = period_sec


@pytest.fixture
def arm(monkeypatch):
    from robot_control import ros_adapter

    stub = ParkableArm(np.zeros(ARM_JOINTS))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    return stub


def test_ready_dry_run_names_both_arms_offline(no_ros, capsys):
    assert main(["pose", "ready"]) == 0

    output = capsys.readouterr().out
    assert "openarm_left_arm" in output and "openarm_right_arm" in output
    assert f"{READY_ELBOW_RAD:g}" in output
    assert "DRY RUN" in output


def test_ready_lifts_the_elbow_before_touching_the_wrist(arm, capsys):
    """A tool resting on the table blocks the wrist's sweep entirely, so any
    wrist target commanded while the arm lies down is a stall, not a move.
    That is what burned left j7: at rest the gripper pins j7 near its limit,
    and driving it to zero fought the table until the rotor overheated.

    Phase 1 therefore raises ONLY the elbow — every other joint holds where it
    measured, including the pinned wrist. Phase 2, airborne, settles the rest.
    """
    pinned = -1.3
    arm.joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, pinned])

    assert main(["pose", "ready", "--group", "openarm_left_arm", "--execute"]) == 0

    lift, settle = arm.trajectories
    for point in lift:
        assert point[WRIST_INDEX] == pytest.approx(pinned), (
            "the wrist may not be commanded while the tool can still touch "
            "the table"
        )
    assert lift[-1][ELBOW_INDEX] == pytest.approx(READY_ELBOW_RAD)

    final = settle[-1]
    expected = np.zeros(ARM_JOINTS)
    expected[ELBOW_INDEX] = READY_ELBOW_RAD
    expected[WRIST_INDEX] = -READY_WRIST_RAD  # mirrored: the left lift is negative
    assert np.allclose(final, expected)

    # Slow enough to react to, in both phases.
    for trajectory in (lift, settle):
        steps = np.diff(np.asarray(trajectory), axis=0)
        assert np.abs(steps).max() <= PARK_SPEED_RAD_PER_SEC * arm.period_sec + 1e-9


def test_ready_wrist_sign_mirrors_between_the_arms(arm, capsys):
    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 0

    final = arm.trajectories[-1][-1]
    assert final[WRIST_INDEX] == pytest.approx(READY_WRIST_RAD)


def test_rest_keeps_the_wrist_lifted_while_the_elbow_comes_down(arm, capsys):
    """The reverse ritual: settle the wrist first, then lower. The wrist stays
    at its lifted angle to the end, so the tool meets the table yielding in
    the direction the table pushes — never commanded straight against it."""
    arm.joints = np.array([0.0, 0.0, 0.0, READY_ELBOW_RAD, 0.0, 0.0, 0.3])

    assert main(["pose", "rest", "--group", "openarm_right_arm", "--execute"]) == 0

    settle, lower = arm.trajectories
    # Phase 1 holds the elbow up while everything else reaches its rest value.
    for point in settle:
        assert point[ELBOW_INDEX] == pytest.approx(READY_ELBOW_RAD)
    assert settle[-1][WRIST_INDEX] == pytest.approx(READY_WRIST_RAD)

    final = lower[-1]
    expected = np.zeros(ARM_JOINTS)
    expected[WRIST_INDEX] = READY_WRIST_RAD
    assert np.allclose(final, expected)


def test_ready_covers_both_arms_by_default(monkeypatch, capsys):
    from robot_control import ros_adapter

    # One fresh parked arm per adapter session, so the second arm does not
    # inherit the first one's final state through a shared stub.
    arms = []

    def fresh(profile, name, execute):
        stub = ParkableArm(np.zeros(ARM_JOINTS))
        arms.append((name, stub))
        return stub

    monkeypatch.setattr(ros_adapter, "RosAdapter", fresh)

    assert main(["pose", "ready", "--execute"]) == 0

    assert [name for name, _ in arms] == ["openarm_left_arm", "openarm_right_arm"]
    assert all(len(stub.trajectories) == 2 for _, stub in arms)  # two phases each


def test_ready_refuses_a_group_that_is_not_an_arm(no_ros, capsys):
    assert main(["pose", "ready", "--group", "tesollo_curl"]) == 2
    assert "arm" in capsys.readouterr().out


def test_an_arm_resting_a_hair_past_its_stop_can_still_be_parked(arm, capsys):
    """Impedance droop reads the elbow slightly below its hard stop at 0.

    This was `refused: position limit exceeded` on the real robot from the
    all-zeros rest pose: the target was fine, the *measured seed* was not.
    """
    arm.joints[ELBOW_INDEX] = -0.003

    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 0

    # The ramp starts from the clamped pose, so every waypoint is in bounds.
    first = arm.trajectories[-1][0]
    assert first[ELBOW_INDEX] >= 0.0


def test_an_arm_grossly_outside_the_profile_is_still_refused_by_name(arm, capsys):
    arm.joints[ELBOW_INDEX] = -(SEED_SLACK_RAD * 4)

    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 3

    output = capsys.readouterr().out
    assert "r_aj_4" in output


def test_pose_joints_also_starts_from_the_clamped_seed(arm, capsys):
    """The same droop broke plain `pose joints` from the rest pose."""
    arm.joints[ELBOW_INDEX] = -0.003
    values = "0,0,0,0.8,0,0,0"

    code = main(
        ["pose", "joints", "--group", "openarm_right_arm",
         "--values", values, "--execute"]
    )

    assert code == 0
    assert arm.trajectories[-1][-1][ELBOW_INDEX] == pytest.approx(0.8)
