"""`pose torque`: pushing a joint with a torque we chose, not one the model gave.

The stub arm is a linear spring with dry friction, so the sweep this writes
should be exactly what the staircase fitter in Task 7 can invert. The point of
testing at this level is the plumbing — one joint driven at a time, the arm
released at the end, the artifact shaped right — not the arithmetic, which
tests/test_excitation.py covers.
"""

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.profile import load_builtin_profile

GROUP = "openarm_right_arm"
KP = np.array([67.0, 64.0, 90.0, 70.0, 15.0, 11.0, 13.0])


def _flat_chain():
    """A seven-joint chain about +y, each link a metre out along +x.

    Mirrors the stub chain other CLI tests build for `_gravity_chain` (see
    tests/test_cli.py and tests/test_cli_collect.py) — the shape
    `chain.gravity_torque` needs, not a real robot description.
    """
    from robot_control import kinematics

    joints = tuple(
        kinematics.Revolute(
            f"j{index}",
            np.array([0.0, 1.0, 0.0]),
            np.zeros(3) if index == 0 else np.array([1e-6, 0.0, 0.0]),
            np.eye(3),
            f"l{index}",
        )
        for index in range(7)
    )
    links = tuple(
        kinematics.Link(f"l{index}", 0.05, np.array([1.0, 0.0, 0.0]))
        for index in range(7)
    )
    return kinematics.Chain(joints, links)


class SpringArm:
    """Deflects by torque/kp, and records every effort it was given."""

    def __init__(self):
        # r_aj_2's lower stop is -0.174533 rad and r_aj_4's is 0.0 (see the
        # same fact noted in tests/test_cli_collect_track.py): either one at
        # canonical zero has less than MARGIN_RAD (0.20 rad) of room, so
        # probe_torque refuses the position before it ever looks at torque.
        # 0.5 rad clears both stops the same way that file's stub does.
        self.joints = np.array([0.0, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.published = []
        self.effort = np.zeros(7)

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_robot_description(self):
        return "<robot name='stub'/>"

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def send_effort(self, effort):
        self.effort = np.asarray(effort, dtype=float).copy()
        self.published.append(self.effort)

    def read_tracking_error(self):
        return self.effort / KP


@pytest.fixture
def arm(monkeypatch):
    from robot_control import ros_adapter

    stub = SpringArm()
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain", lambda *a: _flat_chain()
    )
    return stub


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def test_it_drives_one_joint_at_a_time(arm, tmp_path):
    code = main(["pose", "torque", "--group", GROUP, "--execute",
                 "--steps", "3", "--hold-sec", "0.01",
                 "--output", str(tmp_path / "t.json")])

    assert code == 0
    for effort in arm.published:
        assert np.count_nonzero(effort) <= 1, (
            "only the joint under test may be pushed; the rest are held by "
            "the position loop"
        )


def test_it_releases_the_torque_when_it_finishes(arm, tmp_path):
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "3",
          "--hold-sec", "0.01", "--output", str(tmp_path / "t.json")])

    assert arm.published[-1].tolist() == [0.0] * 7


def test_the_sweep_it_writes_records_the_torque_and_not_a_scale(arm, tmp_path, profile):
    from robot_control.artifacts import read_sweep

    output = tmp_path / "t.json"
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "3",
          "--hold-sec", "0.01", "--joint", "r_aj_5", "--output", str(output)])

    sweep = read_sweep(output, profile)
    assert sweep.rounds == 2 * 3 - 1
    assert np.count_nonzero(sweep.scales) == 0
    assert np.ptp(sweep.applied_torque[:, 4]) > 0


def test_a_dry_run_publishes_nothing(arm, tmp_path):
    assert main(["pose", "torque", "--group", GROUP, "--steps", "3"]) == 0
    assert arm.published == []
