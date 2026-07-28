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
                 "--steps", "4", "--hold-sec", "0.01",
                 "--output", str(tmp_path / "t.json")])

    assert code == 0
    for effort in arm.published:
        assert np.count_nonzero(effort) <= 1, (
            "only the joint under test may be pushed; the rest are held by "
            "the position loop"
        )


def test_it_releases_the_torque_when_it_finishes(arm, tmp_path):
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "4",
          "--hold-sec", "0.01", "--output", str(tmp_path / "t.json")])

    assert arm.published[-1].tolist() == [0.0] * 7


def test_the_sweep_it_writes_records_the_torque_and_not_a_scale(arm, tmp_path, profile):
    from robot_control.artifacts import read_sweep

    output = tmp_path / "t.json"
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "4",
          "--hold-sec", "0.01", "--joint", "r_aj_5", "--output", str(output)])

    sweep = read_sweep(output, profile)
    # One fewer than the staircase's 2*steps - 1 torques: the joint arrives at
    # the first one from the probe, travelling the wrong way for the branch it
    # would be read on, so it is held but not recorded.
    assert sweep.rounds == 2 * 4 - 2
    assert np.count_nonzero(sweep.scales) == 0
    assert np.ptp(sweep.applied_torque[:, 4]) > 0


def test_it_reports_the_seed_each_joint_needed(arm, tmp_path, capsys):
    """The smallest seed that moved a joint is the first measurement of that
    joint's dry friction the run produces — it stood still at half of it — so
    an operator watching should see it per joint, and see the ratio between
    joints, rather than have to work it out afterwards. This stub arm has no
    friction at all, so every joint moves under the first seed.
    """
    main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "4",
          "--hold-sec", "0.01", "--output", str(tmp_path / "t.json")])

    out = capsys.readouterr().out
    assert out.count("N.m seed") == len(KP)
    assert "r_aj_5: moved under a 0.05 N.m seed" in out


def test_a_staircase_too_short_to_fit_is_refused(arm, tmp_path, capsys):
    """--steps 3 writes a sweep that cannot be fitted: its first recorded round
    carries zero torque, which leaves the rising branch one round short. The
    run is refused at the desk rather than discovered after a trip to the
    robot, on the fit that comes hours later.
    """
    code = main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "3",
                 "--hold-sec", "0.01", "--output", str(tmp_path / "t.json")])

    assert code == 2
    assert "--steps" in capsys.readouterr().out
    assert arm.published == []


@pytest.mark.parametrize("noise", ["0", "-1"])
def test_a_non_positive_noise_is_refused(arm, tmp_path, capsys, noise):
    """`--noise 0` puts the motion test back on the 1e-6 divide-by-zero guard,
    which is exactly the defect the noise floor was added to fix: encoder
    dither reads as motion, the escalation returns at the first seed, and a
    torque extrapolated from noise is published before any deflection has been
    measured. On the jittered shoulder that put 7.19 N.m on a joint whose true
    answer is 3.485.

    `identify --noise 0` is harmless, so an operator can carry a benign habit
    onto the one command that publishes torque. This is the command where it
    has to be refused.
    """
    code = main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "4",
                 "--hold-sec", "0.01", "--noise", noise,
                 "--output", str(tmp_path / "t.json")])

    assert code == 2
    assert "--noise" in capsys.readouterr().out
    assert arm.published == []


def test_a_joint_named_twice_is_refused(arm, tmp_path, capsys):
    """A joint's rounds are read back as the block between its first and last
    round carrying torque, so driving one joint twice swallows every joint
    driven in between — the same fit-poisoning that reading a whole file as
    one staircase caused, through a different door.
    """
    code = main(["pose", "torque", "--group", GROUP, "--execute", "--steps", "4",
                 "--hold-sec", "0.01", "--joint", "r_aj_5", "--joint", "r_aj_5",
                 "--output", str(tmp_path / "t.json")])

    assert code == 2
    assert "r_aj_5" in capsys.readouterr().out
    assert arm.published == []


def test_a_dry_run_publishes_nothing(arm, tmp_path):
    assert main(["pose", "torque", "--group", GROUP, "--steps", "4"]) == 0
    assert arm.published == []
