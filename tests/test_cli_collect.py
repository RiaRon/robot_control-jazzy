"""`robotctl r2s identify --collect`: the arm drives its own pose set.

The largest new hazard in this work, so the tests are mostly about what happens
*before* anything moves.
"""

import json

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"
KP = np.linspace(8.0, 28.0, 7)
ALPHA = np.linspace(0.9, 1.15, 7)
OFFSET = 0.0015


def _stub_chain(mass=0.05):
    """A seven-joint planar chain about +y, each link a metre out along +x."""
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
        kinematics.Link(f"l{index}", mass, np.array([1.0, 0.0, 0.0]))
        for index in range(7)
    )
    return kinematics.Chain(joints, links)


class IdentifiableArm:
    """A stub arm that droops exactly as the static model says it should.

    Its true load is ALPHA times the modelled torque, so a fit that recovers
    both KP and ALPHA has separated the stiffness from the model's own error —
    which is the whole point of sweeping at more than one pose.
    """

    def __init__(self, chain):
        self.chain = chain
        self.joints = np.zeros(7)
        self.applied = np.zeros(7)
        self.published = []
        self.moves = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_robot_description(self):
        return "<robot name='stub'/>"  # unused: the chain is injected

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def send_trajectory(self, points, period_sec):
        self.moves.append(np.asarray(points[-1], dtype=float).copy())
        self.joints = np.asarray(points[-1], dtype=float).copy()

    def send_effort(self, effort):
        self.applied = np.asarray(effort, dtype=float).copy()
        self.published.append(self.applied)

    def read_tracking_error(self):
        load = ALPHA * self.chain.gravity_torque(self.joints)
        return (load - self.applied) / KP + OFFSET


@pytest.fixture
def arm(monkeypatch):
    from robot_control import ros_adapter

    chain = _stub_chain()
    stub = IdentifiableArm(chain)
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    monkeypatch.setattr("robot_control.cli._gravity_chain", lambda *a: chain)
    return stub


def _argv(tmp_path, *extra):
    # --hold-sec is the only one that costs wall-clock: the stub's moves return
    # at once, so --duration only sets the velocity budget each leg is checked
    # against, and a realistic 3 s is what the arm would actually be given.
    return [
        "r2s", "identify", "--collect", "--group", GROUP,
        "--sweep-dir", str(tmp_path / "sweeps"),
        "--output", str(tmp_path / "static.json"),
        "--hold-sec", "0.01", "--duration", "3.0",
        *extra,
    ]


def test_collect_identifies_the_arm_it_drove(arm, tmp_path, capsys):
    code = main(_argv(tmp_path, "--execute", "--poses", "4"))

    assert code == 0, capsys.readouterr().out
    # One file per pose, and the arm actually visited each.
    written = sorted((tmp_path / "sweeps").glob("*.json"))
    assert len(written) == 4
    assert len(arm.moves) == 4
    payload = json.loads((tmp_path / "static.json").read_text())
    np.testing.assert_allclose(payload["stiffness_nm_per_rad"], KP, rtol=0.02)
    np.testing.assert_allclose(payload["torque_scale"], ALPHA, rtol=0.02)
    np.testing.assert_allclose(payload["offset_rad"], OFFSET, atol=1e-4)
    assert len(payload["sources"]["sweep_sha256"]) == 4


def test_collect_releases_the_torque_at_every_pose(arm, tmp_path):
    assert main(_argv(tmp_path, "--execute", "--poses", "3")) == 0

    np.testing.assert_allclose(arm.published[-1], np.zeros(7))


def test_collect_without_execute_reviews_the_itinerary_and_moves_nothing(
    arm, tmp_path, capsys
):
    """The printed poses are the review: nothing here checks self-collision."""
    code = main(_argv(tmp_path, "--poses", "4"))

    assert code == 0
    out = capsys.readouterr().out
    assert "DRY RUN" in out
    assert "collision" in out
    assert arm.moves == []
    assert arm.published == []
    assert not (tmp_path / "sweeps").exists()


def test_the_reviewed_itinerary_is_the_one_that_runs(arm, tmp_path, capsys):
    """Same seed, same poses, or a dry run reviews something else."""
    main(_argv(tmp_path, "--poses", "3", "--seed", "5"))
    reviewed = capsys.readouterr().out

    main(_argv(tmp_path, "--poses", "3", "--seed", "5", "--execute"))
    executed = capsys.readouterr().out

    poses = [line for line in reviewed.splitlines() if line.startswith("  pose ")]
    assert poses
    for line in poses:
        assert line in executed


def test_collect_refuses_a_pose_outside_the_limits_before_moving(
    arm, tmp_path, monkeypatch, capsys
):
    from robot_control.identification import PoseSet

    def beyond(*_args, **_kwargs):
        poses = np.zeros((2, 7))
        poses[1, 1] = 9.0  # r_aj_2 stops at 2.0 rad
        return PoseSet(poses=poses, scales=(0.0, 1.0), condition=np.full(7, 3.0))

    monkeypatch.setattr("robot_control.cli.design_pose_set", beyond)

    code = main(_argv(tmp_path, "--execute", "--poses", "2"))

    assert code == 3
    assert "position limit exceeded" in capsys.readouterr().out
    assert arm.moves == [], "a pose set is validated whole, before the first move"
    assert arm.published == []


def test_collect_refuses_a_leg_the_arm_cannot_travel_in_time(
    arm, tmp_path, monkeypatch, capsys
):
    """A run that stopped mid-itinerary would leave the arm somewhere unplanned."""
    from robot_control.identification import PoseSet

    def far(*_args, **_kwargs):
        poses = np.array([np.full(7, -1.5), np.full(7, 1.5)])
        return PoseSet(poses=poses, scales=(0.0, 1.0), condition=np.full(7, 3.0))

    monkeypatch.setattr("robot_control.cli.design_pose_set", far)

    # 3.0 rad in 0.01 s is 300 rad/s against the profile's 2.0.
    code = main(_argv(tmp_path, "--execute", "--poses", "2", "--duration", "0.01"))

    assert code == 3
    assert "velocity limit exceeded" in capsys.readouterr().out
    assert arm.moves == []


def test_collect_refuses_an_under_conditioned_set_before_moving(
    arm, tmp_path, monkeypatch, capsys
):
    from robot_control.identification import PoseSet

    def flat(*_args, **_kwargs):
        return PoseSet(
            poses=np.zeros((3, 7)),
            scales=(0.0, 0.5, 1.0),
            condition=np.array([3.0, 3.0, 1e9, 3.0, 3.0, 3.0, 3.0]),
        )

    monkeypatch.setattr("robot_control.cli.design_pose_set", flat)

    code = main(_argv(tmp_path, "--execute", "--poses", "3"))

    assert code == 3
    out = capsys.readouterr().out
    assert "r_aj_3" in out
    assert arm.moves == []


def test_collect_needs_a_group(tmp_path, capsys):
    code = main(
        ["r2s", "identify", "--collect", "--output", str(tmp_path / "static.json")]
    )

    assert code == 2
    assert "--group" in capsys.readouterr().out


def test_collect_execute_needs_somewhere_to_write(arm, tmp_path, capsys):
    code = main(
        ["r2s", "identify", "--collect", "--group", GROUP, "--execute",
         "--output", str(tmp_path / "static.json")]
    )

    assert code == 2
    assert "--sweep-dir" in capsys.readouterr().out
    assert arm.moves == []


def test_collect_refuses_a_group_with_no_effort_controller(tmp_path, capsys):
    code = main(
        ["r2s", "identify", "--collect", "--group", "tesollo_curl",
         "--output", str(tmp_path / "static.json")]
    )

    assert code == 2
    assert "effort_controller" in capsys.readouterr().out


def test_collected_sweeps_can_be_refitted_from_disk(arm, tmp_path, capsys):
    """The files are the record: identify over them alone gives the same answer."""
    assert main(_argv(tmp_path, "--execute", "--poses", "4")) == 0
    first = json.loads((tmp_path / "static.json").read_text())
    capsys.readouterr()

    argv = ["r2s", "identify", "--output", str(tmp_path / "again.json")]
    for path in sorted((tmp_path / "sweeps").glob("*.json")):
        argv += ["--sweep", str(path)]

    assert main(argv) == 0
    again = json.loads((tmp_path / "again.json").read_text())
    assert again["stiffness_nm_per_rad"] == first["stiffness_nm_per_rad"]


def test_collect_can_add_poses_to_sweeps_already_on_disk(arm, tmp_path, capsys):
    """A joint stuck at every designed pose is fixed by adding a pose by hand."""
    assert main(_argv(tmp_path, "--execute", "--poses", "2")) == 0
    earlier = sorted((tmp_path / "sweeps").glob("*.json"))
    capsys.readouterr()

    code = main(
        _argv(tmp_path / "second", "--execute", "--poses", "2", "--seed", "11")
        + sum([["--sweep", str(path)] for path in earlier], [])
    )

    assert code == 0
    payload = json.loads((tmp_path / "second" / "static.json").read_text())
    assert len(payload["sources"]["sweep_sha256"]) == 4
