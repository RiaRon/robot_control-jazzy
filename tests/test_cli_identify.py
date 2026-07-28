"""`robotctl r2s identify`: sweeps in, a stiffness set out."""

import json

import numpy as np
import pytest

from robot_control.artifacts import write_sweep
from robot_control.cli import main
from robot_control.identification import GravitySweep
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"
SCALES = (0.0, 0.4, 0.8, 1.2)


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def _truth(width):
    return (
        np.linspace(7.5, 28.0, width),  # kp
        np.linspace(0.9, 1.15, width),  # alpha
        np.full(width, 0.0015),  # c
    )


def _write_pose(path, profile, torque_of_pose, frozen=()):
    """Write one sweep whose errors are exactly what the static model predicts."""
    joints = profile.groups[GROUP].joints
    width = len(joints)
    kp, alpha, offset = _truth(width)
    torques = np.full(width, torque_of_pose) * np.linspace(1.0, 2.0, width)
    modelled = np.tile(torques, (len(SCALES), 1))
    grid = np.array([[scale] * width for scale in SCALES])
    errors = (alpha - grid) * modelled / kp + offset
    for name in frozen:
        errors[:, joints.index(name)] = errors[0, joints.index(name)]
    write_sweep(
        path,
        GravitySweep(
            group=GROUP,
            joint_names=joints,
            poses=np.zeros_like(modelled),
            modelled_torque=modelled,
            scales=grid,
            applied_torque=grid * modelled,
            errors=errors,
        ),
        profile,
    )


def _poses(tmp_path, profile, torques=(1.0, 3.0, 5.0), frozen=()):
    paths = []
    for index, torque in enumerate(torques):
        path = tmp_path / f"pose{index}.json"
        _write_pose(path, profile, torque, frozen=frozen)
        paths.append(str(path))
    return paths


def _argv(paths, output, *extra):
    argv = ["r2s", "identify"]
    for path in paths:
        argv += ["--sweep", path]
    return argv + ["--output", str(output), *extra]


def test_identify_recovers_the_stiffness_the_sweeps_were_built_from(
    tmp_path, profile, capsys
):
    output = tmp_path / "static.json"

    assert main(_argv(_poses(tmp_path, profile), output)) == 0

    payload = json.loads(output.read_text())
    kp, alpha, offset = _truth(len(profile.groups[GROUP].joints))
    np.testing.assert_allclose(payload["stiffness_nm_per_rad"], kp, rtol=1e-6)
    np.testing.assert_allclose(payload["torque_scale"], alpha, rtol=1e-6)
    np.testing.assert_allclose(payload["offset_rad"], offset, atol=1e-9)
    assert payload["group"] == GROUP
    assert len(payload["sources"]["sweep_sha256"]) == 3
    assert "r_aj_1" in capsys.readouterr().out


def test_identify_refuses_poses_that_do_not_vary_the_load(tmp_path, profile, capsys):
    """Two poses are only two poses if they load the joints differently.

    Nearly the same pose twice is the same single equation twice, and least
    squares would answer it anyway.
    """
    output = tmp_path / "static.json"

    code = main(_argv(_poses(tmp_path, profile, torques=(3.0, 3.02)), output))

    assert code == 3
    out = capsys.readouterr().out
    for joint in profile.groups[GROUP].joints:
        assert joint in out
    assert "condition" in out
    assert not output.exists(), "an under-determined fit must not be written"


def test_identify_names_a_joint_frozen_in_its_stiction_band(tmp_path, profile, capsys):
    output = tmp_path / "static.json"

    code = main(_argv(_poses(tmp_path, profile, frozen=("r_aj_4",)), output))

    assert code == 3
    out = capsys.readouterr().out
    assert "r_aj_4" in out
    # The other six were identified, and the report says so even though nothing
    # was written: that is what tells the operator which pose to add.
    assert "7.5" in out
    assert not output.exists()


def test_identify_needs_more_than_one_sweep(tmp_path, profile, capsys):
    """Refused before reading anything: one pose can never separate the two."""
    output = tmp_path / "static.json"
    paths = _poses(tmp_path, profile, torques=(1.0,))

    code = main(["r2s", "identify", "--sweep", paths[0], "--output", str(output)])

    assert code == 3
    assert "at least two" in capsys.readouterr().out


def test_identify_requires_an_output(tmp_path, profile, capsys):
    code = main(["r2s", "identify"] + sum(
        [["--sweep", path] for path in _poses(tmp_path, profile)], []
    ))

    assert code == 2
    assert "--output" in capsys.readouterr().out


def test_identify_refuses_a_sweep_from_another_asset(tmp_path, profile, capsys):
    output = tmp_path / "static.json"
    paths = _poses(tmp_path, profile)
    payload = json.loads(open(paths[0]).read())
    payload["asset"]["manifest_sha256"] = "0" * 64
    with open(paths[0], "w") as handle:
        json.dump(payload, handle)

    code = main(_argv(paths, output))

    assert code == 2
    assert "checksum" in capsys.readouterr().out


def test_identify_refuses_sweeps_of_different_groups(tmp_path, profile, capsys):
    output = tmp_path / "static.json"
    paths = _poses(tmp_path, profile)
    left = profile.groups["openarm_left_arm"]
    other = tmp_path / "left.json"
    width = len(left.joints)
    write_sweep(
        other,
        GravitySweep(
            group=left.name,
            joint_names=left.joints,
            poses=np.zeros((2, width)),
            modelled_torque=np.ones((2, width)),
            scales=np.array([[0.0] * width, [1.0] * width]),
            applied_torque=np.array([[0.0] * width, [1.0] * width]),
            errors=np.zeros((2, width)),
        ),
        profile,
    )

    code = main(_argv([*paths, str(other)], output))

    assert code == 2
    assert "different groups" in capsys.readouterr().out
