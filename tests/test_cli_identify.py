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


def _poses_with_zero_model_joint(tmp_path, profile, joint, torques=(1.0, 3.0, 5.0)):
    """Poses like `_poses`, except one joint's model is exactly zero at every
    round and its applied torque instead sweeps independently — the direct-
    torque scenario Task 3 exists for, where stiffness is identifiable but the
    torque-model correction (alpha) genuinely is not."""
    joints = profile.groups[GROUP].joints
    width = len(joints)
    kp, alpha, offset = _truth(width)
    index = joints.index(joint)
    zero_applied = np.linspace(-0.6, 0.6, len(SCALES))

    paths = []
    for pose_index, torque in enumerate(torques):
        path = tmp_path / f"pose{pose_index}.json"
        pose_torques = np.full(width, torque) * np.linspace(1.0, 2.0, width)
        modelled = np.tile(pose_torques, (len(SCALES), 1))
        grid = np.array([[scale] * width for scale in SCALES])
        applied = grid * modelled
        modelled[:, index] = 0.0
        applied[:, index] = zero_applied
        errors = (alpha - grid) * modelled / kp + offset
        errors[:, index] = -applied[:, index] / kp[index] + offset[index]
        write_sweep(
            path,
            GravitySweep(
                group=GROUP,
                joint_names=joints,
                poses=np.zeros_like(modelled),
                modelled_torque=modelled,
                scales=grid,
                applied_torque=applied,
                errors=errors,
            ),
            profile,
        )
        paths.append(str(path))
    return paths


def test_identify_reports_but_refuses_to_write_an_undetermined_alpha(
    tmp_path, profile, capsys
):
    """r_aj_5's model is zero at every round: its stiffness is still
    identified from the applied torque directly, but alpha multiplies a zero
    column and is genuinely undetermined, not merely 0. The report must show
    it the same way it already shows any other unidentified value — not a
    bare 'nan' next to plausible numbers — and the file must not be written,
    since a hole in torque_scale is exactly the kind of hole `_identify`'s own
    docstring says it refuses rather than reports."""
    output = tmp_path / "static.json"
    paths = _poses_with_zero_model_joint(tmp_path, profile, joint="r_aj_5")

    code = main(_argv(paths, output))

    out = capsys.readouterr().out
    assert "r_aj_5" in out
    assert "nan" not in out.lower()
    kp, _alpha, _offset = _truth(len(profile.groups[GROUP].joints))
    zero_index = profile.groups[GROUP].joints.index("r_aj_5")
    # Its stiffness is still reported: this is not the fully-unidentified path.
    assert f"{kp[zero_index]:.2f}" in out
    assert code == 3
    assert not output.exists()


def _staircase_pose(tmp_path, profile, joint, peak=0.6, steps=5, coulomb=0.05):
    """Write one sweep shaped like `pose torque`'s output: scales all zero,
    the applied torque tracing a staircase on *joint* while every other joint
    holds at zero applied torque and the group's own offset error — the shape
    `_identify` must tell apart from a gravity sweep by scales-plus-motion,
    not by name.
    """
    from robot_control.excitation import staircase

    joints = profile.groups[GROUP].joints
    width = len(joints)
    kp, _alpha, offset = _truth(width)
    index = joints.index(joint)
    values = staircase(peak, steps)
    rounds = len(values)

    applied = np.zeros((rounds, width))
    errors = np.tile(offset, (rounds, 1))
    held = None
    for row, torque in enumerate(values):
        target = -torque / kp[index] + offset[index]
        if held is None or abs(target - held) > coulomb / kp[index]:
            direction = 1.0 if row < steps else -1.0
            held = target + direction * coulomb / kp[index]
        applied[row, index] = torque
        errors[row, index] = held

    path = tmp_path / "staircase.json"
    write_sweep(
        path,
        GravitySweep(
            group=GROUP,
            joint_names=joints,
            poses=np.zeros((rounds, width)),
            modelled_torque=np.zeros((rounds, width)),
            scales=np.zeros((rounds, width)),
            applied_torque=applied,
            errors=errors,
        ),
        profile,
    )
    return str(path)


def _joint_row(out, joint):
    for line in out.splitlines():
        if line.strip().startswith(joint):
            return line
    raise AssertionError(f"no report row for {joint!r} in:\n{out}")


def test_identify_reports_fc_and_fo_from_a_staircase_sweep(tmp_path, profile, capsys):
    """A staircase mixed in with the usual gravity sweeps must not be mistaken
    for gravity data — kp/alpha stay the gravity fit's — but must still carry
    Fc and Fo for the joint it covered, and leave every other joint's Fc/Fo at
    the file's usual '—' for a value that was never measured. This is the
    real origin of the chain the brief describes end to end: `_identify`
    actually runs `fit_staircase` and the merged estimate actually reaches
    the written file, not just the console.
    """
    output = tmp_path / "static.json"
    paths = _poses(tmp_path, profile) + [
        _staircase_pose(tmp_path, profile, joint="r_aj_5")
    ]

    code = main(_argv(paths, output))

    assert code == 0
    out = capsys.readouterr().out
    covered = _joint_row(out, "r_aj_5")
    uncovered = _joint_row(out, "r_aj_1")
    assert "—" not in covered, covered
    assert "—" in uncovered, uncovered
    # json.loads parses `null` and the non-standard `NaN` token the same way,
    # so the only way to pin which one is actually on disk is to read the
    # file as raw text before parsing it.
    text = output.read_text()
    assert "null" in text
    assert "NaN" not in text
    payload = json.loads(text)
    joints = profile.groups[GROUP].joints
    covered_index = joints.index("r_aj_5")
    coulomb = payload["coulomb_nm"]
    # Fo + tau_gravity(pose), which is why it is not called bias_nm: the fit
    # file's key of that name holds Fo alone.
    bias = payload["bias_with_gravity_nm"]
    assert coulomb[covered_index] is not None
    assert coulomb[covered_index] == pytest.approx(0.05, rel=0.15)
    assert bias[covered_index] is not None
    for index in range(len(joints)):
        if index == covered_index:
            continue
        assert coulomb[index] is None, coulomb
        assert bias[index] is None, bias


def test_identify_continues_when_the_staircase_fit_raises(
    tmp_path, profile, monkeypatch, capsys
):
    """`fit_staircase` raising is a structural failure (no sweeps, mismatched
    joints) rather than a per-joint one, and unreachable in practice here
    since `fit_static_gravity` already validated group/joint agreement across
    the same sweep list moments earlier. Still, the brief asks for the catch,
    so this pins the behaviour with a monkeypatch: report and continue on the
    gravity estimate, the same technique Task 9 used for its own
    unreachable-in-practice path.
    """
    from robot_control.identification import FitError

    def _raise(_sweeps):
        raise FitError("staircase blew up")

    monkeypatch.setattr("robot_control.cli.fit_staircase", _raise)
    output = tmp_path / "static.json"
    paths = _poses(tmp_path, profile) + [
        _staircase_pose(tmp_path, profile, joint="r_aj_5")
    ]

    code = main(_argv(paths, output))

    assert code == 0
    out = capsys.readouterr().out
    assert "staircase: staircase blew up" in out
    payload = json.loads(output.read_text())
    # The merge never happened, so the file looks exactly like a gravity-only
    # run: neither key present, same as `fit_static_gravity`'s own None.
    assert "coulomb_nm" not in payload
    assert "bias_with_gravity_nm" not in payload
    assert "bias_nm" not in payload


def test_identify_without_a_staircase_still_writes_the_same_numbers(
    tmp_path, profile, capsys
):
    """Every existing caller passes gravity sweeps only. Wiring the staircase
    fit in must not change what a gravity-only run measures or writes — the
    regression guard for Task 11.
    """
    output = tmp_path / "static.json"

    assert main(_argv(_poses(tmp_path, profile), output)) == 0

    payload = json.loads(output.read_text())
    kp, alpha, offset = _truth(len(profile.groups[GROUP].joints))
    np.testing.assert_allclose(payload["stiffness_nm_per_rad"], kp, rtol=1e-6)
    np.testing.assert_allclose(payload["torque_scale"], alpha, rtol=1e-6)
    np.testing.assert_allclose(payload["offset_rad"], offset, atol=1e-9)
    out = capsys.readouterr().out
    # No staircase sweep means Fc/Fo were never measured for any joint: the
    # None-handling path, distinct from fit_staircase leaving a joint NaN.
    for joint in profile.groups[GROUP].joints:
        assert "—" in _joint_row(out, joint)


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
