"""A gravity sweep, persisted.

The sweep is already an identification experiment; these tests are about it
surviving the trip to disk intact, and refusing to be read against a robot it
was not measured on.
"""

import dataclasses
import hashlib
import json

import numpy as np
import pytest

from robot_control.artifacts import ArtifactError, read_sweep, write_sweep
from robot_control.identification import FitError, GravitySweep
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def _sweep(profile, group=GROUP, sweep_joint=None):
    joints = profile.groups[group].joints
    width = len(joints)
    scales = np.array([[0.0] * width, [0.5] * width, [1.0] * width])
    poses = np.tile(np.linspace(0.0, 0.3, width), (3, 1))
    torque = np.tile(np.linspace(1.0, 5.0, width), (3, 1))
    return GravitySweep(
        group=group,
        joint_names=joints,
        poses=poses,
        modelled_torque=torque,
        scales=scales,
        errors=(torque - scales * torque) / 20.0,
        sweep_joint=sweep_joint,
    )


def test_a_sweep_round_trips_through_a_file(profile, tmp_path):
    original = _sweep(profile, sweep_joint="r_aj_2")
    path = tmp_path / "sweep.json"

    write_sweep(path, original, profile)
    loaded = read_sweep(path, profile)

    assert loaded.group == original.group
    assert loaded.joint_names == original.joint_names
    assert loaded.sweep_joint == "r_aj_2"
    for field in ("poses", "modelled_torque", "scales", "errors"):
        np.testing.assert_allclose(getattr(loaded, field), getattr(original, field))


def test_a_sweep_carries_its_profile_and_asset(profile, tmp_path):
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)

    payload = json.loads(path.read_text())

    assert payload["profile"] == profile.name
    assert payload["asset"]["id"] == profile.asset_id
    assert payload["asset"]["manifest_sha256"] == profile.manifest_sha256


def test_a_sweep_measured_on_one_asset_is_refused_against_another(profile, tmp_path):
    """The numbers describe a specific robot; a different one would be a lie."""
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    other = dataclasses.replace(profile, manifest_sha256="0" * 64)

    with pytest.raises(ArtifactError, match="profile or asset"):
        read_sweep(path, other)


def test_an_edited_sweep_is_refused(profile, tmp_path):
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload["rounds"][0]["error"][0] += 0.01
    path.write_text(json.dumps(payload))

    with pytest.raises(ArtifactError, match="checksum"):
        read_sweep(path, profile)


def _resign(path, payload):
    """Rewrite *payload* with a valid checksum, so a test can reach past it."""
    payload = {key: value for key, value in payload.items() if key != "checksum_sha256"}
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    payload["checksum_sha256"] = hashlib.sha256(canonical).hexdigest()
    path.write_text(json.dumps(payload))


def test_a_sweep_naming_joints_the_group_does_not_have_is_refused(profile, tmp_path):
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload["joint_names"][0] = "l_aj_1"
    _resign(path, payload)

    with pytest.raises(ArtifactError, match="joint"):
        read_sweep(path, profile)


def test_a_sweep_for_an_unknown_group_is_refused(profile, tmp_path):
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload["group"] = "no_such_group"
    _resign(path, payload)

    with pytest.raises(ArtifactError, match="group"):
        read_sweep(path, profile)


def test_a_future_schema_is_refused_rather_than_guessed(profile, tmp_path):
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload["schema_version"] = 99
    _resign(path, payload)

    with pytest.raises(ArtifactError, match="schema_version"):
        read_sweep(path, profile)


def test_a_sweep_with_no_checksum_is_refused(profile, tmp_path):
    """Unlike a calibration bundle, a sweep is always written by this tool, so
    an unsigned one was written by something else."""
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload.pop("checksum_sha256")
    path.write_text(json.dumps(payload))

    with pytest.raises(ArtifactError, match="checksum"):
        read_sweep(path, profile)


def test_a_round_missing_a_measurement_is_refused(profile):
    """Every round needs a pose, a torque, a scale and an error, or none fit."""
    good = _sweep(profile)

    with pytest.raises(FitError, match="round"):
        dataclasses.replace(good, errors=good.errors[:2])


def test_a_sweep_with_no_rounds_is_refused(profile):
    good = _sweep(profile)

    with pytest.raises(FitError, match="round"):
        dataclasses.replace(
            good,
            poses=good.poses[:0],
            modelled_torque=good.modelled_torque[:0],
            scales=good.scales[:0],
            errors=good.errors[:0],
        )


def test_a_sweep_joint_outside_the_sweep_is_refused(profile):
    with pytest.raises(FitError, match="sweep_joint"):
        _sweep(profile, sweep_joint="l_aj_1")


def test_a_sweep_digest_changes_with_what_it_measured(profile):
    from robot_control.artifacts import sweep_sha256

    original = _sweep(profile)
    nudged = dataclasses.replace(original, errors=original.errors + 1e-9)

    assert sweep_sha256(original) == sweep_sha256(_sweep(profile))
    assert sweep_sha256(original) != sweep_sha256(nudged)


def _estimate(profile, group=GROUP):
    from robot_control.identification import StaticEstimate

    joints = profile.groups[group].joints
    width = len(joints)
    return StaticEstimate(
        joint_names=joints,
        stiffness=np.linspace(7.5, 28.0, width),
        torque_scale=np.linspace(0.9, 1.2, width),
        offset=np.full(width, 0.0015),
        residual_rmse=np.full(width, 3e-4),
        condition=np.full(width, 12.0),
        used=np.full(width, 12, dtype=int),
        excluded=np.zeros(width, dtype=int),
        unidentifiable=(),
    )


def test_a_static_estimate_round_trips_through_a_file(profile, tmp_path):
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    original = _estimate(profile)
    path = tmp_path / "static.json"

    write_static_estimate(
        path, original, profile, group=GROUP, noise_rad=4e-4, sources=["a" * 64]
    )
    group, loaded = read_static_estimate(path, profile)

    assert group == GROUP
    assert loaded.joint_names == original.joint_names
    np.testing.assert_allclose(loaded.stiffness, original.stiffness)
    np.testing.assert_allclose(loaded.torque_scale, original.torque_scale)
    np.testing.assert_allclose(loaded.offset, original.offset)
    assert json.loads(path.read_text())["sources"]["sweep_sha256"] == ["a" * 64]


def test_an_incomplete_static_estimate_is_never_written(profile, tmp_path):
    """A hole in the file reads back as an answer for the joints it does have."""
    from robot_control.artifacts import write_static_estimate

    partial = dataclasses.replace(
        _estimate(profile), unidentifiable=(("r_aj_4", "frozen"),)
    )
    path = tmp_path / "static.json"

    with pytest.raises(ArtifactError, match="r_aj_4"):
        write_static_estimate(
            path, partial, profile, group=GROUP, noise_rad=4e-4, sources=[]
        )
    assert not path.exists()


def test_a_static_estimate_is_refused_against_another_asset(profile, tmp_path):
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    path = tmp_path / "static.json"
    write_static_estimate(
        path, _estimate(profile), profile, group=GROUP, noise_rad=4e-4, sources=[]
    )
    other = dataclasses.replace(profile, asset_id="something-else")

    with pytest.raises(ArtifactError, match="profile or asset"):
        read_static_estimate(path, other)


def test_a_sweep_cannot_be_read_as_a_static_estimate(profile, tmp_path):
    """The two files share a header; nothing else about them is interchangeable."""
    from robot_control.artifacts import read_static_estimate

    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)

    with pytest.raises(ArtifactError, match="expected a static_gravity_estimate"):
        read_static_estimate(path, profile)
