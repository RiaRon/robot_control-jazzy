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


def _sweep(profile, group=GROUP, sweep_joint=None, applied=None, modelled=None):
    joints = profile.groups[group].joints
    width = len(joints)
    rounds = (
        np.asarray(modelled).shape[0]
        if modelled is not None
        else (np.asarray(applied).shape[0] if applied is not None else 3)
    )
    scales = np.linspace(0.0, 1.0, rounds)[:, None] * np.ones(width)
    poses = np.tile(np.linspace(0.0, 0.3, width), (rounds, 1))
    torque = (
        np.asarray(modelled, dtype=float)
        if modelled is not None
        else np.tile(np.linspace(1.0, 5.0, width), (rounds, 1))
    )
    applied_torque = (
        np.asarray(applied, dtype=float) if applied is not None else scales * torque
    )
    return GravitySweep(
        group=group,
        joint_names=joints,
        poses=poses,
        modelled_torque=torque,
        scales=scales,
        applied_torque=applied_torque,
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
    for field in ("poses", "modelled_torque", "scales", "applied_torque", "errors"):
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
    other = dataclasses.replace(profile, asset_id="some_other_robot")

    with pytest.raises(ArtifactError, match="profile or asset"):
        read_sweep(path, other)


def test_a_sweep_outlives_a_manifest_revision_of_its_own_asset(profile, tmp_path):
    """A manifest revision (a camera moved, a note added) is the same arm.

    The joints the numbers describe are guarded separately, by name — refusing
    on the hash alone would orphan every measured file each time the asset's
    metadata was touched.
    """
    path = tmp_path / "sweep.json"
    write_sweep(path, _sweep(profile), profile)
    revised = dataclasses.replace(profile, manifest_sha256="0" * 64)

    sweep = read_sweep(path, revised)
    assert list(sweep.joint_names) == list(_sweep(profile).joint_names)


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


def test_a_version_1_sweep_still_loads(profile, tmp_path):
    """Nine sweeps were collected before applied_torque existed. In a v1 file
    the applied torque is the scale times the model, because that is the only
    thing the old excitation could publish."""
    path = tmp_path / "v1.json"
    write_sweep(path, _sweep(profile), profile)
    payload = json.loads(path.read_text())
    payload["schema_version"] = 1
    for entry in payload["rounds"]:
        entry.pop("applied_torque")
    _resign(path, payload)

    loaded = read_sweep(path, profile)

    assert loaded.applied_torque == pytest.approx(
        loaded.scales * loaded.modelled_torque
    )


def test_a_version_2_sweep_round_trips_an_applied_torque_the_model_cannot_explain(
    profile, tmp_path
):
    sweep = _sweep(profile, applied=np.full((2, 7), 0.4), modelled=np.zeros((2, 7)))
    path = tmp_path / "v2.json"
    write_sweep(path, sweep, profile)

    assert read_sweep(path, profile).applied_torque == pytest.approx(0.4)


def test_a_version_1_sweep_with_applied_torque_present_uses_the_stored_value(
    profile, tmp_path
):
    """Transitional files: schema_version 1, written after applied_torque
    joined the round dict but before the version was bumped for it.
    Reconstruction has to key off whether the field is present, not off the
    version number, or this file's real applied torque gets thrown away in
    favour of a recomputed one that disagrees with it."""
    sweep = _sweep(profile, applied=np.full((2, 7), 0.4), modelled=np.zeros((2, 7)))
    path = tmp_path / "v1_transitional.json"
    write_sweep(path, sweep, profile)
    payload = json.loads(path.read_text())
    payload["schema_version"] = 1
    _resign(path, payload)

    loaded = read_sweep(path, profile)

    assert loaded.applied_torque == pytest.approx(0.4)


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
            applied_torque=good.applied_torque[:0],
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
    artifact = read_static_estimate(path, profile)
    loaded = artifact.estimate

    assert artifact.group == GROUP
    assert artifact.sweep_sha256 == ("a" * 64,)
    assert artifact.noise_rad == pytest.approx(4e-4)
    assert loaded.joint_names == original.joint_names
    np.testing.assert_allclose(loaded.stiffness, original.stiffness)
    np.testing.assert_allclose(loaded.torque_scale, original.torque_scale)
    np.testing.assert_allclose(loaded.offset, original.offset)
    assert json.loads(path.read_text())["sources"]["sweep_sha256"] == ["a" * 64]


def test_a_static_estimate_carries_coulomb_and_bias_through_a_file(profile, tmp_path):
    """One joint covered by a staircase, six not — the shape `_identify`
    produces once a staircase covers only one joint of a group."""
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    width = len(profile.groups[GROUP].joints)
    coulomb = np.full(width, np.nan)
    coulomb[0] = 0.08
    bias = np.full(width, np.nan)
    bias[0] = 0.3
    original = dataclasses.replace(_estimate(profile), coulomb_nm=coulomb, bias_nm=bias)
    path = tmp_path / "static.json"

    write_static_estimate(
        path, original, profile, group=GROUP, noise_rad=4e-4, sources=["a" * 64]
    )
    loaded = read_static_estimate(path, profile).estimate

    assert loaded.coulomb_nm[0] == pytest.approx(0.08)
    assert loaded.bias_nm[0] == pytest.approx(0.3)
    assert np.all(np.isnan(loaded.coulomb_nm[1:]))
    assert np.all(np.isnan(loaded.bias_nm[1:]))
    # json.loads parses both `null` and the non-standard `NaN` token the same
    # way, so this has to read the file as raw text to tell them apart.
    text = path.read_text()
    assert "null" in text
    assert "NaN" not in text


def test_the_static_bias_key_says_the_gravity_torque_is_still_in_it(
    profile, tmp_path
):
    """A staircase's bias is Fo + tau_gravity(pose) — where the midline of the
    two branches crosses zero torque, with the pose's gravity torque still in
    it. `r2s fit` writes a key called `bias_nm` too, and that one is Fo alone.
    Two files, one name, two quantities: nothing in the code confuses them, but
    the files are meant to be read side by side by a person.
    """
    from robot_control.artifacts import write_static_estimate

    width = len(profile.groups[GROUP].joints)
    bias = np.full(width, np.nan)
    bias[0] = 0.3
    original = dataclasses.replace(_estimate(profile), bias_nm=bias)
    path = tmp_path / "static.json"

    write_static_estimate(
        path, original, profile, group=GROUP, noise_rad=4e-4, sources=[]
    )

    payload = json.loads(path.read_text())
    assert payload["bias_with_gravity_nm"][0] == pytest.approx(0.3)
    assert "bias_nm" not in payload


def test_a_static_estimate_written_before_the_rename_still_loads(profile, tmp_path):
    """The reader keys off presence, so the old name costs nothing to accept
    and every estimate already on disk carries it."""
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    width = len(profile.groups[GROUP].joints)
    bias = np.full(width, np.nan)
    bias[0] = 0.3
    original = dataclasses.replace(_estimate(profile), bias_nm=bias)
    path = tmp_path / "static.json"
    write_static_estimate(
        path, original, profile, group=GROUP, noise_rad=4e-4, sources=[]
    )
    payload = json.loads(path.read_text())
    payload["bias_nm"] = payload.pop("bias_with_gravity_nm")
    payload["schema_version"] = 2
    _resign(path, payload)

    loaded = read_static_estimate(path, profile).estimate

    assert loaded.bias_nm[0] == pytest.approx(0.3)


def test_a_static_estimate_with_no_staircase_never_writes_the_keys_at_all(
    profile, tmp_path
):
    """The common case: a gravity-only run. coulomb_nm/bias_nm stay `None` —
    `fit_static_gravity` never sets them — and the file this writes carries
    neither key at all, rather than one full of `null`."""
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    path = tmp_path / "static.json"
    write_static_estimate(
        path, _estimate(profile), profile, group=GROUP, noise_rad=4e-4, sources=[]
    )

    payload = json.loads(path.read_text())
    assert "coulomb_nm" not in payload
    assert "bias_nm" not in payload
    loaded = read_static_estimate(path, profile).estimate
    assert loaded.coulomb_nm is None
    assert loaded.bias_nm is None


def test_a_version_1_static_estimate_still_loads(profile, tmp_path):
    """Every static estimate on disk before this task predates coulomb_nm and
    bias_nm entirely; the reader has to fill None for both rather than
    refuse, or every file already written stops loading."""
    from robot_control.artifacts import read_static_estimate, write_static_estimate

    path = tmp_path / "v1.json"
    write_static_estimate(
        path, _estimate(profile), profile, group=GROUP, noise_rad=4e-4, sources=[]
    )
    payload = json.loads(path.read_text())
    payload["schema_version"] = 1
    _resign(path, payload)

    loaded = read_static_estimate(path, profile).estimate

    assert loaded.coulomb_nm is None
    assert loaded.bias_nm is None


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


def test_a_static_estimate_with_undetermined_alpha_is_never_written(profile, tmp_path):
    """torque_scale can be nan for a joint that is not in `unidentifiable` at
    all -- its stiffness fit succeeded, only alpha (a zero-model column) did
    not. Writing that nan would crash the json encoder rather than refuse
    cleanly, since json.dumps(allow_nan=False) does not raise ArtifactError."""
    from robot_control.artifacts import write_static_estimate

    original = _estimate(profile)
    holed = dataclasses.replace(
        original,
        torque_scale=np.where(
            np.arange(len(original.joint_names)) == 2, np.nan, original.torque_scale
        ),
    )
    path = tmp_path / "static.json"

    with pytest.raises(ArtifactError, match="r_aj_3"):
        write_static_estimate(
            path, holed, profile, group=GROUP, noise_rad=4e-4, sources=[]
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
