"""Identified parameters inside a calibration bundle, and what refuses them.

The header says what a bundle claims to be. The identified block says what its
numbers were measured on. A mismatch between the two is a block pasted from
another robot's bundle, which is the one failure a checksum cannot catch — the
checksum is recomputed on write, so it signs whatever was pasted.
"""

import json

import numpy as np
import pytest

from robot_control.calibration import (
    CalibrationError,
    identified_block,
    load_bundle,
    write_bundle,
)
from robot_control.identification import CombinedEstimate
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def _combined(profile, group=GROUP):
    joints = profile.groups[group].joints
    width = len(joints)
    return CombinedEstimate(
        joint_names=joints,
        inertia=np.linspace(0.35, 0.05, width),
        damping=np.linspace(1.5, 0.4, width),
        friction=np.linspace(0.4, 0.1, width),
        stiffness=np.linspace(20.0, 8.0, width),
        inertia_from_gravity=np.linspace(0.355, 0.051, width),
        disagreement=np.full(width, 0.01),
    )


def _base(profile):
    groups = {
        name: {
            "nominal": {
                "stiffness": 50.0,
                "damping": 2.0,
                "friction": 0.1,
                "effort_limit": 10.0,
                "velocity_limit": 2.0,
            }
        }
        for name in profile.groups
    }
    return {
        "schema_version": 2,
        "profile": profile.name,
        "asset": {"id": profile.asset_id, "manifest_sha256": profile.manifest_sha256},
        "controller": {
            "command_period_sec": 0.01,
            "delay_sec": 0.02,
            "interpolation": "linear",
            "filter": {"type": "none"},
        },
        "groups": groups,
    }


def _combined_fully_measured(profile, group=GROUP):
    """A CombinedEstimate as `combine` would return with a staircase behind it."""
    import dataclasses

    combined = _combined(profile, group)
    width = len(combined.joint_names)
    return dataclasses.replace(
        combined,
        bias=np.linspace(0.05, 0.02, width),
        coulomb_nm=np.linspace(0.08, 0.05, width),
        static_bias_nm=np.linspace(0.3, 0.1, width),
    )


def _with_identified(profile, **overrides):
    payload = _base(profile)
    block = identified_block(
        _combined(profile),
        profile,
        torque_scale=np.ones(7),
        sweep_sha256=["a" * 64, "b" * 64],
        track_sha256="c" * 64,
    )
    block.update(overrides)
    payload["groups"][GROUP]["identified"] = block
    return payload


def test_a_bundle_carries_identified_parameters(profile, tmp_path):
    path = tmp_path / "bundle.json"

    bundle = write_bundle(path, _with_identified(profile), profile)

    identified = bundle.identified[GROUP]
    assert identified.joint_names == profile.groups[GROUP].joints
    np.testing.assert_allclose(identified.inertia, np.linspace(0.35, 0.05, 7))
    np.testing.assert_allclose(identified.stiffness, np.linspace(20.0, 8.0, 7))
    assert identified.sweep_sha256 == ("a" * 64, "b" * 64)
    assert identified.track_sha256 == "c" * 64


def test_a_bundle_without_identified_parameters_still_loads(profile, tmp_path):
    """The block is additive: bundles predating it are not retroactively broken."""
    path = tmp_path / "bundle.json"

    bundle = write_bundle(path, _base(profile), profile)

    assert bundle.identified == {}


def test_parameters_fitted_against_another_asset_are_refused(profile, tmp_path):
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["fitted_against"]["manifest_sha256"] = "0" * 64
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="fitted against"):
        load_bundle(path, profile)


def test_parameters_fitted_against_another_profile_are_refused(profile, tmp_path):
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["fitted_against"]["profile"] = "someone_else"
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="fitted against"):
        load_bundle(path, profile)


def test_a_resigned_bundle_does_not_launder_a_pasted_block(profile, tmp_path):
    """The checksum signs whatever is there, so it cannot be the check."""
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["fitted_against"]["asset_id"] = "other-arm"

    with pytest.raises(CalibrationError, match="fitted against"):
        write_bundle(path, payload, profile)


def test_parameters_for_the_wrong_joints_are_refused(profile, tmp_path):
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["joint_names"][0] = "l_aj_1"
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="joint"):
        load_bundle(path, profile)


def test_a_short_parameter_array_is_refused(profile, tmp_path):
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["inertia_kg_m2"].pop()
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="inertia_kg_m2"):
        load_bundle(path, profile)


@pytest.mark.parametrize(
    "key, value",
    [
        ("inertia_kg_m2", 0.0),
        ("stiffness_nm_per_rad", -1.0),
        ("damping_nm_s_per_rad", -0.1),
        ("friction_nm", -0.1),
    ],
)
def test_a_physically_impossible_parameter_is_refused(profile, tmp_path, key, value):
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"][key][2] = value
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match=key):
        load_bundle(path, profile)


def test_parameters_with_no_provenance_are_refused(profile, tmp_path):
    """A number with no source is not a measurement."""
    path = tmp_path / "bundle.json"
    payload = _with_identified(profile)
    payload["groups"][GROUP]["identified"]["provenance"]["sweep_sha256"] = []
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="provenance"):
        load_bundle(path, profile)


def test_the_bundle_carries_the_staircase_measurements_through(profile, tmp_path):
    """Fc, Fo and the static bias are additive fields: written when the
    CombinedEstimate behind the block carries them."""
    path = tmp_path / "bundle.json"
    payload = _base(profile)
    payload["groups"][GROUP]["identified"] = identified_block(
        _combined_fully_measured(profile),
        profile,
        torque_scale=np.ones(7),
        sweep_sha256=["a" * 64, "b" * 64],
        track_sha256="c" * 64,
    )

    bundle = write_bundle(path, payload, profile)

    identified = bundle.identified[GROUP]
    np.testing.assert_allclose(identified.bias, np.linspace(0.05, 0.02, 7))
    np.testing.assert_allclose(identified.coulomb_nm, np.linspace(0.08, 0.05, 7))
    np.testing.assert_allclose(identified.static_bias_nm, np.linspace(0.3, 0.1, 7))


def test_a_bundle_predating_the_staircase_fields_reads_them_as_nan(profile, tmp_path):
    """Additive, like the rest of the identified block: an older bundle has
    nothing to say about Fc or Fo, not a zero."""
    path = tmp_path / "bundle.json"

    bundle = write_bundle(path, _with_identified(profile), profile)

    identified = bundle.identified[GROUP]
    assert np.all(np.isnan(identified.coulomb_nm))
    assert np.all(np.isnan(identified.bias))
    assert np.all(np.isnan(identified.static_bias_nm))


def test_a_nan_measurement_writes_null_on_disk_and_reads_back_as_nan(profile, tmp_path):
    """A joint the staircase never covered is a legitimate, expected state:
    written as JSON's own null, not the non-standard `NaN` token, and not
    refused. Reproduces the reviewer's exact case: one joint measured, the
    rest not.
    """
    import dataclasses

    path = tmp_path / "bundle.json"
    payload = _base(profile)
    coulomb = np.linspace(0.08, 0.05, 7)
    coulomb[1:] = np.nan
    combined = dataclasses.replace(_combined_fully_measured(profile), coulomb_nm=coulomb)
    payload["groups"][GROUP]["identified"] = identified_block(
        combined,
        profile,
        torque_scale=np.ones(7),
        sweep_sha256=["a" * 64],
        track_sha256="c" * 64,
    )

    bundle = write_bundle(path, payload, profile)

    text = path.read_text()
    assert "null" in text
    assert "NaN" not in text
    identified = bundle.identified[GROUP]
    assert identified.coulomb_nm[0] == pytest.approx(0.08)
    assert np.all(np.isnan(identified.coulomb_nm[1:]))


def test_a_negative_coulomb_measurement_is_refused(profile, tmp_path):
    """Fc is a friction magnitude; a joint cannot measure a negative one."""
    path = tmp_path / "bundle.json"
    payload = _base(profile)
    block = identified_block(
        _combined_fully_measured(profile),
        profile,
        torque_scale=np.ones(7),
        sweep_sha256=["a" * 64],
        track_sha256="c" * 64,
    )
    block["coulomb_nm"][2] = -0.01
    payload["groups"][GROUP]["identified"] = block
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="coulomb_nm"):
        load_bundle(path, profile)


def test_the_block_records_the_cross_check_it_survived(profile, tmp_path):
    """Whether the two inertias agreed is part of what the numbers are worth."""
    path = tmp_path / "bundle.json"

    bundle = write_bundle(path, _with_identified(profile), profile)

    np.testing.assert_allclose(
        bundle.identified[GROUP].inertia_disagreement, np.full(7, 0.01)
    )
