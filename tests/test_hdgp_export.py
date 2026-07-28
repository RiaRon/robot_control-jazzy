"""Converting a calibration bundle into the file hdgp's training env reads.

hdgp's ``get_actuator_params`` returns the env's own defaults for any group name
it does not find, and says nothing. A calibration that names its groups wrongly
therefore trains exactly as if it had never been produced. Every test here
exists because that failure is silent on the far side.
"""

import json
from dataclasses import replace

import numpy as np
import pytest

from robot_control.calibration import identified_block, write_bundle
from robot_control.hdgp_export import (
    HDGP_SCHEMA_VERSION,
    HdgpExportError,
    hdgp_calibration,
    write_hdgp_calibration,
)
from robot_control.identification import CombinedEstimate
from robot_control.profile import load_builtin_profile


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def _combined(
    profile,
    group,
    *,
    stiffness,
    damping,
    friction=0.2,
    coulomb_nm=None,
    bias=None,
    static_bias_nm=None,
):
    joints = profile.groups[group].joints
    width = len(joints)
    return CombinedEstimate(
        joint_names=joints,
        inertia=np.full(width, 0.1),
        damping=np.asarray(damping, dtype=float),
        friction=np.full(width, friction),
        stiffness=np.asarray(stiffness, dtype=float),
        inertia_from_gravity=np.full(width, 0.1),
        disagreement=np.zeros(width),
        bias=None if bias is None else np.asarray(bias, dtype=float),
        coulomb_nm=None if coulomb_nm is None else np.asarray(coulomb_nm, dtype=float),
        static_bias_nm=(
            None if static_bias_nm is None else np.asarray(static_bias_nm, dtype=float)
        ),
    )


def _bundle(tmp_path, profile, measured):
    """Write a v2 bundle carrying an identified block for each named group."""
    groups = {
        name: {"nominal": {"stiffness": 50.0, "damping": 2.0, "friction": 0.1}}
        for name in profile.groups
    }
    for name, combined in measured.items():
        groups[name]["identified"] = identified_block(
            combined,
            profile,
            torque_scale=np.ones(len(combined.joint_names)),
            sweep_sha256=["a" * 64],
            track_sha256="c" * 64,
        )
    payload = {
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
    return write_bundle(tmp_path / "bundle.json", payload, profile)


def _uniform_curl(profile, *, stiffness=30.0, damping=5.0):
    width = len(profile.groups["tesollo_curl"].joints)
    return _combined(
        profile,
        "tesollo_curl",
        stiffness=np.full(width, stiffness),
        damping=np.full(width, damping),
    )


def test_group_is_renamed_to_the_name_the_training_env_uses(tmp_path, profile):
    bundle = _bundle(tmp_path, profile, {"tesollo_curl": _uniform_curl(profile)})

    payload = hdgp_calibration(bundle, profile)

    # The profile calls it tesollo_curl; the env's actuator dict does not.
    assert set(payload["groups"]) == {"tesollo_hand_curl"}
    assert payload["schema_version"] == HDGP_SCHEMA_VERSION
    assert payload["robot_asset"] == profile.asset_id


def test_measured_values_survive_the_conversion(tmp_path, profile):
    bundle = _bundle(
        tmp_path, profile, {"tesollo_curl": _uniform_curl(profile, stiffness=27.5)}
    )

    group = hdgp_calibration(bundle, profile)["groups"]["tesollo_hand_curl"]

    assert group["stiffness"] == pytest.approx(27.5)
    assert group["damping"] == pytest.approx(5.0)
    assert group["joint_friction"] == pytest.approx(0.2)


def test_unmeasured_groups_are_absent_rather_than_guessed(tmp_path, profile):
    bundle = _bundle(tmp_path, profile, {"tesollo_curl": _uniform_curl(profile)})

    payload = hdgp_calibration(bundle, profile)

    # A group with no measurement must not appear at all: hdgp then keeps the
    # env's own gain, which is the honest answer. Writing the bundle's nominal
    # guess instead would look like a measurement downstream.
    assert "tesollo_hand_abduction" not in payload["groups"]


def test_a_measured_group_with_no_declared_hdgp_name_is_refused(tmp_path, profile):
    bundle = _bundle(tmp_path, profile, {"tesollo_curl": _uniform_curl(profile)})
    undeclared = replace(
        profile,
        groups={
            **profile.groups,
            "tesollo_curl": replace(profile.groups["tesollo_curl"], hdgp_group=None),
        },
    )

    # Dropping it silently would leave the env on its default gain while the
    # export reported success, which is the whole failure this guards.
    with pytest.raises(HdgpExportError, match="tesollo_curl"):
        hdgp_calibration(bundle, undeclared)


def test_two_groups_cannot_claim_the_same_hdgp_name(tmp_path, profile):
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "tesollo_curl": _uniform_curl(profile),
            "tesollo_pip": _combined(
                profile,
                "tesollo_pip",
                stiffness=np.full(5, 12.0),
                damping=np.full(5, 3.0),
            ),
        },
    )
    collided = replace(
        profile,
        groups={
            **profile.groups,
            "tesollo_pip": replace(
                profile.groups["tesollo_pip"], hdgp_group="tesollo_hand_curl"
            ),
        },
    )

    with pytest.raises(HdgpExportError, match="tesollo_hand_curl"):
        hdgp_calibration(bundle, collided)


def test_joints_that_disagree_too_far_are_not_averaged_into_one_number(
    tmp_path, profile
):
    # The real arm runs kp 70 / 60 / 10 across its seven joints. One scalar for
    # the group is a fiction there, and a fiction that trains without complaint.
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=[70, 70, 70, 60, 10, 10, 10],
                damping=[2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
            )
        },
    )

    with pytest.raises(HdgpExportError, match="stiffness"):
        hdgp_calibration(bundle, profile)


def test_the_spread_a_caller_accepts_is_theirs_to_widen(tmp_path, profile):
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=[70, 70, 70, 60, 10, 10, 10],
                damping=[2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
            )
        },
    )

    payload = hdgp_calibration(bundle, profile, max_spread=2.0)

    assert payload["groups"]["openarm_right_arm"]["stiffness"] == pytest.approx(42.857, rel=1e-3)


def test_the_per_joint_measurement_is_carried_alongside_the_collapsed_one(
    tmp_path, profile
):
    bundle = _bundle(tmp_path, profile, {"tesollo_curl": _uniform_curl(profile)})

    measured = hdgp_calibration(bundle, profile)["measured"]["tesollo_hand_curl"]

    # hdgp reads only the scalar. The joint-wise numbers are what makes the
    # scalar auditable after the fact, and inertia has no hdgp entry point at
    # all, so this block is the only place it survives.
    assert measured["source_group"] == "tesollo_curl"
    assert measured["joint_names"] == list(profile.groups["tesollo_curl"].joints)
    assert len(measured["inertia_kg_m2"]) == len(measured["joint_names"])
    assert measured["spread"]["stiffness"] == pytest.approx(0.0)


def test_the_written_file_is_what_hdgps_loader_accepts(tmp_path, profile):
    bundle = _bundle(tmp_path, profile, {"tesollo_curl": _uniform_curl(profile)})
    payload = hdgp_calibration(bundle, profile)
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    reloaded = json.loads(path.read_text())

    # Mirrors hdgp/source/openarm/.../real2sim_actuator_cfg.py: schema v1, a
    # non-empty groups object, and float-valued gains under those exact keys.
    assert int(reloaded["schema_version"]) == 1
    assert isinstance(reloaded["groups"], dict) and reloaded["groups"]
    for body in reloaded["groups"].values():
        assert isinstance(body, dict)
        for key in ("stiffness", "damping", "joint_friction"):
            assert isinstance(body[key], float)
        assert isinstance(body["delay_steps"], int)


def test_the_static_coulomb_measurement_is_what_export_writes(tmp_path, profile):
    """Two measurements of one quantity. The staircase measures it directly —
    a known torque, at rest, at the moment the joint breaks free — and the
    dynamic fit infers it as a regression coefficient over a moving track."""
    width = len(profile.groups["openarm_right_arm"].joints)
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=np.full(width, 50.0),
                damping=np.full(width, 3.0),
                friction=0.31,
                coulomb_nm=np.full(width, 0.08),
            )
        },
    )

    payload = hdgp_calibration(bundle, profile)

    body = payload["groups"]["openarm_right_arm"]
    audit = payload["measured"]["openarm_right_arm"]
    assert body["joint_friction"] == pytest.approx(0.08)
    assert audit["dynamic_friction_nm"] == pytest.approx([0.31] * width)


def test_a_joint_no_staircase_covered_falls_back_to_the_dynamic_coefficient(
    tmp_path, profile
):
    width = len(profile.groups["openarm_right_arm"].joints)
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=np.full(width, 50.0),
                damping=np.full(width, 3.0),
                friction=0.31,
                coulomb_nm=np.full(width, np.nan),
            )
        },
    )

    body = hdgp_calibration(bundle, profile)["groups"]["openarm_right_arm"]

    assert body["joint_friction"] == pytest.approx(0.31)


def test_each_joint_uses_its_own_source_when_the_staircase_only_covered_some(
    tmp_path, profile
):
    width = len(profile.groups["openarm_right_arm"].joints)
    coulomb = np.full(width, np.nan)
    coulomb[0] = 0.08
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=np.full(width, 50.0),
                damping=np.full(width, 3.0),
                friction=0.31,
                coulomb_nm=coulomb,
            )
        },
    )

    audit = hdgp_calibration(bundle, profile)["measured"]["openarm_right_arm"]

    # friction_nm is the per-joint source of the collapsed joint_friction
    # scalar, so it carries the same per-joint choice: measured where the
    # staircase reached the joint, the dynamic coefficient elsewhere.
    assert audit["friction_nm"][0] == pytest.approx(0.08)
    assert audit["friction_nm"][1] == pytest.approx(0.31)


def test_both_bias_measurements_land_in_the_audit_block_unaveraged(tmp_path, profile):
    """Fo (dynamic) and Fo + tau_gravity (static) are not the same number, and
    hdgp has no field for either, so both only ever survive here."""
    width = len(profile.groups["openarm_right_arm"].joints)
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=np.full(width, 50.0),
                damping=np.full(width, 3.0),
                bias=np.full(width, 0.05),
                static_bias_nm=np.full(width, 0.3),
            )
        },
    )

    payload = hdgp_calibration(bundle, profile)
    body = payload["groups"]["openarm_right_arm"]
    audit = payload["measured"]["openarm_right_arm"]

    assert audit["bias_nm"] == pytest.approx([0.05] * width)
    assert audit["static_bias_nm"] == pytest.approx([0.3] * width)
    assert "bias" not in body and "bias_nm" not in body


def test_an_unmeasured_bias_writes_null_on_disk_not_the_nan_token(tmp_path, profile):
    """A joint no staircase covered is expected, not an error, and has to
    reach disk as JSON's own null — the in-memory `hdgp_calibration` return
    value (checked above) keeps nan; only what's written to disk changes."""
    width = len(profile.groups["openarm_right_arm"].joints)
    bundle = _bundle(
        tmp_path,
        profile,
        {
            "openarm_right_arm": _combined(
                profile,
                "openarm_right_arm",
                stiffness=np.full(width, 50.0),
                damping=np.full(width, 3.0),
                # bias (dynamic) measured; static_bias_nm not — the mixed
                # case the reviewer reproduced.
                bias=np.full(width, 0.05),
            )
        },
    )
    hdgp_path = tmp_path / "real2sim.json"

    written = write_hdgp_calibration(hdgp_path, bundle, profile)

    text = hdgp_path.read_text()
    assert "null" in text
    assert "NaN" not in text
    # The function's own return value is untouched by the disk conversion.
    assert np.all(np.isnan(written["measured"]["openarm_right_arm"]["static_bias_nm"]))
    reloaded = json.loads(text)
    static_bias = reloaded["measured"]["openarm_right_arm"]["static_bias_nm"]
    assert static_bias == [None] * width


def test_a_bundle_with_nothing_measured_is_refused(tmp_path, profile):
    bundle = _bundle(tmp_path, profile, {})

    # An empty groups object passes hdgp's loader and applies nothing. Failing
    # here is the difference between "no calibration" and "calibration applied".
    with pytest.raises(HdgpExportError, match="no identified"):
        hdgp_calibration(bundle, profile)


def test_every_profile_group_declares_where_it_lands_in_hdgp(profile):
    # Not a property of any one export: a group that gains a measurement later
    # would fail at export time, which is far from where the name was omitted.
    undeclared = [name for name, g in profile.groups.items() if g.hdgp_group is None]

    assert undeclared == []
