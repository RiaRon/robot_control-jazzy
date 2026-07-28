"""`robotctl r2s bundle`, and what validate and export then see."""

import json

import numpy as np
import pytest

from robot_control.calibration import write_bundle
from robot_control.cli import main
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


@pytest.fixture
def base(profile, tmp_path):
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
    path = tmp_path / "base.json"
    write_bundle(
        path,
        {
            "schema_version": 2,
            "profile": profile.name,
            "asset": {
                "id": profile.asset_id,
                "manifest_sha256": profile.manifest_sha256,
            },
            "controller": {
                "command_period_sec": 0.01,
                "delay_sec": 0.02,
                "interpolation": "linear",
                "filter": {"type": "none"},
            },
            "groups": groups,
        },
        profile,
    )
    return path


def _fit(path, group=GROUP, **overrides):
    known = load_builtin_profile("openarm_tesollo").groups
    payload = {
        "population": 128,
        "group": group,
        "joint_names": list(known.get(group, known[GROUP]).joints),
        "stiffness": np.linspace(57.0, 160.0, 7).tolist(),
        "damping": np.linspace(4.3, 8.0, 7).tolist(),
        "friction": np.linspace(1.1, 2.0, 7).tolist(),
        "residual_rmse": [1e-3] * 7,
        "track_sha256": "c" * 64,
        "stiffness_nm_per_rad": np.linspace(20.0, 8.0, 7).tolist(),
        "inertia_kg_m2": np.linspace(0.35, 0.05, 7).tolist(),
        "damping_nm_s_per_rad": np.linspace(1.5, 0.4, 7).tolist(),
        "friction_nm": np.linspace(0.4, 0.1, 7).tolist(),
        "inertia_from_gravity_kg_m2": np.linspace(0.355, 0.051, 7).tolist(),
        "inertia_disagreement": [0.01] * 7,
        "torque_scale": [1.05] * 7,
        "sweep_sha256": ["a" * 64, "b" * 64],
    }
    payload.update(overrides)
    path.write_text(json.dumps(payload))
    return path


def test_bundle_merges_the_parameters_and_their_provenance(base, tmp_path, capsys):
    output = tmp_path / "bundled.json"

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(output),
         "--fit", str(_fit(tmp_path / "right.json"))]
    )

    assert code == 0, capsys.readouterr().out
    identified = json.loads(output.read_text())["groups"][GROUP]["identified"]
    np.testing.assert_allclose(identified["inertia_kg_m2"], np.linspace(0.35, 0.05, 7))
    assert identified["provenance"]["sweep_sha256"] == ["a" * 64, "b" * 64]
    assert identified["provenance"]["track_sha256"] == "c" * 64
    assert GROUP in capsys.readouterr().out


def test_bundle_carries_the_staircase_measurements_through(base, tmp_path, capsys):
    output = tmp_path / "bundled.json"
    fit = _fit(
        tmp_path / "right.json",
        bias_nm=np.linspace(0.05, 0.02, 7).tolist(),
        coulomb_nm=np.linspace(0.08, 0.05, 7).tolist(),
        static_bias_nm=np.linspace(0.3, 0.1, 7).tolist(),
    )

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(output),
         "--fit", str(fit)]
    )

    assert code == 0, capsys.readouterr().out
    identified = json.loads(output.read_text())["groups"][GROUP]["identified"]
    np.testing.assert_allclose(identified["bias_nm"], np.linspace(0.05, 0.02, 7))
    np.testing.assert_allclose(identified["coulomb_nm"], np.linspace(0.08, 0.05, 7))
    np.testing.assert_allclose(identified["static_bias_nm"], np.linspace(0.3, 0.1, 7))


def test_bundle_from_a_fit_predating_the_staircase_fields_still_works(
    base, tmp_path, capsys
):
    """Additive: a fit file written before Fc and Fo existed bundles the same
    as it always did, with nothing fabricated for the fields it never had."""
    output = tmp_path / "bundled.json"

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(output),
         "--fit", str(_fit(tmp_path / "right.json"))]
    )

    assert code == 0, capsys.readouterr().out
    identified = json.loads(output.read_text())["groups"][GROUP]["identified"]
    assert "bias_nm" not in identified
    assert "coulomb_nm" not in identified
    assert "static_bias_nm" not in identified


def test_bundle_takes_one_fit_per_group(base, tmp_path, capsys):
    output = tmp_path / "bundled.json"

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(output),
         "--fit", str(_fit(tmp_path / "right.json")),
         "--fit", str(_fit(
             tmp_path / "left.json",
             group="openarm_left_arm",
             joint_names=list(load_builtin_profile("openarm_tesollo")
                              .groups["openarm_left_arm"].joints),
         ))]
    )

    assert code == 0, capsys.readouterr().out
    groups = json.loads(output.read_text())["groups"]
    assert "identified" in groups[GROUP]
    assert "identified" in groups["openarm_left_arm"]
    assert "identified" not in groups["tesollo_curl"]


def test_bundle_refuses_a_fit_that_ran_without_static(base, tmp_path, capsys):
    """Its stiffness is a ratio to an inertia, not a stiffness."""
    ratios = _fit(tmp_path / "ratios.json")
    payload = json.loads(ratios.read_text())
    del payload["inertia_kg_m2"]
    ratios.write_text(json.dumps(payload))

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(tmp_path / "out.json"),
         "--fit", str(ratios)]
    )

    assert code == 2
    assert "--static" in capsys.readouterr().out


def test_bundle_refuses_a_fit_for_a_group_the_bundle_lacks(base, tmp_path, capsys):
    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(tmp_path / "out.json"),
         "--fit", str(_fit(tmp_path / "odd.json", group="no_such_group"))]
    )

    assert code == 2
    assert "no_such_group" in capsys.readouterr().out


def test_bundle_refuses_an_impossible_parameter(base, tmp_path, capsys):
    bad = _fit(tmp_path / "bad.json", inertia_kg_m2=[0.0] * 7)

    code = main(
        ["r2s", "bundle", "--base", str(base), "--output", str(tmp_path / "out.json"),
         "--fit", str(bad)]
    )

    assert code == 3
    assert "inertia_kg_m2" in capsys.readouterr().out


def test_bundle_needs_its_three_paths(base, tmp_path, capsys):
    code = main(["r2s", "bundle", "--base", str(base)])

    assert code == 2
    assert "--fit" in capsys.readouterr().out


def _metrics(path):
    path.write_text(
        json.dumps(
            {
                "openarm_rmse_rad": 0.02,
                "tesollo_rmse_rad": 0.04,
                "delay_residual_sec": 0.005,
                "command_period_sec": 0.01,
                "improvement_fraction": 0.5,
            }
        )
    )
    return path


def test_validate_reports_which_groups_carry_parameters(base, tmp_path, capsys):
    bundled = tmp_path / "bundled.json"
    main(["r2s", "bundle", "--base", str(base), "--output", str(bundled),
          "--fit", str(_fit(tmp_path / "right.json"))])
    capsys.readouterr()

    code = main(
        ["r2s", "validate", "--bundle", str(bundled),
         "--metrics", str(_metrics(tmp_path / "m.json")),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 0
    out = capsys.readouterr().out
    assert "identified parameters" in out
    assert GROUP in out


def test_validate_reports_both_friction_measurements(base, tmp_path, capsys):
    """A large disagreement is evidence about the model rather than about the
    arm, and preferring one silently would hide it."""
    bundled = tmp_path / "bundled.json"
    main(
        ["r2s", "bundle", "--base", str(base), "--output", str(bundled),
         "--fit", str(_fit(tmp_path / "right.json", coulomb_nm=[0.08] * 7))]
    )
    capsys.readouterr()

    code = main(
        ["r2s", "validate", "--bundle", str(bundled),
         "--metrics", str(_metrics(tmp_path / "m.json")),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 0
    out = capsys.readouterr().out
    assert "coulomb" in out and "dynamic_friction" in out
    assert "0.0800" in out


def test_an_exported_bundle_carries_the_parameters(base, tmp_path, capsys):
    bundled = tmp_path / "bundled.json"
    main(["r2s", "bundle", "--base", str(base), "--output", str(bundled),
          "--fit", str(_fit(tmp_path / "right.json"))])
    verdict = tmp_path / "verdict.json"
    verdict.write_text(json.dumps({"status": "validated", "failures": []}))
    exported = tmp_path / "exported.json"

    code = main(
        ["r2s", "export", "--bundle", str(bundled),
         "--validation", str(verdict), "--output", str(exported)]
    )

    assert code == 0
    identified = json.loads(exported.read_text())["groups"][GROUP]["identified"]
    assert identified["provenance"]["track_sha256"] == "c" * 64
    assert identified["fitted_against"]["asset_id"] == (
        load_builtin_profile("openarm_tesollo").asset_id
    )


def test_validate_refuses_parameters_measured_on_another_asset(base, tmp_path, capsys):
    """The bundle's own header is right; the block inside it is from elsewhere.

    Re-signed after the edit, because the checksum signs whatever is there and so
    cannot be the check that catches this.
    """
    import hashlib

    bundled = tmp_path / "bundled.json"
    main(["r2s", "bundle", "--base", str(base), "--output", str(bundled),
          "--fit", str(_fit(tmp_path / "right.json"))])
    payload = json.loads(bundled.read_text())
    payload["groups"][GROUP]["identified"]["fitted_against"]["manifest_sha256"] = "0" * 64
    payload.pop("checksum_sha256")
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    payload["checksum_sha256"] = hashlib.sha256(canonical).hexdigest()
    bundled.write_text(json.dumps(payload))
    capsys.readouterr()

    code = main(["r2s", "validate", "--bundle", str(bundled),
                 "--metrics", str(_metrics(tmp_path / "m.json")),
                 "--output", str(tmp_path / "verdict.json")])

    assert code == 2
    assert "fitted against" in capsys.readouterr().out


def _validated_bundle(base, tmp_path):
    bundled = tmp_path / "bundled.json"
    main(["r2s", "bundle", "--base", str(base), "--output", str(bundled),
          "--fit", str(_fit(tmp_path / "right.json"))])
    verdict = tmp_path / "verdict.json"
    verdict.write_text(json.dumps({"status": "validated", "failures": []}))
    return bundled, verdict


def test_export_writes_the_file_hdgps_env_loads(base, tmp_path, capsys):
    bundled, verdict = _validated_bundle(base, tmp_path)
    hdgp = tmp_path / "real2sim.json"

    code = main(
        ["r2s", "export", "--bundle", str(bundled), "--validation", str(verdict),
         "--output", str(tmp_path / "exported.json"), "--hdgp", str(hdgp),
         # The seven arm joints genuinely disagree by more than the default
         # allows; accepting the average is the caller's call to make.
         "--hdgp-max-spread", "2.0"]
    )

    assert code == 0
    payload = json.loads(hdgp.read_text())
    assert payload["schema_version"] == 1
    assert set(payload["groups"]) == {GROUP}
    out = capsys.readouterr().out
    assert "export hdgp" in out
    # The groups that keep the env's own gain are named, not left to be noticed.
    assert "tesollo_hand_curl" in out


def test_export_refuses_to_average_joints_that_disagree(base, tmp_path, capsys):
    bundled, verdict = _validated_bundle(base, tmp_path)
    hdgp = tmp_path / "real2sim.json"

    code = main(
        ["r2s", "export", "--bundle", str(bundled), "--validation", str(verdict),
         "--output", str(tmp_path / "exported.json"), "--hdgp", str(hdgp)]
    )

    assert code == 2
    assert not hdgp.exists()
    assert "one scalar describes no joint" in capsys.readouterr().out


def test_export_refuses_a_bundle_it_cannot_read(base, tmp_path, capsys):
    verdict = tmp_path / "verdict.json"
    verdict.write_text(json.dumps({"status": "validated", "failures": []}))
    broken = tmp_path / "broken.json"
    payload = json.loads(base.read_text())
    payload["asset"]["manifest_sha256"] = "0" * 64
    broken.write_text(json.dumps(payload))

    code = main(["r2s", "export", "--bundle", str(broken),
                 "--validation", str(verdict), "--output", str(tmp_path / "out.json")])

    assert code == 2
    assert "error" in capsys.readouterr().out
