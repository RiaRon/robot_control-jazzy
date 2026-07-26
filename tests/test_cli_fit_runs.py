"""`r2s fit --manifest`: fit across the two runs the manifest names.

`split_repetitions` returns two runs to fit and one to hold out, and the bundle's
`source` block has fields for both. Fitting a single `--track` could never fill
them honestly.
"""

import json

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.profile import load_builtin_profile

from test_cli_validate_holdout import (  # noqa: F401
    DAMPING,
    KP,
    GROUP,
    _write_runs,
    bundle,
    profile,
)


def test_fit_uses_both_runs_the_manifest_names(tmp_path, profile, capsys):
    manifest = _write_runs(tmp_path, profile)
    output = tmp_path / "estimate.json"

    code = main(
        ["r2s", "fit", "--manifest", str(manifest), "--output", str(output)]
    )

    assert code == 0, capsys.readouterr().out
    payload = json.loads(output.read_text())
    np.testing.assert_allclose(payload["stiffness"], KP, rtol=1e-3)
    np.testing.assert_allclose(payload["damping"], DAMPING, rtol=1e-2)


def test_the_estimate_cites_the_runs_it_was_fitted_on(tmp_path, profile):
    manifest = _write_runs(tmp_path, profile)
    output = tmp_path / "estimate.json"

    main(["r2s", "fit", "--manifest", str(manifest), "--output", str(output)])

    payload = json.loads(output.read_text())
    assert payload["fit_runs"] == ["run0.npz", "run1.npz"]
    assert payload["holdout_runs"] == ["run2.npz"]
    assert payload["group"] == GROUP


def test_the_holdout_run_is_not_fitted_on(tmp_path, profile):
    """Otherwise validating against it would validate nothing."""
    manifest = _write_runs(tmp_path, profile)
    payload = json.loads(manifest.read_text())
    payload["fit_runs"] = [0, 1, 2]
    manifest.write_text(json.dumps(payload))

    code = main(
        ["r2s", "fit", "--manifest", str(manifest),
         "--output", str(tmp_path / "estimate.json")]
    )

    assert code == 2


def test_fit_needs_a_track_or_a_manifest(tmp_path, capsys):
    code = main(["r2s", "fit", "--output", str(tmp_path / "estimate.json")])

    assert code == 2
    out = capsys.readouterr().out
    assert "--track" in out and "--manifest" in out


def test_a_manifest_from_another_asset_is_refused(tmp_path, profile, capsys):
    manifest = _write_runs(tmp_path, profile)
    payload = json.loads(manifest.read_text())
    payload["profile"] = "someone_else"
    manifest.write_text(json.dumps(payload))

    code = main(
        ["r2s", "fit", "--manifest", str(manifest),
         "--output", str(tmp_path / "estimate.json")]
    )

    assert code == 2
    assert "profile" in capsys.readouterr().out


def test_the_bundle_carries_the_runs_the_fit_cited(tmp_path, profile, bundle):
    from robot_control.artifacts import write_static_estimate
    from robot_control.identification import StaticEstimate

    manifest = _write_runs(tmp_path, profile)
    estimate = tmp_path / "estimate.json"
    main(["r2s", "fit", "--manifest", str(manifest), "--output", str(estimate)])

    # A fit with no --static carries ratios, which r2s bundle refuses, so add
    # the physical block the way fit --static would.
    payload = json.loads(estimate.read_text())
    payload.update(
        {
            "stiffness_nm_per_rad": KP.tolist(),
            "inertia_kg_m2": np.full(7, 1.0).tolist(),
            "damping_nm_s_per_rad": DAMPING.tolist(),
            "friction_nm": np.zeros(7).tolist(),
            "inertia_from_gravity_kg_m2": np.full(7, 1.0).tolist(),
            "inertia_disagreement": [0.0] * 7,
            "torque_scale": [1.0] * 7,
            "sweep_sha256": ["a" * 64],
        }
    )
    estimate.write_text(json.dumps(payload))
    output = tmp_path / "bundled.json"

    code = main(
        ["r2s", "bundle", "--base", str(bundle), "--fit", str(estimate),
         "--output", str(output)]
    )

    assert code == 0
    source = json.loads(output.read_text())["source"]
    assert source["fit_runs"] == ["run0.npz", "run1.npz"]
    assert source["holdout_runs"] == ["run2.npz"]
