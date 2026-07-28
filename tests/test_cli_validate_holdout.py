"""`robotctl r2s validate` scoring a held-out run instead of reading a file."""

import json

import numpy as np
import pytest

from robot_control.calibration import write_bundle
from robot_control.cli import main
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"
RATE = 100.0
STEP = 1.0 / RATE
SAMPLES = 1200
KP = np.linspace(90.0, 40.0, 7)
DAMPING = np.linspace(8.0, 3.0, 7)


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


@pytest.fixture
def bundle(profile, tmp_path):
    groups = {
        name: {"nominal": {"stiffness": 50.0, "damping": 2.0, "friction": 0.1}}
        for name in profile.groups
    }
    path = tmp_path / "bundle.json"
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
                "command_period_sec": STEP,
                "delay_sec": 0.02,
                "interpolation": "linear",
                "filter": {"type": "none"},
            },
            "groups": groups,
        },
        profile,
    )
    return path


def _simulate(seed):
    """A second-order arm with no standing load, at the profile's rate."""
    rng = np.random.default_rng(seed)
    clock = np.arange(SAMPLES) * STEP
    command = np.column_stack(
        [
            0.15 * np.sin(2 * np.pi * (0.5 + 0.3 * j) * clock + rng.uniform(0, 1))
            for j in range(7)
        ]
    )
    measured = np.zeros((SAMPLES, 7))
    position, velocity = np.zeros(7), np.zeros(7)
    for index in range(SAMPLES):
        measured[index] = position
        acceleration = KP * (command[index] - position) - DAMPING * velocity
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    return clock, command, measured


def _write_runs(tmp_path, profile):
    """Three recordings and a manifest, as collect --repetitions 3 leaves them."""
    joints = list(profile.groups[GROUP].joints)
    names = []
    for index in range(3):
        clock, command, measured = _simulate(index)
        stamps = (clock * 1e9).astype(np.int64)
        name = f"run{index}.npz"
        np.savez(
            tmp_path / name,
            command_time_ns=stamps,
            command=command,
            measured_time_ns=stamps,
            measured=measured,
            joint_names=np.array(joints),
            profile=np.array(profile.name),
            asset_id=np.array(profile.asset_id),
            manifest_sha256=np.array(profile.manifest_sha256),
            incomplete=np.array(0),
        )
        names.append(name)
    manifest = tmp_path / "run.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "kind": "excitation_runs",
                "profile": profile.name,
                "asset": {
                    "id": profile.asset_id,
                    "manifest_sha256": profile.manifest_sha256,
                },
                "group": GROUP,
                "runs": [{"path": name} for name in names],
                "fit_runs": [0, 1],
                "holdout_runs": [2],
            },
            indent=2,
        )
    )
    return manifest


def _fit_file(tmp_path, profile, stiffness=None, damping=None):
    """A fit estimate as r2s fit writes one, without the gravity column."""
    path = tmp_path / "estimate.json"
    path.write_text(
        json.dumps(
            {
                "population": 128,
                "joint_names": list(profile.groups[GROUP].joints),
                "stiffness": (KP if stiffness is None else stiffness).tolist(),
                "damping": (DAMPING if damping is None else damping).tolist(),
                "friction": np.zeros(7).tolist(),
                "residual_rmse": [1e-9] * 7,
                "track_sha256": "a" * 64,
            }
        )
    )
    return path


def _fit_payload(profile, **extra):
    return {
        "population": 128,
        "joint_names": list(profile.groups[GROUP].joints),
        "stiffness": KP.tolist(),
        "damping": DAMPING.tolist(),
        "friction": np.zeros(7).tolist(),
        "residual_rmse": [1e-9] * 7,
        "track_sha256": "a" * 64,
        **extra,
    }


def test_the_scored_model_carries_the_bias_the_fit_measured(tmp_path, profile):
    """`r2s fit` writes Fo under `bias_nm`, in N.m. What is read back looked
    for a key called `bias`, which nothing has ever written, so the bias was
    always zero and `score_holdout` scored a model that was never fitted.

    The units differ too: the file's `bias_nm` is `combine`'s Fo = Fo/J times
    J, and `SecondOrderEstimate.bias` is Fo/J, so the inertia written beside it
    in the same file is what converts it back. At Fo/J = 5 rad/s^2 dropping the
    term costs 0.0165 rad of RMSE against a 0.03 rad threshold, which is enough
    to move a verdict in either direction.
    """
    from robot_control.cli import _estimate_from_fit

    path = tmp_path / "estimate.json"
    inertia = np.linspace(0.4, 0.1, 7)
    path.write_text(
        json.dumps(
            _fit_payload(
                profile,
                inertia_kg_m2=inertia.tolist(),
                bias_nm=(5.0 * inertia).tolist(),
            )
        )
    )

    estimate = _estimate_from_fit(path, profile.groups[GROUP])

    np.testing.assert_allclose(estimate.bias, np.full(7, 5.0))


def test_a_fit_file_from_before_the_bias_was_written_scores_without_one(
    tmp_path, profile
):
    """Zero is what an absent bias term means; it is not a stand-in for a
    measurement the file is missing."""
    from robot_control.cli import _estimate_from_fit

    path = tmp_path / "estimate.json"
    path.write_text(json.dumps(_fit_payload(profile)))

    estimate = _estimate_from_fit(path, profile.groups[GROUP])

    np.testing.assert_array_equal(estimate.bias, np.zeros(7))


def test_a_bias_with_no_inertia_beside_it_is_refused(tmp_path, profile):
    """Fo in N.m cannot be turned into Fo/J without J. Guessing one would put
    a number of the wrong size into the model rather than say so."""
    from robot_control.cli import _estimate_from_fit

    path = tmp_path / "estimate.json"
    path.write_text(json.dumps(_fit_payload(profile, bias_nm=[0.5] * 7)))

    with pytest.raises(ValueError, match="inertia"):
        _estimate_from_fit(path, profile.groups[GROUP])


def test_validate_scores_the_holdout_without_a_metrics_file(
    bundle, tmp_path, profile, capsys
):
    manifest = _write_runs(tmp_path, profile)
    verdict = tmp_path / "verdict.json"

    code = main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--fit", str(_fit_file(tmp_path, profile)), "--output", str(verdict)]
    )

    assert code == 0, capsys.readouterr().out
    payload = json.loads(verdict.read_text())
    assert payload["status"] == "validated"
    assert payload["metrics"]["openarm_rmse_rad"] < 0.03
    assert payload["metrics"]["improvement_fraction"] > 0.3
    assert payload["holdout_runs"] == ["run2.npz"]


def test_a_model_that_does_not_predict_the_holdout_fails(
    bundle, tmp_path, profile, capsys
):
    manifest = _write_runs(tmp_path, profile)
    wrong = _fit_file(tmp_path, profile, stiffness=KP * 0.2, damping=DAMPING * 4)
    verdict = tmp_path / "verdict.json"

    code = main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--fit", str(wrong), "--output", str(verdict)]
    )

    assert code == 3
    payload = json.loads(verdict.read_text())
    assert payload["status"] == "model_inadequate"
    assert "openarm_rmse" in payload["failures"]


def test_the_verdict_names_the_run_it_was_scored_on(bundle, tmp_path, profile):
    manifest = _write_runs(tmp_path, profile)
    verdict = tmp_path / "verdict.json"

    main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--fit", str(_fit_file(tmp_path, profile)), "--output", str(verdict)]
    )

    payload = json.loads(verdict.read_text())
    assert payload["group"] == GROUP
    assert payload["fit_runs"] == ["run0.npz", "run1.npz"]


def test_a_hand_written_metrics_file_still_works(bundle, tmp_path, capsys):
    """Kept: a verdict computed elsewhere is still a verdict."""
    metrics = tmp_path / "metrics.json"
    metrics.write_text(
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

    code = main(
        ["r2s", "validate", "--bundle", str(bundle), "--metrics", str(metrics),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 0


def test_validate_needs_metrics_or_a_manifest(bundle, tmp_path, capsys):
    code = main(
        ["r2s", "validate", "--bundle", str(bundle),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 2
    out = capsys.readouterr().out
    assert "--metrics" in out and "--manifest" in out


def test_a_manifest_needs_a_fit_to_score(bundle, tmp_path, profile, capsys):
    manifest = _write_runs(tmp_path, profile)

    code = main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 2
    assert "--fit" in capsys.readouterr().out


def test_a_manifest_from_another_asset_is_refused(bundle, tmp_path, profile, capsys):
    manifest = _write_runs(tmp_path, profile)
    payload = json.loads(manifest.read_text())
    payload["asset"]["manifest_sha256"] = "0" * 64
    manifest.write_text(json.dumps(payload))

    code = main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--fit", str(_fit_file(tmp_path, profile)),
         "--output", str(tmp_path / "verdict.json")]
    )

    assert code == 2
    assert "asset" in capsys.readouterr().out


def test_the_metric_is_named_after_the_group_s_component(
    bundle, tmp_path, profile, capsys
):
    """openarm_rmse_rad and tesollo_rmse_rad have different thresholds, so which
    one a run fills has to come from the group rather than be assumed."""
    manifest = _write_runs(tmp_path, profile)
    verdict = tmp_path / "verdict.json"

    main(
        ["r2s", "validate", "--bundle", str(bundle), "--manifest", str(manifest),
         "--fit", str(_fit_file(tmp_path, profile)), "--output", str(verdict)]
    )

    metrics = json.loads(verdict.read_text())["metrics"]
    # An arm run says nothing about the hand, so the hand's metric is left at a
    # value that cannot fail rather than invented.
    assert metrics["openarm_rmse_rad"] > 0
    assert metrics["tesollo_rmse_rad"] == 0.0
