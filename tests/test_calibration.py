import hashlib
import json

import pytest

from robot_control.calibration import (
    CalibrationError,
    load_bundle,
    write_bundle,
)
from robot_control.profile import load_builtin_profile


def _v2(profile):
    groups = {}
    for name in profile.groups:
        groups[name] = {
            "nominal": {
                "stiffness": 50.0,
                "damping": 2.0,
                "friction": 0.1,
                "effort_limit": 10.0,
                "velocity_limit": 2.0,
            },
            "dr": {
                "stiffness": [45.0, 55.0],
                "damping": [1.8, 2.2],
                "friction": [0.08, 0.12],
            },
        }
    return {
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
            "filter": {"type": "ema", "alpha": 0.4},
        },
        "source": {"track_sha256": "a" * 64, "fit_runs": [1, 2], "holdout_runs": [3]},
        "metrics": {
            "fit": {"position_rmse_rad": 0.01},
            "holdout": {"openarm_rmse_rad": 0.02, "tesollo_rmse_rad": 0.04},
        },
        "groups": groups,
    }


def test_v2_round_trip_has_canonical_checksum(tmp_path):
    profile = load_builtin_profile("openarm_tesollo")
    path = tmp_path / "bundle.json"

    write_bundle(path, _v2(profile), profile)
    loaded = load_bundle(path, profile)
    raw = json.loads(path.read_text())

    assert loaded.schema_version == 2
    assert loaded.controller.delay_steps == 2
    payload = dict(raw)
    checksum = payload.pop("checksum_sha256")
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    assert checksum == hashlib.sha256(canonical).hexdigest()


def test_v2_fails_closed_on_unknown_group(tmp_path):
    profile = load_builtin_profile("openarm_tesollo")
    payload = _v2(profile)
    payload["groups"]["mystery"] = payload["groups"].pop("tesollo_dip")
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(payload))

    with pytest.raises(CalibrationError, match="group coverage mismatch"):
        load_bundle(path, profile)


def test_v1_is_read_compatibly_but_cannot_be_written(tmp_path):
    profile = load_builtin_profile("openarm_tesollo")
    path = tmp_path / "v1.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "robot_asset": profile.asset_id,
                "source_dataset": "legacy.hdf5",
                "groups": {
                    name: {"stiffness": 10, "damping": 1, "joint_friction": 0.2}
                    for name in profile.groups
                },
            }
        )
    )

    loaded = load_bundle(path, profile)
    assert loaded.schema_version == 1
    with pytest.raises(CalibrationError, match="only schema v2"):
        write_bundle(tmp_path / "out.json", json.loads(path.read_text()), profile)
