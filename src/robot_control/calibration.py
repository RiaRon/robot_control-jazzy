from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from .profile import RobotProfile


class CalibrationError(ValueError):
    pass


@dataclass(frozen=True)
class ControllerCalibration:
    command_period_sec: float
    delay_sec: float
    delay_steps: int
    interpolation: str
    filter: Mapping[str, Any]


@dataclass(frozen=True)
class CalibrationBundle:
    schema_version: int
    profile: str
    asset_id: str
    groups: Mapping[str, Mapping[str, Any]]
    controller: ControllerCalibration
    payload: Mapping[str, Any]


def _canonical_bytes(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()


def _validate_groups(raw: Mapping[str, Any], profile: RobotProfile) -> None:
    expected = set(profile.groups)
    actual = set(raw)
    if actual != expected:
        raise CalibrationError(
            f"group coverage mismatch: missing={sorted(expected-actual)}, unknown={sorted(actual-expected)}"
        )


def load_bundle(path: str | Path, profile: RobotProfile) -> CalibrationBundle:
    payload = json.loads(Path(path).read_text())
    version = int(payload.get("schema_version", 0))
    groups = payload.get("groups")
    if not isinstance(groups, dict) or not groups:
        raise CalibrationError("groups must be a non-empty object")
    _validate_groups(groups, profile)

    if version == 1:
        if payload.get("robot_asset") != profile.asset_id:
            raise CalibrationError("asset mismatch")
        normalized = {
            name: {
                "nominal": {
                    "stiffness": float(body["stiffness"]),
                    "damping": float(body["damping"]),
                    "friction": float(body.get("joint_friction", 0.0)),
                }
            }
            for name, body in groups.items()
        }
        controller = ControllerCalibration(0.0, 0.0, 0, "none", {"type": "none"})
        return CalibrationBundle(1, profile.name, profile.asset_id, normalized, controller, payload)
    if version != 2:
        raise CalibrationError(f"unsupported schema_version: {version}")

    asset = payload.get("asset", {})
    if (
        payload.get("profile") != profile.name
        or asset.get("id") != profile.asset_id
        or asset.get("manifest_sha256") != profile.manifest_sha256
    ):
        raise CalibrationError("profile or asset manifest mismatch")
    checksum = payload.get("checksum_sha256")
    if checksum:
        unsigned = dict(payload)
        unsigned.pop("checksum_sha256")
        actual = hashlib.sha256(_canonical_bytes(unsigned)).hexdigest()
        if checksum != actual:
            raise CalibrationError("bundle checksum mismatch")
    control = payload.get("controller", {})
    period = float(control.get("command_period_sec", 0.0))
    delay = float(control.get("delay_sec", 0.0))
    if period <= 0 or delay < 0:
        raise CalibrationError("invalid controller timing")
    controller = ControllerCalibration(
        period,
        delay,
        int(round(delay / period)),
        str(control.get("interpolation", "none")),
        control.get("filter", {"type": "none"}),
    )
    return CalibrationBundle(2, profile.name, profile.asset_id, groups, controller, payload)


def write_bundle(
    path: str | Path, payload: Mapping[str, Any], profile: RobotProfile
) -> CalibrationBundle:
    if int(payload.get("schema_version", 0)) != 2:
        raise CalibrationError("only schema v2 may be written")
    unsigned = dict(payload)
    unsigned.pop("checksum_sha256", None)
    out = dict(unsigned)
    out["checksum_sha256"] = hashlib.sha256(_canonical_bytes(unsigned)).hexdigest()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    return load_bundle(path, profile)
