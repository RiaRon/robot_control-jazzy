from __future__ import annotations

import hashlib
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .identification import CombinedEstimate
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
class IdentifiedParameters:
    """Measured joint parameters, in physical units, with where they came from.

    Distinct from the ``nominal`` block, which is a per-group guess: these are
    per joint and were measured, so they carry the sweeps and the track they were
    measured from and the cross-check they survived.
    """

    joint_names: tuple[str, ...]
    #: J, kg m^2.
    inertia: np.ndarray
    #: b, N.m.s/rad.
    damping: np.ndarray
    #: tau_f, N.m.
    friction: np.ndarray
    #: kp, N.m/rad.
    stiffness: np.ndarray
    #: The factor the URDF's modelled gravity torque was wrong by.
    torque_scale: np.ndarray
    #: How far the two independent routes to the inertia differed.
    inertia_disagreement: np.ndarray
    sweep_sha256: tuple[str, ...]
    track_sha256: str


@dataclass(frozen=True)
class CalibrationBundle:
    schema_version: int
    profile: str
    asset_id: str
    groups: Mapping[str, Mapping[str, Any]]
    controller: ControllerCalibration
    payload: Mapping[str, Any]
    #: Per group, by group name. Empty for v1 and for v2 bundles predating it.
    identified: Mapping[str, IdentifiedParameters] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.identified is None:
            object.__setattr__(self, "identified", {})


#: Every measured array in an identified block, and the floor each must clear.
#: Inertia and stiffness are strict: a joint with neither cannot be simulated at
#: all. Damping and friction may legitimately be zero.
_IDENTIFIED_FIELDS = (
    ("inertia_kg_m2", "inertia", False),
    ("damping_nm_s_per_rad", "damping", True),
    ("friction_nm", "friction", True),
    ("stiffness_nm_per_rad", "stiffness", False),
    ("torque_scale", "torque_scale", True),
    ("inertia_disagreement", "inertia_disagreement", True),
)


def identified_block(
    combined: CombinedEstimate,
    profile: RobotProfile,
    *,
    torque_scale: Sequence[float],
    sweep_sha256: Sequence[str],
    track_sha256: str,
) -> dict[str, Any]:
    """Build the block a bundle carries, so there is one way to produce it."""
    torque_scale = np.asarray(torque_scale, dtype=float)
    if torque_scale.shape != combined.inertia.shape:
        raise CalibrationError("torque_scale must carry one value per joint")
    return {
        "joint_names": list(combined.joint_names),
        "inertia_kg_m2": combined.inertia.tolist(),
        "damping_nm_s_per_rad": combined.damping.tolist(),
        "friction_nm": combined.friction.tolist(),
        "stiffness_nm_per_rad": combined.stiffness.tolist(),
        "torque_scale": torque_scale.tolist(),
        "inertia_disagreement": np.nan_to_num(
            combined.disagreement, nan=0.0
        ).tolist(),
        # What the numbers were measured on, as opposed to what the bundle around
        # them claims to be. The checksum is recomputed on write, so it signs a
        # pasted block just as happily; this is what does not.
        "fitted_against": {
            "profile": profile.name,
            "asset_id": profile.asset_id,
            "manifest_sha256": profile.manifest_sha256,
        },
        "provenance": {
            "sweep_sha256": list(sweep_sha256),
            "track_sha256": str(track_sha256),
        },
    }


def _identified(raw: Any, group_name: str, profile: RobotProfile) -> IdentifiedParameters:
    if not isinstance(raw, dict):
        raise CalibrationError(f"{group_name}: identified must be an object")
    where = f"{group_name} identified"

    fitted = raw.get("fitted_against") or {}
    if (
        fitted.get("profile") != profile.name
        or fitted.get("asset_id") != profile.asset_id
        or fitted.get("manifest_sha256") != profile.manifest_sha256
    ):
        raise CalibrationError(
            f"{where}: parameters were fitted against {fitted.get('profile')!r} / "
            f"{fitted.get('asset_id')!r}, not this profile and asset"
        )

    joints = tuple(raw.get("joint_names") or ())
    expected = profile.groups[group_name].joints
    if joints != expected:
        raise CalibrationError(
            f"{where}: joint names {list(joints)} are not group {group_name!r}'s "
            f"{list(expected)}"
        )

    provenance = raw.get("provenance") or {}
    sweeps = provenance.get("sweep_sha256") or []
    track = provenance.get("track_sha256")
    if not sweeps or not track:
        raise CalibrationError(
            f"{where}: provenance needs the sweeps and the track it was measured "
            "from; a number with no source is not a measurement"
        )

    values: dict[str, np.ndarray] = {}
    for key, field, allow_zero in _IDENTIFIED_FIELDS:
        array = np.asarray(raw.get(key), dtype=float)
        if array.shape != (len(joints),) or not np.isfinite(array).all():
            raise CalibrationError(
                f"{where}: {key} must carry one finite value per joint, "
                f"{len(joints)} of them"
            )
        floor = 0.0 if allow_zero else np.nextafter(0.0, 1.0)
        if np.any(array < floor):
            raise CalibrationError(
                f"{where}: {key} carries {array.min():g}, which is not physical"
            )
        values[field] = array

    return IdentifiedParameters(
        joint_names=joints,
        sweep_sha256=tuple(str(item) for item in sweeps),
        track_sha256=str(track),
        **values,
    )


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
    identified = {
        name: _identified(body["identified"], name, profile)
        for name, body in groups.items()
        if isinstance(body, dict) and "identified" in body
    }
    return CalibrationBundle(
        2, profile.name, profile.asset_id, groups, controller, payload, identified
    )


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
