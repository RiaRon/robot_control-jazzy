from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .identification import FitError, GravitySweep, StaticEstimate  # noqa: F401
from .profile import RobotProfile
from .track import CanonicalTrack


class ArtifactError(RuntimeError):
    pass


#: Bumped when a payload's meaning changes, not when a field is added that older
#: readers can ignore. 2: applied_torque joined the round payload; a version 1
#: reader cannot skip it, so this is a meaning change rather than an addition.
SWEEP_SCHEMA_VERSION = 2
#: Bumped when a payload's meaning changes, not when a field is added that older
#: readers can ignore — there are none, so it never has been.
STATIC_SCHEMA_VERSION = 1

SWEEP_KIND = "gravity_sweep"
STATIC_KIND = "static_gravity_estimate"


def track_sha256(track: CanonicalTrack) -> str:
    digest = hashlib.sha256()
    digest.update(track.timestamps_ns.astype("<i8").tobytes())
    digest.update(track.command.astype("<f8").tobytes())
    digest.update(track.measured.astype("<f8").tobytes())
    digest.update(json.dumps(track.joint_names, separators=(",", ":")).encode())
    return digest.hexdigest()


def sweep_sha256(sweep: GravitySweep) -> str:
    """A digest of what a sweep measured, for citing it as a fit's source."""
    digest = hashlib.sha256()
    for grid in (sweep.poses, sweep.modelled_torque, sweep.scales, sweep.errors):
        digest.update(np.ascontiguousarray(grid, dtype="<f8").tobytes())
    digest.update(
        json.dumps(
            [sweep.group, list(sweep.joint_names), sweep.sweep_joint],
            separators=(",", ":"),
        ).encode()
    )
    return digest.hexdigest()


def _canonical_bytes(payload: Mapping[str, Any]) -> bytes:
    # allow_nan=False on purpose: a nan reaching a file would read back as a
    # number in Python and as a syntax error everywhere else.
    return json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()


def nan_to_null(value: Any) -> Any:
    """Recursively replace a float nan with None, everywhere it appears.

    "Not measured" is a legitimate value for several fields elsewhere in this
    package (a joint no staircase covered, a bias term a fit predates), and it
    has to reach disk as JSON's own `null` rather than the non-standard `NaN`
    token plain `json.dumps` writes for a float nan by default — see
    `_canonical_bytes` above for why a nan must never reach a `json.dumps`
    call in the first place. `null` round-trips back to nan on its own:
    `np.asarray([..., None, ...], dtype=float)` already turns a `None` entry
    into nan, so no matching "null_to_nan" reader is needed.

    Returns a new structure — lists and dicts are rebuilt, never mutated — so
    a caller that keeps the original around still sees nan, not None.
    """
    if isinstance(value, float) and math.isnan(value):
        return None
    if isinstance(value, (list, tuple)):
        return [nan_to_null(item) for item in value]
    if isinstance(value, dict):
        return {key: nan_to_null(item) for key, item in value.items()}
    return value


def _header(kind: str, version: int, profile: RobotProfile) -> dict[str, Any]:
    """The provenance every measured artifact carries.

    The profile name, the asset id and the manifest hash go in the file rather
    than being supplied at read time, because these are numbers about one
    physical arm: read against a different one they are not merely stale, they
    are wrong in a way nothing downstream could detect.
    """
    return {
        "schema_version": version,
        "kind": kind,
        "profile": profile.name,
        "asset": {
            "id": profile.asset_id,
            "manifest_sha256": profile.manifest_sha256,
        },
    }


def _sign_and_write(path: str | Path, payload: dict[str, Any]) -> None:
    payload["checksum_sha256"] = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    )


def _verify(
    path: str | Path,
    profile: RobotProfile,
    kind: str,
    version: int,
    *,
    minimum: int | None = None,
) -> dict[str, Any]:
    """Read a signed artifact, refusing anything it was not measured against.

    Unlike a calibration bundle, an unsigned file is refused outright rather
    than trusted: this tool is the only thing that writes these. ``minimum``
    lets a reader accept older schemas it knows how to reconstruct; a writer
    only ever calls with the exact version, so every other caller is
    unaffected.
    """
    payload = json.loads(Path(path).read_text())
    if not isinstance(payload, dict):
        raise ArtifactError(f"{kind} is not an object")
    if payload.get("kind") != kind:
        raise ArtifactError(f"expected a {kind}, found {payload.get('kind')!r}")
    found = payload.get("schema_version")
    floor = version if minimum is None else minimum
    if not isinstance(found, int) or not floor <= found <= version:
        raise ArtifactError(
            f"unsupported {kind} schema_version: {found!r}, expected between "
            f"{floor} and {version}"
        )

    checksum = payload.get("checksum_sha256")
    unsigned = {key: value for key, value in payload.items() if key != "checksum_sha256"}
    if checksum != hashlib.sha256(_canonical_bytes(unsigned)).hexdigest():
        raise ArtifactError(f"{kind} checksum mismatch")

    asset = payload.get("asset") or {}
    if (
        payload.get("profile") != profile.name
        or asset.get("id") != profile.asset_id
        or asset.get("manifest_sha256") != profile.manifest_sha256
    ):
        raise ArtifactError("profile or asset manifest mismatch")
    return payload


def _group_joints(payload: Mapping[str, Any], profile: RobotProfile, kind: str):
    name = payload.get("group")
    group = profile.groups.get(name)
    if group is None:
        raise ArtifactError(
            f"{kind} names group {name!r}, which this profile does not have"
        )
    names = tuple(payload.get("joint_names") or ())
    if names != group.joints:
        raise ArtifactError(
            f"{kind} joint names do not match group {name!r}: {list(names)} "
            f"against {list(group.joints)}"
        )
    return name, names


def write_sweep(path: str | Path, sweep: GravitySweep, profile: RobotProfile) -> None:
    payload = _header(SWEEP_KIND, SWEEP_SCHEMA_VERSION, profile)
    payload.update(
        {
            "group": sweep.group,
            "joint_names": list(sweep.joint_names),
            "sweep_joint": sweep.sweep_joint,
            "rounds": [
                {
                    "pose": sweep.poses[index].tolist(),
                    "modelled_torque": sweep.modelled_torque[index].tolist(),
                    "scale": sweep.scales[index].tolist(),
                    "applied_torque": sweep.applied_torque[index].tolist(),
                    "error": sweep.errors[index].tolist(),
                }
                for index in range(sweep.rounds)
            ],
        }
    )
    _sign_and_write(path, payload)


def read_sweep(path: str | Path, profile: RobotProfile) -> GravitySweep:
    payload = _verify(path, profile, SWEEP_KIND, SWEEP_SCHEMA_VERSION, minimum=1)
    name, names = _group_joints(payload, profile, SWEEP_KIND)
    rounds = payload.get("rounds") or []
    try:
        scales = np.array([entry["scale"] for entry in rounds], dtype=float)
        modelled = np.array(
            [entry["modelled_torque"] for entry in rounds], dtype=float
        )
        # A version 1 file predates the field. Its excitation could only ever
        # publish a multiple of the model, so that product is not a guess.
        applied = (
            np.array([entry["applied_torque"] for entry in rounds], dtype=float)
            if rounds and "applied_torque" in rounds[0]
            else scales * modelled
        )
        return GravitySweep(
            group=name,
            joint_names=names,
            poses=np.array([entry["pose"] for entry in rounds], dtype=float),
            modelled_torque=modelled,
            scales=scales,
            applied_torque=applied,
            errors=np.array([entry["error"] for entry in rounds], dtype=float),
            sweep_joint=payload.get("sweep_joint"),
        )
    except (FitError, KeyError, TypeError, ValueError) as error:
        raise ArtifactError(f"malformed sweep: {error}") from error


def write_static_estimate(
    path: str | Path,
    estimate: StaticEstimate,
    profile: RobotProfile,
    *,
    group: str,
    noise_rad: float,
    sources: Sequence[str],
) -> None:
    """Write an identified stiffness set, citing the sweeps it came from.

    Only a complete estimate is written. A file with a hole in it would read
    back as an authoritative answer for the joints it does have, and Task 4's
    inertia is a product of the two fits: a missing stiffness there becomes a
    missing inertia, silently.
    """
    if estimate.unidentifiable:
        raise ArtifactError(
            "refusing to write an incomplete estimate; unidentified: "
            + ", ".join(name for name, _reason in estimate.unidentifiable)
        )
    # A joint can clear the check above (its stiffness fit succeeded) with
    # torque_scale still nan: its gravity model was zero at every used round,
    # so alpha was never determined. `unidentifiable` does not see this hole —
    # by the definitions above the joint *was* identified — but `_fit`'s
    # gravity column would still read the nan back as a number.
    undetermined_alpha = [
        name
        for index, name in enumerate(estimate.joint_names)
        if not np.isfinite(estimate.torque_scale[index])
    ]
    if undetermined_alpha:
        raise ArtifactError(
            "refusing to write an incomplete estimate; torque_scale "
            "undetermined (gravity model was zero at every used round): "
            + ", ".join(undetermined_alpha)
        )
    payload = _header(STATIC_KIND, STATIC_SCHEMA_VERSION, profile)
    payload.update(
        {
            "group": group,
            "joint_names": list(estimate.joint_names),
            "noise_rad": float(noise_rad),
            "sources": {"sweep_sha256": list(sources)},
            "stiffness_nm_per_rad": estimate.stiffness.tolist(),
            "torque_scale": estimate.torque_scale.tolist(),
            "offset_rad": estimate.offset.tolist(),
            "residual_rmse_rad": estimate.residual_rmse.tolist(),
            "condition": estimate.condition.tolist(),
            "rounds_used": estimate.used.tolist(),
            "rounds_excluded": estimate.excluded.tolist(),
        }
    )
    _sign_and_write(path, payload)


@dataclass(frozen=True)
class StaticArtifact:
    """A static estimate as it was stored: the numbers plus their provenance."""

    group: str
    estimate: StaticEstimate
    noise_rad: float
    sweep_sha256: tuple[str, ...]


def read_static_estimate(path: str | Path, profile: RobotProfile) -> StaticArtifact:
    """Read an identified stiffness set with the sweeps it was measured from."""
    payload = _verify(path, profile, STATIC_KIND, STATIC_SCHEMA_VERSION)
    name, names = _group_joints(payload, profile, STATIC_KIND)

    def grid(key: str, dtype=float) -> np.ndarray:
        values = np.asarray(payload.get(key), dtype=dtype)
        if values.shape != (len(names),):
            raise ArtifactError(
                f"{key} must carry one value per joint: expected {len(names)}, "
                f"got {values.shape}"
            )
        return values

    try:
        estimate = StaticEstimate(
            joint_names=names,
            stiffness=grid("stiffness_nm_per_rad"),
            torque_scale=grid("torque_scale"),
            offset=grid("offset_rad"),
            residual_rmse=grid("residual_rmse_rad"),
            condition=grid("condition"),
            used=grid("rounds_used", int),
            excluded=grid("rounds_excluded", int),
            unidentifiable=(),
        )
    except (TypeError, ValueError) as error:
        raise ArtifactError(f"malformed static estimate: {error}") from error
    if not np.all(estimate.stiffness > 0):
        raise ArtifactError("static estimate carries a stiffness that is not positive")
    sources = (payload.get("sources") or {}).get("sweep_sha256") or []
    return StaticArtifact(
        group=name,
        estimate=estimate,
        noise_rad=float(payload.get("noise_rad", 0.0)),
        sweep_sha256=tuple(str(item) for item in sources),
    )


def write_hdf5(path: str | Path, track: CanonicalTrack) -> None:
    try:
        import h5py
    except ImportError as exc:
        raise ArtifactError("HDF5 support requires `pip install robot-control[hdf5]`") from exc
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as h5:
        demo = h5.create_group("data/demo_0")
        demo.create_dataset("timestamps_ns", data=track.timestamps_ns)
        cmd = demo.create_dataset("command/position", data=track.command)
        measured = demo.create_dataset("measured/position", data=track.measured)
        encoded_names = json.dumps(track.joint_names)
        cmd.attrs["joint_names"] = encoded_names
        measured.attrs["joint_names"] = encoded_names
        demo.attrs["track_sha256"] = track_sha256(track)
        demo.attrs["schema_version"] = 1


def read_hdf5(path: str | Path) -> CanonicalTrack:
    try:
        import h5py
    except ImportError as exc:
        raise ArtifactError("HDF5 support requires `pip install robot-control[hdf5]`") from exc
    with h5py.File(path, "r") as h5:
        demo = h5["data/demo_0"]
        command = np.asarray(demo["command/position"][:], dtype=float)
        measured = np.asarray(demo["measured/position"][:], dtype=float)
        names = tuple(json.loads(demo["command/position"].attrs["joint_names"]))
        result = CanonicalTrack(
            np.asarray(demo["timestamps_ns"][:], dtype=np.int64),
            command,
            measured,
            names,
        )
        expected = demo.attrs.get("track_sha256")
    if expected and expected != track_sha256(result):
        raise ArtifactError("track checksum mismatch")
    return result
