from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .identification import FitError, GravitySweep
from .profile import RobotProfile
from .track import CanonicalTrack


class ArtifactError(RuntimeError):
    pass


#: Bumped when the sweep payload's meaning changes, not when a field is added
#: that older readers can ignore — there are none, so it never has been.
SWEEP_SCHEMA_VERSION = 1


def track_sha256(track: CanonicalTrack) -> str:
    digest = hashlib.sha256()
    digest.update(track.timestamps_ns.astype("<i8").tobytes())
    digest.update(track.command.astype("<f8").tobytes())
    digest.update(track.measured.astype("<f8").tobytes())
    digest.update(json.dumps(track.joint_names, separators=(",", ":")).encode())
    return digest.hexdigest()


def _canonical_bytes(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()


def write_sweep(
    path: str | Path, sweep: GravitySweep, profile: RobotProfile
) -> None:
    """Write a measured sweep, tied to the robot it was measured on.

    The profile name, the asset id and the manifest hash go in the file rather
    than being supplied at read time, because a sweep is a set of numbers about
    one physical arm: read against a different one it is not merely stale, it
    is wrong in a way nothing downstream could detect.
    """
    payload: dict[str, Any] = {
        "schema_version": SWEEP_SCHEMA_VERSION,
        "kind": "gravity_sweep",
        "profile": profile.name,
        "asset": {
            "id": profile.asset_id,
            "manifest_sha256": profile.manifest_sha256,
        },
        "group": sweep.group,
        "joint_names": list(sweep.joint_names),
        "sweep_joint": sweep.sweep_joint,
        "rounds": [
            {
                "pose": sweep.poses[index].tolist(),
                "modelled_torque": sweep.modelled_torque[index].tolist(),
                "scale": sweep.scales[index].tolist(),
                "error": sweep.errors[index].tolist(),
            }
            for index in range(sweep.rounds)
        ],
    }
    payload["checksum_sha256"] = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def read_sweep(path: str | Path, profile: RobotProfile) -> GravitySweep:
    payload = json.loads(Path(path).read_text())
    version = payload.get("schema_version")
    if version != SWEEP_SCHEMA_VERSION:
        raise ArtifactError(
            f"unsupported sweep schema_version: {version!r}, expected "
            f"{SWEEP_SCHEMA_VERSION}"
        )

    checksum = payload.get("checksum_sha256")
    unsigned = {key: value for key, value in payload.items() if key != "checksum_sha256"}
    if checksum != hashlib.sha256(_canonical_bytes(unsigned)).hexdigest():
        raise ArtifactError("sweep checksum mismatch")

    asset = payload.get("asset") or {}
    if (
        payload.get("profile") != profile.name
        or asset.get("id") != profile.asset_id
        or asset.get("manifest_sha256") != profile.manifest_sha256
    ):
        raise ArtifactError("profile or asset manifest mismatch")

    name = payload.get("group")
    group = profile.groups.get(name)
    if group is None:
        raise ArtifactError(
            f"sweep names group {name!r}, which this profile does not have"
        )
    names = tuple(payload.get("joint_names") or ())
    if names != group.joints:
        raise ArtifactError(
            f"sweep joint names do not match group {name!r}: {list(names)} "
            f"against {list(group.joints)}"
        )

    rounds = payload.get("rounds") or []
    try:
        return GravitySweep(
            group=name,
            joint_names=names,
            poses=np.array([entry["pose"] for entry in rounds], dtype=float),
            modelled_torque=np.array(
                [entry["modelled_torque"] for entry in rounds], dtype=float
            ),
            scales=np.array([entry["scale"] for entry in rounds], dtype=float),
            errors=np.array([entry["error"] for entry in rounds], dtype=float),
            sweep_joint=payload.get("sweep_joint"),
        )
    except (FitError, KeyError, TypeError, ValueError) as error:
        raise ArtifactError(f"malformed sweep: {error}") from error


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
