from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from .track import CanonicalTrack


class ArtifactError(RuntimeError):
    pass


def track_sha256(track: CanonicalTrack) -> str:
    digest = hashlib.sha256()
    digest.update(track.timestamps_ns.astype("<i8").tobytes())
    digest.update(track.command.astype("<f8").tobytes())
    digest.update(track.measured.astype("<f8").tobytes())
    digest.update(json.dumps(track.joint_names, separators=(",", ":")).encode())
    return digest.hexdigest()


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
