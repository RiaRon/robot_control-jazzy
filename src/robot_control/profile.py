from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
from typing import Any

import yaml


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Joint:
    canonical: str
    source: str
    sign: int
    unit: str
    lower: float
    upper: float
    velocity: float
    effort: float


@dataclass(frozen=True)
class Group:
    name: str
    joints: tuple[str, ...]


@dataclass(frozen=True)
class RosEndpoint:
    command_topic: str
    state_topic: str
    controller: str
    command_rate_hz: float


@dataclass(frozen=True)
class RobotProfile:
    name: str
    components: tuple[str, ...]
    asset_id: str
    manifest_path: Path
    manifest_sha256: str
    joints: tuple[Joint, ...]
    groups: dict[str, Group]
    ros: dict[str, RosEndpoint]

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.canonical for j in self.joints)


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProfileError(f"{label} must be an object")
    return value


def _resolve_manifest(profile_path: Path, value: str) -> Path:
    configured = Path(value)
    if configured.is_absolute():
        return configured

    direct = (profile_path.parent / configured).resolve()
    if direct.is_file():
        return direct

    parts = configured.parts
    if "hdgp" not in parts:
        return direct
    hdgp_relative = Path(*parts[parts.index("hdgp") + 1 :])

    explicit_root = os.environ.get("HDGP_ROOT")
    if explicit_root:
        return (Path(explicit_root).expanduser() / hdgp_relative).resolve()

    for ancestor in profile_path.parents:
        candidate = ancestor / "hdgp" / hdgp_relative
        if candidate.is_file():
            return candidate.resolve()
    return direct


def load_profile(path: str | Path) -> RobotProfile:
    path = Path(path).resolve()
    raw = _mapping(yaml.safe_load(path.read_text()), "profile")
    asset = _mapping(raw.get("asset"), "asset")
    manifest = _resolve_manifest(path, str(asset["manifest"]))
    if not manifest.is_file():
        raise ProfileError(f"asset manifest not found: {manifest}")
    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    expected = str(asset.get("manifest_sha256", ""))
    if digest != expected:
        raise ProfileError(f"manifest hash mismatch: expected {expected}, got {digest}")

    joints = tuple(Joint(**item) for item in raw.get("joints", []))
    names = tuple(j.canonical for j in joints)
    if not joints or len(set(names)) != len(names):
        raise ProfileError("joints must be non-empty and canonical names unique")
    for joint in joints:
        if joint.sign not in (-1, 1) or joint.unit != "rad":
            raise ProfileError(f"invalid normalization for joint {joint.canonical}")
        if not joint.lower < joint.upper or joint.velocity <= 0 or joint.effort <= 0:
            raise ProfileError(f"invalid safety limits for joint {joint.canonical}")

    manifest_raw = _mapping(yaml.safe_load(manifest.read_text()), "manifest")
    manifest_joints = set(manifest_raw.get("control_joint_order", []))
    missing_manifest = set(names) - manifest_joints
    if missing_manifest:
        raise ProfileError(f"joints absent from asset manifest: {sorted(missing_manifest)}")

    groups = {
        name: Group(name=name, joints=tuple(_mapping(body, f"group {name}")["joints"]))
        for name, body in _mapping(raw.get("groups"), "groups").items()
    }
    counts = {name: 0 for name in names}
    unknown: set[str] = set()
    for group in groups.values():
        for name in group.joints:
            if name in counts:
                counts[name] += 1
            else:
                unknown.add(name)
    if unknown or any(count != 1 for count in counts.values()):
        raise ProfileError(
            "every canonical joint must belong to exactly one actuator group"
            f"; unknown={sorted(unknown)}, counts={counts}"
        )

    ros = {
        distro: RosEndpoint(**_mapping(body, f"ros.{distro}"))
        for distro, body in _mapping(raw.get("ros"), "ros").items()
    }
    for distro, endpoint in ros.items():
        if distro not in {"humble", "jazzy"} or endpoint.command_rate_hz <= 0:
            raise ProfileError(f"invalid ROS endpoint: {distro}")
    return RobotProfile(
        name=str(raw["name"]),
        components=tuple(raw["components"]),
        asset_id=str(asset["id"]),
        manifest_path=manifest,
        manifest_sha256=digest,
        joints=joints,
        groups=groups,
        ros=ros,
    )


def load_builtin_profile(name: str) -> RobotProfile:
    resource = files("robot_control").joinpath("profiles", f"{name}.yaml")
    if not resource.is_file():
        raise ProfileError(f"unknown profile: {name}")
    return load_profile(Path(str(resource)))
