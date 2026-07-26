from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from .profile import RobotProfile


class InterfaceError(ValueError):
    pass


class CanonicalInterface:
    """Name/order/sign/unit boundary shared by ROS, fake hardware, and sim2real."""

    def __init__(self, profile: RobotProfile):
        self.profile = profile
        self._sources = tuple(joint.source for joint in profile.joints)
        self._sign = np.asarray([joint.sign for joint in profile.joints], dtype=float)
        index = {joint.canonical: position for position, joint in enumerate(profile.joints)}
        self._group_index = {
            name: tuple(index[canonical] for canonical in group.joints)
            for name, group in profile.groups.items()
        }

    def command_to_source(self, values: Sequence[float]) -> dict[str, float]:
        values = np.asarray(values, dtype=float)
        if values.shape != (len(self._sources),) or not np.isfinite(values).all():
            raise InterfaceError("canonical command shape/value mismatch")
        return dict(zip(self._sources, values * self._sign))

    def state_to_canonical(self, values: Mapping[str, float]) -> np.ndarray:
        missing = set(self._sources) - set(values)
        unknown = set(values) - set(self._sources)
        if missing or unknown:
            raise InterfaceError(
                f"source joint coverage mismatch: missing={sorted(missing)}, unknown={sorted(unknown)}"
            )
        source = np.asarray([values[name] for name in self._sources], dtype=float)
        if not np.isfinite(source).all():
            raise InterfaceError("source state contains non-finite values")
        return source * self._sign

    def group_source_names(self, group: str) -> tuple[str, ...]:
        """Return one group's source joint names, in profile order."""
        return tuple(self._sources[index] for index in self._indices(group))

    def group_command_to_source(
        self, group: str, values: Sequence[float]
    ) -> dict[str, float]:
        indices = self._indices(group)
        values = np.asarray(values, dtype=float)
        if values.shape != (len(indices),) or not np.isfinite(values).all():
            raise InterfaceError(
                f"canonical command shape/value mismatch for group {group}: "
                f"expected {len(indices)} finite values"
            )
        return {
            self._sources[index]: float(value * self._sign[index])
            for value, index in zip(values, indices)
        }

    def group_state_to_canonical(
        self, group: str, values: Mapping[str, float]
    ) -> np.ndarray:
        """Extract one group's canonical values from a whole-robot joint state.

        A ``/joint_states`` message carries every joint the robot publishes, so
        names outside the group are expected and ignored. Every joint the group
        does own must be present.
        """
        indices = self._indices(group)
        sources = [self._sources[index] for index in indices]
        missing = [name for name in sources if name not in values]
        if missing:
            raise InterfaceError(
                f"source joint coverage mismatch for group {group}: missing={missing}"
            )
        source = np.asarray([values[name] for name in sources], dtype=float)
        if not np.isfinite(source).all():
            raise InterfaceError(f"source state for group {group} is non-finite")
        return source * self._sign[list(indices)]

    def _indices(self, group: str) -> tuple[int, ...]:
        if group not in self._group_index:
            raise InterfaceError(
                f"unknown group {group!r}; known groups are "
                f"{sorted(self._group_index)}"
            )
        return self._group_index[group]
