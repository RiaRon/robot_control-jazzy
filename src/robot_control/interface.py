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
