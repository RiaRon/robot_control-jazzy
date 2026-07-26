from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np


class SafetyError(RuntimeError):
    pass


@dataclass
class CommandGate:
    execute: bool
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    command_period_sec: float
    watchdog_sec: float = 0.25
    _last: np.ndarray | None = field(default=None, init=False)
    _last_time: float | None = field(default=None, init=False)
    _estopped: bool = field(default=False, init=False)

    def authorize(self, command: np.ndarray, now_sec: float) -> np.ndarray:
        self._refuse_if_closed()
        command = np.asarray(command, dtype=float)
        self._check_position(command, "")
        if self._last_time is not None:
            elapsed = now_sec - self._last_time
            if elapsed > self.watchdog_sec:
                raise SafetyError("watchdog expired; hold pose required")
            self._check_velocity(command, self._last, elapsed, "")
        self._last = command.copy()
        self._last_time = now_sec
        return command

    def authorize_trajectory(
        self,
        points: Sequence[np.ndarray],
        start_time_sec: float,
        period_sec: float,
    ) -> list[np.ndarray]:
        """Authorize a whole trajectory, or none of it.

        A truncated trajectory is its own hazard: it stops the robot at an
        arbitrary intermediate waypoint. So every waypoint is validated before
        any is returned, and gate state is committed only once all of them pass.

        ``start_time_sec`` is when the trajectory is dispatched; waypoint *i* is
        reached at ``start_time_sec + (i + 1) * period_sec``. Every waypoint,
        including the first, therefore has one period of travel budget. The gap
        since the last authorized command is the watchdog's business, not a
        velocity budget: charging the first waypoint against it would reject a
        trajectory dispatched immediately after reading the current pose.

        The watchdog guards that leading gap, not the spacing inside the
        trajectory. A trajectory is handed to the controller as a single goal
        and interpolated there, so waypoints deliberately spaced wider than the
        watchdog are a planned motion rather than a stalled command stream.
        """
        self._refuse_if_closed()
        points = [np.asarray(point, dtype=float) for point in points]
        if not points:
            raise SafetyError("trajectory has no waypoints")
        if not np.isfinite(period_sec) or period_sec <= 0:
            raise SafetyError("trajectory period must be positive and finite")

        if self._last_time is not None:
            idle = start_time_sec - self._last_time
            if idle < 0:
                raise SafetyError("trajectory starts before the last authorized command")
            if idle > self.watchdog_sec:
                raise SafetyError("watchdog expired; hold pose required")

        previous = self._last
        for index, point in enumerate(points):
            where = f" at waypoint {index}"
            self._check_position(point, where)
            if previous is not None:
                self._check_velocity(point, previous, period_sec, where)
            previous = point

        self._last = points[-1].copy()
        self._last_time = start_time_sec + len(points) * period_sec
        return points

    def _refuse_if_closed(self) -> None:
        if not self.execute:
            raise SafetyError("publishing requires explicit --execute")
        if self._estopped:
            raise SafetyError("E-stop is active")

    def _check_position(self, command: np.ndarray, where: str) -> None:
        if command.shape != self.lower.shape or not np.isfinite(command).all():
            raise SafetyError(f"invalid command shape or value{where}")
        if np.any(command < self.lower) or np.any(command > self.upper):
            raise SafetyError(f"position limit exceeded{where}")

    def _check_velocity(
        self,
        command: np.ndarray,
        previous: np.ndarray,
        elapsed: float,
        where: str,
    ) -> None:
        permitted = self.velocity * max(elapsed, self.command_period_sec)
        if np.any(np.abs(command - previous) > permitted + 1e-12):
            raise SafetyError(f"velocity limit exceeded{where}")

    def estop(self) -> None:
        self._estopped = True

    def hold_pose(self) -> np.ndarray:
        if self._last is None:
            raise SafetyError("no safe pose is available to hold")
        return self._last.copy()
