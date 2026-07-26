from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np


class SafetyError(RuntimeError):
    pass


@dataclass
class CommandGate:
    """Bounds every command that reaches a controller.

    Two contracts, deliberately different, because discrete commands and a
    servo stream want opposite failure modes:

    ``authorize`` and ``authorize_trajectory`` **refuse**. A discrete command
    that breaks a limit was a mistake, and silently moving the robot somewhere
    adjacent to what was asked hides it.

    ``follow`` **clamps and reports**. An operator dragging a marker faster than
    the arm can move is not making a mistake, and aborting the session on the
    first quick flick would make servoing unusable. The step is limited, and
    what limited it is returned so the caller can say so.

    ``effort`` is optional: a gate built for position commands has no torque to
    bound, and asking it to authorize one is a programming error rather than a
    limit violation.
    """

    execute: bool
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    command_period_sec: float
    effort: np.ndarray | None = None
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

    def follow(
        self,
        target: np.ndarray,
        measured: np.ndarray,
        elapsed_sec: float,
    ) -> tuple[np.ndarray, str | None]:
        """Step from *measured* towards *target*, within the limits.

        Returns the command and, if anything bounded it, a phrase naming what.
        The step is budgeted from the *measured* pose rather than from the last
        command on purpose: these arms sit behind their command by the droop
        their impedance control needs to hold position, so budgeting from the
        command would let the real movement exceed the velocity limit by that
        standing error.
        """
        self._refuse_if_closed()
        target = np.asarray(target, dtype=float)
        measured = np.asarray(measured, dtype=float)
        for values, name in ((target, "target"), (measured, "measured pose")):
            if values.shape != self.lower.shape or not np.isfinite(values).all():
                raise SafetyError(f"invalid {name} shape or value")
        if not np.isfinite(elapsed_sec) or elapsed_sec < 0:
            raise SafetyError("elapsed time must be finite and not negative")

        limited: list[str] = []
        permitted = self.velocity * max(elapsed_sec, self.command_period_sec)
        step = np.clip(target - measured, -permitted, permitted)
        if np.any(np.abs(target - measured) > permitted + 1e-12):
            limited.append("velocity")

        command = measured + step
        clamped = np.clip(command, self.lower, self.upper)
        if np.any(clamped != command):
            limited.append("position")
        command = clamped

        # _last is updated so hold_pose still names something safe, but not
        # _last_time: follow is given an interval, not a clock, and feeding the
        # watchdog from here would let a stalled stream look alive.
        self._last = command.copy()
        return command, (" and ".join(limited) + " limit" if limited else None)

    def authorize_effort(self, effort: np.ndarray) -> np.ndarray:
        """Authorize feedforward torque against the profile's effort limits.

        Refused rather than clamped, unlike a servo step. Torque is not a
        request to be somewhere, it is a force: too much does not put the arm in
        the wrong place, it accelerates it out of the place it was holding.
        """
        self._refuse_if_closed()
        if self.effort is None:
            raise SafetyError(
                "no effort limit is configured for this gate; build it with "
                "the profile's effort bounds before commanding torque"
            )
        effort = np.asarray(effort, dtype=float)
        if effort.shape != self.effort.shape or not np.isfinite(effort).all():
            raise SafetyError("invalid effort shape or value")
        if np.any(np.abs(effort) > self.effort + 1e-12):
            raise SafetyError("effort limit exceeded")
        return effort

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
