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
    #: How far a streamed command may run ahead of the measured position. Needed
    #: because ``follow`` rate-limits from the previous command: that is what
    #: lets the command outrun the arm's standing droop, and on its own it would
    #: also let a blocked joint wind up command, and torque, without limit.
    max_lead: np.ndarray | None = None
    #: Joint names, in the group's order, used only to say which joint broke a
    #: limit. "position limit exceeded" over seven joints names nothing an
    #: operator can act on; whether the pose is wrong or the profile is wrong is
    #: exactly what the numbers decide.
    names: Sequence[str] | None = None
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
        """Step towards *target*, rate-limited and bounded, for a servo stream.

        Returns the command and, if anything bounded it, a phrase naming what.

        The step is rate-limited from the **previous command**, not from the
        measured position. That distinction decides whether the arm moves at all:
        these joints hold position through impedance control, so they sit behind
        their command by the droop that produces their holding torque. Budgeting
        from the measured position caps the command at one period's travel ahead
        of where the arm *is*, which is less than that standing droop — so the
        command lands behind the equilibrium and the arm never advances. Fake
        hardware, having no droop, tracks perfectly either way and hides it.

        Rate-limiting from the command alone would let a joint held still by an
        obstacle accumulate command without limit, and with it the torque its
        stiffness produces, so ``max_lead`` bounds how far the command may run
        ahead of the measured position.
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
        # Nothing to rate-limit from on the first sample, so the measured pose is
        # the only honest starting point.
        base = measured if self._last is None else self._last
        permitted = self.velocity * max(elapsed_sec, self.command_period_sec)
        step = np.clip(target - base, -permitted, permitted)
        if np.any(np.abs(target - base) > permitted + 1e-12):
            limited.append("velocity")

        command = base + step
        if self.max_lead is not None:
            lead = command - measured
            bounded = np.clip(lead, -self.max_lead, self.max_lead)
            if np.any(bounded != lead):
                limited.append("lead")
            command = measured + bounded

        clamped = np.clip(command, self.lower, self.upper)
        if np.any(clamped != command):
            limited.append("position")
        command = clamped

        # _last carries the command, which is what the next step budgets from,
        # and what hold_pose should name. Not _last_time: follow is given an
        # interval rather than a clock, and feeding the watchdog from here would
        # let a stalled stream look alive.
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
        outside = (command < self.lower) | (command > self.upper)
        if outside.any():
            offenders = self._offenders(command, outside)
            raise SafetyError(f"position limit exceeded{where}: {offenders}")

    def _offenders(self, command: np.ndarray, outside: np.ndarray) -> str:
        """Name each joint that broke a bound, with the value and the bound."""
        parts = []
        for index in np.flatnonzero(outside):
            name = self.names[index] if self.names else f"joint {index}"
            parts.append(
                f"{name}={command[index]:+.4f} outside "
                f"[{self.lower[index]:+.4f}, {self.upper[index]:+.4f}]"
            )
        return ", ".join(parts)

    def _check_velocity(
        self,
        command: np.ndarray,
        previous: np.ndarray,
        elapsed: float,
        where: str,
    ) -> None:
        permitted = self.velocity * max(elapsed, self.command_period_sec)
        over = np.abs(command - previous) > permitted + 1e-12
        if over.any():
            moved = np.abs(command - previous)
            parts = [
                f"{self.names[i] if self.names else f'joint {i}'} moves "
                f"{moved[i]:.4f} rad against a budget of {permitted[i]:.4f}"
                for i in np.flatnonzero(over)
            ]
            raise SafetyError(f"velocity limit exceeded{where}: {', '.join(parts)}")

    def estop(self) -> None:
        self._estopped = True

    def hold_pose(self) -> np.ndarray:
        if self._last is None:
            raise SafetyError("no safe pose is available to hold")
        return self._last.copy()
