from __future__ import annotations

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
        if not self.execute:
            raise SafetyError("publishing requires explicit --execute")
        if self._estopped:
            raise SafetyError("E-stop is active")
        command = np.asarray(command, dtype=float)
        if command.shape != self.lower.shape or not np.isfinite(command).all():
            raise SafetyError("invalid command shape or value")
        if np.any(command < self.lower) or np.any(command > self.upper):
            raise SafetyError("position limit exceeded")
        if self._last_time is not None:
            elapsed = now_sec - self._last_time
            if elapsed > self.watchdog_sec:
                raise SafetyError("watchdog expired; hold pose required")
            permitted = self.velocity * max(elapsed, self.command_period_sec)
            if np.any(np.abs(command - self._last) > permitted + 1e-12):
                raise SafetyError("velocity limit exceeded")
        self._last = command.copy()
        self._last_time = now_sec
        return command

    def estop(self) -> None:
        self._estopped = True

    def hold_pose(self) -> np.ndarray:
        if self._last is None:
            raise SafetyError("no safe pose is available to hold")
        return self._last.copy()
