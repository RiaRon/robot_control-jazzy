"""ROS-independent Cartesian feedback control for marker servoing."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class ServoError(ValueError):
    """The Cartesian servo received an invalid setting or sample."""


@dataclass(frozen=True)
class ServoStep:
    """One bounded Cartesian velocity request and its diagnostics."""

    velocity: np.ndarray
    error_norm: float
    within_tolerance: bool
    speed_limited: bool
    integral_frozen: bool
    target_reset: bool


class CartesianPI:
    """Three-axis PI control with conditional-integration anti-windup."""

    def __init__(
        self,
        *,
        kp: float,
        ki: float,
        tolerance: float,
        max_speed: float,
        reset_distance: float = 0.005,
    ):
        values = (kp, ki, tolerance, max_speed, reset_distance)
        if not all(np.isfinite(value) for value in values):
            raise ServoError("Cartesian servo settings must be finite")
        if kp <= 0 or ki < 0 or tolerance <= 0 or max_speed <= 0:
            raise ServoError("invalid Cartesian servo gain or limit")
        if reset_distance <= 0:
            raise ServoError("reset distance must be positive")

        self.kp = float(kp)
        self.ki = float(ki)
        self.tolerance = float(tolerance)
        self.max_speed = float(max_speed)
        self.reset_distance = float(reset_distance)
        self._integral_limit = self.max_speed / self.ki if self.ki > 0 else 0.0
        self._integral = np.zeros(3)
        self._pending = self._integral.copy()
        self._previous_target: np.ndarray | None = None

    def update(
        self,
        measured: np.ndarray,
        target: np.ndarray,
        dt: float,
    ) -> ServoStep:
        """Propose a bounded velocity; call :meth:`commit` after joint gating."""
        measured = np.asarray(measured, dtype=float)
        target = np.asarray(target, dtype=float)
        if (
            measured.shape != (3,)
            or target.shape != (3,)
            or not np.isfinite(measured).all()
            or not np.isfinite(target).all()
            or not np.isfinite(dt)
            or dt <= 0
        ):
            raise ServoError("invalid Cartesian servo input")

        reset = (
            self._previous_target is not None
            and np.linalg.norm(target - self._previous_target) > self.reset_distance
        )
        self._previous_target = target.copy()
        if reset:
            self._integral = np.zeros(3)

        error = target - measured
        error_norm = float(np.linalg.norm(error))
        if error_norm <= self.tolerance:
            self._pending = self._integral.copy()
            return ServoStep(np.zeros(3), error_norm, True, False, False, reset)

        candidate = (
            np.zeros(3)
            if self.ki == 0
            else np.clip(
                self._integral + error * dt,
                -self._integral_limit,
                self._integral_limit,
            )
        )
        velocity = self.kp * error + self.ki * candidate
        speed = float(np.linalg.norm(velocity))
        speed_limited = speed > self.max_speed
        if speed_limited:
            velocity = velocity * (self.max_speed / speed)
            # Keep per-axis updates that unwind an existing integral, while
            # freezing axes that would grow farther into saturation.
            unwinding = np.abs(candidate) < np.abs(self._integral)
            self._pending = np.where(unwinding, candidate, self._integral)
        else:
            self._pending = candidate
        if reset:
            # A new goal must not inherit or immediately replace the old
            # goal's compensation on the reset sample.
            self._pending = np.zeros(3)

        return ServoStep(
            velocity=velocity,
            error_norm=error_norm,
            within_tolerance=False,
            speed_limited=speed_limited,
            integral_frozen=(
                speed_limited
                and not np.array_equal(self._pending, candidate)
            ),
            target_reset=reset,
        )

    def commit(self, *, joint_limited: bool) -> None:
        """Accept the proposed integral only if joint safety did not intervene."""
        if not joint_limited:
            self._integral = self._pending.copy()

    @property
    def integral(self) -> np.ndarray:
        """Return a copy of the bounded integral state for diagnostics."""
        return self._integral.copy()
