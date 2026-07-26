from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


class FitError(ValueError):
    pass


@dataclass(frozen=True)
class SecondOrderEstimate:
    stiffness: np.ndarray
    damping: np.ndarray
    friction: np.ndarray
    residual_rmse: np.ndarray


@dataclass(frozen=True)
class ValidationResult:
    status: str
    failures: tuple[str, ...]


@dataclass(frozen=True)
class GravitySweep:
    """One arm pose held at several gravity feedforward scales, and what it did.

    At each round a known torque is published and the joint's standing error is
    read, which is the observation a stiffness estimate is made of:

    ``kp (q_cmd - q) = tau_g(q) - s tau_model(q)``

    Every round carries its own pose and its own modelled torque rather than one
    for the whole sweep. Compensation moves the arm, so the load a joint holds
    at scale 1.0 is not the load it was holding at 0.0, and pairing an error
    with the torque from a different pose is the one way to get a confident
    wrong number out of the fit.
    """

    group: str
    joint_names: tuple[str, ...]
    #: (rounds, joints) canonical joint values measured before each round's read.
    poses: np.ndarray
    #: (rounds, joints) modelled gravity torque at that round's pose, unscaled.
    modelled_torque: np.ndarray
    #: (rounds, joints) the fraction of it actually published.
    scales: np.ndarray
    #: (rounds, joints) the controller's own tracking error after the hold.
    errors: np.ndarray
    #: The joint whose scale varied, when the sweep varied only one.
    sweep_joint: str | None = None

    _GRIDS = ("poses", "modelled_torque", "scales", "errors")

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        if not names:
            raise FitError("a sweep needs at least one joint")
        object.__setattr__(self, "joint_names", names)
        for field_name in self._GRIDS:
            grid = np.asarray(getattr(self, field_name), dtype=float)
            if grid.ndim != 2 or grid.shape[1] != len(names):
                raise FitError(
                    f"{field_name} must be (rounds, {len(names)}), got {grid.shape}"
                )
            if not np.isfinite(grid).all():
                raise FitError(f"{field_name} carries a non-finite value")
            object.__setattr__(self, field_name, grid)
        counts = {getattr(self, name).shape[0] for name in self._GRIDS}
        if len(counts) != 1:
            raise FitError(
                "every round needs a pose, a modelled torque, a scale and an "
                f"error, but the counts differ: {sorted(counts)}"
            )
        if self.rounds == 0:
            raise FitError("a sweep needs at least one round")
        if self.sweep_joint is not None and self.sweep_joint not in names:
            raise FitError(
                f"sweep_joint {self.sweep_joint!r} is not one of {list(names)}"
            )

    @property
    def rounds(self) -> int:
        return int(self.poses.shape[0])


def build_excitation(
    neutral: np.ndarray,
    amplitude: np.ndarray,
    rate_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a deterministic step/ramp/hold/multisine identification track."""
    neutral = np.asarray(neutral, dtype=float)
    amplitude = np.asarray(amplitude, dtype=float)
    if neutral.shape != amplitude.shape or neutral.ndim != 1 or rate_hz <= 0:
        raise FitError("invalid excitation arguments")
    definitions = (
        ("hold", 0.5),
        ("step", 0.5),
        ("hold", 0.5),
        ("ramp", 1.0),
        ("multisine", 3.0),
        ("hold", 0.5),
    )
    blocks: list[np.ndarray] = []
    labels: list[str] = []
    phase_start = neutral.copy()
    elapsed = 0.0
    for name, duration in definitions:
        count = max(2, int(round(duration * rate_hz)))
        local = np.arange(count) / rate_hz
        if name == "step":
            values = np.tile(neutral + amplitude, (count, 1))
        elif name == "ramp":
            alpha = np.linspace(1.0, -1.0, count)[:, None]
            values = neutral + alpha * amplitude
        elif name == "multisine":
            frequencies = (0.7, 1.3, 2.1, 3.7)
            wave = sum(np.sin(2 * np.pi * f * (elapsed + local)) for f in frequencies)
            wave /= len(frequencies)
            values = neutral + wave[:, None] * amplitude
        else:
            values = np.tile(phase_start, (count, 1))
        phase_start = values[-1]
        blocks.append(values)
        labels.extend([name] * count)
        elapsed += duration
    command = np.vstack(blocks)
    time = np.arange(len(command), dtype=float) / rate_hz
    return time, command, np.asarray(labels)


def split_repetitions(run_ids: Sequence[str]) -> tuple[tuple[str, ...], tuple[str, ...]]:
    if len(run_ids) != 3 or len(set(run_ids)) != 3:
        raise FitError("identification requires exactly three unique repetitions")
    return tuple(run_ids[:2]), (run_ids[2],)


def fit_second_order(
    time_sec: np.ndarray, command: np.ndarray, measured: np.ndarray
) -> SecondOrderEstimate:
    """Fit qdd = k(q_cmd-q) - d*qd - f*sign(qd) independently per joint."""
    time_sec = np.asarray(time_sec, dtype=float)
    command = np.asarray(command, dtype=float)
    measured = np.asarray(measured, dtype=float)
    if (
        time_sec.ndim != 1
        or command.ndim != 2
        or measured.shape != command.shape
        or len(time_sec) != len(command)
        or len(time_sec) < 5
        or np.any(np.diff(time_sec) <= 0)
    ):
        raise FitError("invalid fit track")
    dt = np.diff(time_sec)
    if np.max(dt) - np.min(dt) > np.mean(dt) * 1e-4:
        raise FitError("fit track must be uniformly sampled")
    step = float(np.mean(dt))
    velocity = np.diff(measured, axis=0) / step
    acceleration = np.diff(velocity, axis=0) / step
    stiffness = np.empty(command.shape[1])
    damping = np.empty(command.shape[1])
    friction = np.empty(command.shape[1])
    residual = np.empty(command.shape[1])
    for joint in range(command.shape[1]):
        error = command[1:-1, joint] - measured[1:-1, joint]
        prior_velocity = velocity[:-1, joint]
        design = np.column_stack((error, -prior_velocity, -np.sign(prior_velocity)))
        params, _, rank, _ = np.linalg.lstsq(design, acceleration[:, joint], rcond=None)
        if rank < 2 or params[0] <= 0 or params[1] < 0:
            raise FitError(f"unidentifiable dynamics for joint {joint}")
        prediction = design @ params
        stiffness[joint], damping[joint], friction[joint] = params
        residual[joint] = float(np.sqrt(np.mean((prediction - acceleration[:, joint]) ** 2)))
    return SecondOrderEstimate(stiffness, damping, friction, residual)


def validate_holdout(
    *,
    openarm_rmse_rad: float,
    tesollo_rmse_rad: float,
    delay_residual_sec: float,
    command_period_sec: float,
    improvement_fraction: float,
) -> ValidationResult:
    failures = []
    if openarm_rmse_rad > 0.03:
        failures.append("openarm_rmse")
    if tesollo_rmse_rad > 0.05:
        failures.append("tesollo_rmse")
    if delay_residual_sec > command_period_sec:
        failures.append("delay_residual")
    if improvement_fraction < 0.30:
        failures.append("tracking_improvement")
    return ValidationResult(
        status="validated" if not failures else "model_inadequate",
        failures=tuple(failures),
    )
