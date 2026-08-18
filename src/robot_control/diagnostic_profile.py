"""Deterministic, bounded Cartesian target profiles for pose-follow diagnosis."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


DEFAULT_DISTANCE_M = 0.010
DEFAULT_ANGLE_RAD = math.radians(5.0)
DEFAULT_LINEAR_SPEED_M_S = 0.005
DEFAULT_ANGULAR_SPEED_RAD_S = 0.05
DEFAULT_HOLD_SEC = 3.0
DEFAULT_REPETITIONS = 1
MAX_DISTANCE_M = 0.030
MAX_ANGLE_RAD = math.radians(10.0)
MAX_LINEAR_SPEED_M_S = 0.020
MAX_ANGULAR_SPEED_RAD_S = 0.10
MAX_HOLD_SEC = 10.0
MAX_REPETITIONS = 3
PROFILE_KINDS = ("translation", "rotation", "translation-rotation")
AXES = ("x", "y", "z")


@dataclass(frozen=True)
class ProfileSample:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    phase: str
    repetition: int
    complete: bool


@dataclass(frozen=True)
class _Stage:
    phase: str
    repetition: int
    duration_sec: float
    translation_start: float = 0.0
    translation_end: float = 0.0
    rotation_start: float = 0.0
    rotation_end: float = 0.0


@dataclass(frozen=True)
class DiagnosticProfile:
    """A round-trip target path with a hard safety envelope."""

    kind: str = "translation"
    distance_m: float = DEFAULT_DISTANCE_M
    angle_rad: float = DEFAULT_ANGLE_RAD
    linear_speed_m_s: float = DEFAULT_LINEAR_SPEED_M_S
    angular_speed_rad_s: float = DEFAULT_ANGULAR_SPEED_RAD_S
    hold_sec: float = DEFAULT_HOLD_SEC
    repetitions: int = DEFAULT_REPETITIONS
    translation_axis: str = "x"
    rotation_axis: str = "z"

    def __post_init__(self) -> None:
        if self.kind not in PROFILE_KINDS:
            raise ValueError(f"unknown diagnostic profile {self.kind!r}")
        if self.translation_axis not in AXES or self.rotation_axis not in AXES:
            raise ValueError("diagnostic axes must be x, y, or z")
        for name, value, maximum in (
            ("distance", self.distance_m, MAX_DISTANCE_M),
            ("angle", self.angle_rad, MAX_ANGLE_RAD),
            ("linear speed", self.linear_speed_m_s, MAX_LINEAR_SPEED_M_S),
            ("angular speed", self.angular_speed_rad_s, MAX_ANGULAR_SPEED_RAD_S),
            ("hold", self.hold_sec, MAX_HOLD_SEC),
        ):
            if not np.isfinite(value) or value <= 0.0 or value > maximum:
                raise ValueError(
                    f"diagnostic {name} must be finite, positive, and no "
                    f"greater than {maximum:g}"
                )
        if not isinstance(self.repetitions, int) or not (
            1 <= self.repetitions <= MAX_REPETITIONS
        ):
            raise ValueError(
                f"diagnostic repetitions must be between 1 and {MAX_REPETITIONS}"
            )

    @property
    def stages(self) -> tuple[_Stage, ...]:
        stages: list[_Stage] = []
        for repetition in range(1, self.repetitions + 1):
            if self.kind in ("translation", "translation-rotation"):
                ramp = self.distance_m / self.linear_speed_m_s
                stages += [
                    _Stage("translation_ramp_out", repetition, ramp, 0.0, 1.0),
                    _Stage("translation_hold", repetition, self.hold_sec, 1.0, 1.0),
                    _Stage("translation_ramp_back", repetition, ramp, 1.0, 0.0),
                    _Stage("origin_hold", repetition, self.hold_sec),
                ]
            if self.kind in ("rotation", "translation-rotation"):
                ramp = self.angle_rad / self.angular_speed_rad_s
                stages += [
                    _Stage(
                        "rotation_ramp_out", repetition, ramp,
                        rotation_start=0.0, rotation_end=1.0,
                    ),
                    _Stage(
                        "rotation_hold", repetition, self.hold_sec,
                        rotation_start=1.0, rotation_end=1.0,
                    ),
                    _Stage(
                        "rotation_ramp_back", repetition, ramp,
                        rotation_start=1.0, rotation_end=0.0,
                    ),
                    _Stage("origin_hold", repetition, self.hold_sec),
                ]
        return tuple(stages)

    @property
    def duration_sec(self) -> float:
        return float(sum(stage.duration_sec for stage in self.stages))

    def sample(self, elapsed_sec, origin_position, origin_orientation) -> ProfileSample:
        position = np.asarray(origin_position, dtype=float)
        orientation = _normalise_quaternion(origin_orientation)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("diagnostic profile origin position must be finite xyz")
        remaining = max(0.0, float(elapsed_sec))
        for stage in self.stages:
            if remaining <= stage.duration_sec:
                fraction = min(1.0, remaining / stage.duration_sec)
                translation = stage.translation_start + fraction * (
                    stage.translation_end - stage.translation_start
                )
                rotation = stage.rotation_start + fraction * (
                    stage.rotation_end - stage.rotation_start
                )
                return self._sample_at(
                    position, orientation, translation, rotation,
                    stage.phase, stage.repetition, False,
                )
            remaining -= stage.duration_sec
        return self._sample_at(
            position, orientation, 0.0, 0.0,
            "complete", self.repetitions, True,
        )

    def _sample_at(
        self, origin_position, origin_orientation,
        translation_fraction, rotation_fraction, phase, repetition, complete,
    ) -> ProfileSample:
        direction = np.zeros(3)
        direction[AXES.index(self.translation_axis)] = 1.0
        position = origin_position + direction * self.distance_m * translation_fraction
        axis = np.zeros(3)
        axis[AXES.index(self.rotation_axis)] = 1.0
        half_angle = self.angle_rad * rotation_fraction / 2.0
        delta = np.concatenate((axis * math.sin(half_angle), [math.cos(half_angle)]))
        rotated = _normalise_quaternion(
            _quaternion_multiply(origin_orientation, delta)
        )
        return ProfileSample(
            tuple(float(value) for value in position),
            tuple(float(value) for value in rotated),
            phase, repetition, complete,
        )

    def as_dict(self) -> dict:
        return {
            "kind": self.kind,
            "distance_m": float(self.distance_m),
            "angle_rad": float(self.angle_rad),
            "linear_speed_m_s": float(self.linear_speed_m_s),
            "angular_speed_rad_s": float(self.angular_speed_rad_s),
            "hold_sec": float(self.hold_sec),
            "repetitions": int(self.repetitions),
            "translation_axis_world": self.translation_axis,
            "rotation_axis_local": self.rotation_axis,
            "duration_sec": self.duration_sec,
        }


def _normalise_quaternion(values) -> np.ndarray:
    quaternion = np.asarray(values, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("diagnostic origin orientation must be finite xyzw")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("diagnostic origin orientation must be non-zero")
    return quaternion / norm


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ), dtype=float)
