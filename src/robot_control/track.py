from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class TrackError(ValueError):
    pass


@dataclass(frozen=True)
class CanonicalTrack:
    timestamps_ns: np.ndarray
    command: np.ndarray
    measured: np.ndarray
    joint_names: tuple[str, ...]


def _validate(time: np.ndarray, values: np.ndarray, label: str, width: int) -> None:
    if time.ndim != 1 or values.ndim != 2 or len(time) != len(values) or values.shape[1] != width:
        raise TrackError(f"invalid {label} track shape")
    if len(time) < 2 or np.any(np.diff(time) <= 0):
        raise TrackError(f"{label} timestamps must be strictly increasing")
    if not np.isfinite(values).all():
        raise TrackError(f"{label} contains non-finite values")


def normalize_track(
    command_time_ns,
    command,
    measured_time_ns,
    measured,
    joint_names,
    rate_hz,
    minimum_range_rad=1e-3,
) -> CanonicalTrack:
    command_time_ns = np.asarray(command_time_ns, dtype=np.int64)
    measured_time_ns = np.asarray(measured_time_ns, dtype=np.int64)
    command = np.asarray(command, dtype=float)
    measured = np.asarray(measured, dtype=float)
    width = len(joint_names)
    _validate(command_time_ns, command, "command", width)
    _validate(measured_time_ns, measured, "measured", width)
    if rate_hz <= 0:
        raise TrackError("rate_hz must be positive")
    ranges = np.ptp(command, axis=0)
    deficient = [joint_names[i] for i, value in enumerate(ranges) if value < minimum_range_rad]
    if deficient:
        raise TrackError(f"insufficient excitation: {deficient}")
    start = max(command_time_ns[0], measured_time_ns[0])
    stop = min(command_time_ns[-1], measured_time_ns[-1])
    step = int(round(1e9 / rate_hz))
    timestamps = np.arange(start, stop + 1, step, dtype=np.int64)
    if len(timestamps) < 2:
        raise TrackError("command and measured tracks do not overlap")

    def interpolate(time, values):
        return np.column_stack(
            [np.interp(timestamps, time, values[:, index]) for index in range(width)]
        )

    return CanonicalTrack(
        timestamps, interpolate(command_time_ns, command), interpolate(measured_time_ns, measured),
        tuple(joint_names),
    )
