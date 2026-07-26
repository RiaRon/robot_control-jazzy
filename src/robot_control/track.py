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


@dataclass(frozen=True)
class Recording:
    """One stream as it actually arrived, on its own clock and at its own rate.

    Deliberately not a :class:`CanonicalTrack`. That type is the *output* of
    :func:`normalize_track`, on a uniform grid shared with another stream.
    Resampling here instead would throw away the arrival pattern, and the
    arrival pattern is evidence about the pipeline being identified: how often
    messages came, how many went missing, and whether they came in order.

    Nothing here requires the stamps to be monotonic or evenly spaced. A
    recording is a record of what happened, and what happened is sometimes
    reordering or a dropped run; that has to be visible rather than repaired.
    """

    timestamps_ns: np.ndarray
    values: np.ndarray
    joint_names: tuple[str, ...]
    #: Messages received that did not cover every joint asked for, so could not
    #: be turned into a row. A gap in the record rather than an error.
    incomplete: int = 0

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        object.__setattr__(self, "joint_names", names)
        stamps = np.asarray(self.timestamps_ns, dtype=np.int64)
        values = np.asarray(self.values, dtype=float)
        if values.ndim != 2 or values.shape[1] != len(names):
            raise TrackError(
                f"a recording needs one column per joint: expected "
                f"{len(names)}, got {values.shape}"
            )
        if stamps.ndim != 1 or len(stamps) != len(values):
            raise TrackError(
                f"every sample needs a stamp: {len(stamps)} stamps against "
                f"{len(values)} rows"
            )
        if not len(stamps):
            raise TrackError(
                "the recording is empty; nothing arrived, which is a failed run "
                "rather than a short one"
            )
        if not np.isfinite(values).all():
            raise TrackError("the recording carries a non-finite value")
        object.__setattr__(self, "timestamps_ns", stamps)
        object.__setattr__(self, "values", values)

    def __len__(self) -> int:
        return int(len(self.timestamps_ns))

    @property
    def is_monotonic(self) -> bool:
        return bool(np.all(np.diff(self.timestamps_ns) > 0))

    @property
    def largest_gap_ns(self) -> int:
        """The longest wait between consecutive samples, 0 if there is only one.

        What says whether messages were dropped. Interpolating across a gap is
        drawing a smooth line through data nobody measured, so the caller has to
        be able to see how long the longest one was.
        """
        if len(self) < 2:
            return 0
        return int(np.max(np.diff(self.timestamps_ns)))

    @property
    def median_period_ns(self) -> float:
        """The typical spacing, which is what a gap should be judged against."""
        if len(self) < 2:
            return 0.0
        return float(np.median(np.diff(self.timestamps_ns)))


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
