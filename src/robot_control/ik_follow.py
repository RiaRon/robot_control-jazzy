"""Asynchronous, latest-wins MoveIt IK target calculation."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable

import numpy as np

from .ros_adapter import Pose


@dataclass(frozen=True)
class IkRequest:
    sequence: int
    pose: Pose
    seed: np.ndarray
    requested_at_sec: float


@dataclass(frozen=True)
class IkTiming:
    """Lifecycle timestamps for one latest-wins IK request."""

    sequence: int
    requested_at_sec: float
    started_at_sec: float | None
    completed_at_sec: float | None
    accepted_at_sec: float | None
    outcome: str


@dataclass(frozen=True)
class IkStatus:
    target: np.ndarray | None
    target_sequence: int | None
    submitted: int
    succeeded: int
    failed: int
    superseded: int
    timings: tuple[IkTiming, ...]


class LatestIkWorker:
    """Run blocking IK without letting stale results replace newer goals."""

    def __init__(
        self,
        solve: Callable[[Pose, np.ndarray], np.ndarray],
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._solve = solve
        self._clock = clock
        self._condition = threading.Condition()
        self._pending: IkRequest | None = None
        self._sequence = 0
        self._target: np.ndarray | None = None
        self._target_sequence: int | None = None
        self._submitted = 0
        self._succeeded = 0
        self._failed = 0
        self._superseded = 0
        self._timings: dict[int, dict[str, float | str | None]] = {}
        self._closing = False
        self._thread = threading.Thread(
            target=self._run, name="robotctl-ik-follow", daemon=True
        )
        self._thread.start()

    def submit(self, pose: Pose, seed: np.ndarray) -> None:
        copied_seed = np.asarray(seed, dtype=float).copy()
        if not np.all(np.isfinite(copied_seed)):
            raise ValueError("IK seed must contain only finite values")
        copied_pose = Pose(
            tuple(float(value) for value in pose.position),
            tuple(float(value) for value in pose.orientation),
            pose.frame_id,
        )
        if not np.all(
            np.isfinite(np.asarray(copied_pose.position + copied_pose.orientation))
        ):
            raise ValueError("IK pose must contain only finite values")
        with self._condition:
            if self._closing:
                raise RuntimeError("IK worker is closed")
            self._sequence += 1
            if self._pending is not None:
                self._timings[self._pending.sequence]["outcome"] = (
                    "superseded_before_start"
                )
                self._superseded += 1
            requested_at = self._clock()
            self._pending = IkRequest(
                self._sequence,
                copied_pose,
                copied_seed,
                requested_at,
            )
            self._timings[self._sequence] = {
                "requested_at_sec": requested_at,
                "started_at_sec": None,
                "completed_at_sec": None,
                "accepted_at_sec": None,
                "outcome": "pending",
            }
            self._submitted += 1
            self._condition.notify()

    def snapshot(self) -> IkStatus:
        with self._condition:
            target = None if self._target is None else self._target.copy()
            return IkStatus(
                target=target,
                target_sequence=self._target_sequence,
                submitted=self._submitted,
                succeeded=self._succeeded,
                failed=self._failed,
                superseded=self._superseded,
                timings=tuple(
                    IkTiming(sequence=sequence, **values)
                    for sequence, values in sorted(self._timings.items())
                ),
            )

    def close(self) -> None:
        with self._condition:
            if self._closing:
                return
            self._closing = True
            if self._pending is not None:
                self._timings[self._pending.sequence]["outcome"] = (
                    "superseded_on_close"
                )
                self._pending = None
                self._superseded += 1
            self._condition.notify()
        self._thread.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closing:
                    self._condition.wait()
                if self._closing:
                    return
                request = self._pending
                self._pending = None
                self._timings[request.sequence]["started_at_sec"] = (
                    self._clock()
                )
            try:
                solution = np.asarray(
                    self._solve(request.pose, request.seed), dtype=float
                ).copy()
                if not np.all(np.isfinite(solution)):
                    raise ValueError("IK solution must contain only finite values")
                error = None
            except Exception as caught:  # The control loop must retain its last target.
                solution = None
                error = caught
            with self._condition:
                completed_at = self._clock()
                timing = self._timings[request.sequence]
                timing["completed_at_sec"] = completed_at
                if request.sequence != self._sequence:
                    timing["outcome"] = "superseded_after_complete"
                    self._superseded += 1
                elif error is not None:
                    timing["outcome"] = "failed"
                    self._failed += 1
                else:
                    self._target = solution
                    self._target_sequence = request.sequence
                    timing["accepted_at_sec"] = completed_at
                    timing["outcome"] = "accepted"
                    self._succeeded += 1
