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
class IkContinuityRejection:
    """One IK solution rejected before it can replace the held target."""

    sequence: int
    attempt: int
    rejected_at_sec: float
    reference: np.ndarray
    solution: np.ndarray
    joint_delta_rad: np.ndarray
    exhausted: bool


@dataclass(frozen=True)
class IkStatus:
    target: np.ndarray | None
    target_sequence: int | None
    submitted: int
    succeeded: int
    failed: int
    superseded: int
    solve_attempts: int
    continuity_rejected: int
    continuity_retries: int
    continuity_exhausted: int
    continuity_rejections: tuple[IkContinuityRejection, ...]
    continuity_refusal: IkContinuityRejection | None
    timings: tuple[IkTiming, ...]


class LatestIkWorker:
    """Run blocking IK without letting stale results replace newer goals."""

    def __init__(
        self,
        solve: Callable[[Pose, np.ndarray], np.ndarray],
        *,
        clock: Callable[[], float] = time.monotonic,
        max_target_jump_rad: float | None = None,
        max_continuity_attempts: int = 1,
    ):
        if max_target_jump_rad is not None and (
            not np.isfinite(max_target_jump_rad)
            or max_target_jump_rad <= 0.0
        ):
            raise ValueError("max_target_jump_rad must be finite and positive")
        if max_continuity_attempts < 1:
            raise ValueError("max_continuity_attempts must be at least one")
        self._solve = solve
        self._clock = clock
        self._max_target_jump_rad = max_target_jump_rad
        self._max_continuity_attempts = int(max_continuity_attempts)
        self._condition = threading.Condition()
        self._pending: IkRequest | None = None
        self._sequence = 0
        self._target: np.ndarray | None = None
        self._target_sequence: int | None = None
        self._submitted = 0
        self._succeeded = 0
        self._failed = 0
        self._superseded = 0
        self._solve_attempts = 0
        self._continuity_rejected = 0
        self._continuity_retries = 0
        self._continuity_exhausted = 0
        self._continuity_rejections: list[IkContinuityRejection] = []
        self._continuity_refusal: IkContinuityRejection | None = None
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
            if self._max_target_jump_rad is not None and self._target is not None:
                copied_seed = self._target.copy()
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
                solve_attempts=self._solve_attempts,
                continuity_rejected=self._continuity_rejected,
                continuity_retries=self._continuity_retries,
                continuity_exhausted=self._continuity_exhausted,
                continuity_rejections=tuple(
                    IkContinuityRejection(
                        sequence=event.sequence,
                        attempt=event.attempt,
                        rejected_at_sec=event.rejected_at_sec,
                        reference=event.reference.copy(),
                        solution=event.solution.copy(),
                        joint_delta_rad=event.joint_delta_rad.copy(),
                        exhausted=event.exhausted,
                    )
                    for event in self._continuity_rejections
                ),
                continuity_refusal=(
                    None
                    if self._continuity_refusal is None
                    else IkContinuityRejection(
                        sequence=self._continuity_refusal.sequence,
                        attempt=self._continuity_refusal.attempt,
                        rejected_at_sec=(
                            self._continuity_refusal.rejected_at_sec
                        ),
                        reference=self._continuity_refusal.reference.copy(),
                        solution=self._continuity_refusal.solution.copy(),
                        joint_delta_rad=(
                            self._continuity_refusal.joint_delta_rad.copy()
                        ),
                        exhausted=self._continuity_refusal.exhausted,
                    )
                ),
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
            attempt = 0
            last_discontinuous: IkContinuityRejection | None = None
            while attempt < self._max_continuity_attempts:
                attempt += 1
                try:
                    solution = np.asarray(
                        self._solve(request.pose, request.seed), dtype=float
                    ).copy()
                    if not np.all(np.isfinite(solution)):
                        raise ValueError(
                            "IK solution must contain only finite values"
                        )
                    error = None
                except Exception as caught:
                    # A failed solve never replaces the previously held target.
                    solution = None
                    error = caught
                with self._condition:
                    self._solve_attempts += 1
                    completed_at = self._clock()
                    timing = self._timings[request.sequence]
                    if request.sequence != self._sequence:
                        timing["completed_at_sec"] = completed_at
                        timing["outcome"] = "superseded_after_complete"
                        self._superseded += 1
                        break
                    if error is not None:
                        if last_discontinuous is not None:
                            if attempt < self._max_continuity_attempts:
                                self._continuity_retries += 1
                                continue
                            refusal = IkContinuityRejection(
                                sequence=request.sequence,
                                attempt=attempt,
                                rejected_at_sec=completed_at,
                                reference=last_discontinuous.reference.copy(),
                                solution=last_discontinuous.solution.copy(),
                                joint_delta_rad=(
                                    last_discontinuous.joint_delta_rad.copy()
                                ),
                                exhausted=True,
                            )
                            timing["completed_at_sec"] = completed_at
                            timing["outcome"] = "continuity_refused"
                            self._continuity_exhausted += 1
                            self._continuity_refusal = refusal
                            break
                        timing["completed_at_sec"] = completed_at
                        timing["outcome"] = "failed"
                        self._failed += 1
                        break

                    joint_delta = solution - request.seed
                    discontinuous = (
                        self._max_target_jump_rad is not None
                        and np.any(
                            np.abs(joint_delta)
                            >= self._max_target_jump_rad
                        )
                    )
                    if discontinuous:
                        exhausted = attempt >= self._max_continuity_attempts
                        rejection = IkContinuityRejection(
                            sequence=request.sequence,
                            attempt=attempt,
                            rejected_at_sec=completed_at,
                            reference=request.seed.copy(),
                            solution=solution.copy(),
                            joint_delta_rad=joint_delta.copy(),
                            exhausted=exhausted,
                        )
                        self._continuity_rejections.append(rejection)
                        self._continuity_rejected += 1
                        last_discontinuous = rejection
                        if exhausted:
                            timing["completed_at_sec"] = completed_at
                            timing["outcome"] = "continuity_refused"
                            self._continuity_exhausted += 1
                            self._continuity_refusal = rejection
                            break
                        self._continuity_retries += 1
                        continue

                    self._target = solution
                    self._target_sequence = request.sequence
                    self._continuity_refusal = None
                    timing["completed_at_sec"] = completed_at
                    timing["accepted_at_sec"] = completed_at
                    timing["outcome"] = "accepted"
                    self._succeeded += 1
                    break
