"""Asynchronous latest-wins IK with bounded closest-solution selection."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, Sequence

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
    sequence: int
    requested_at_sec: float
    started_at_sec: float | None
    completed_at_sec: float | None
    accepted_at_sec: float | None
    outcome: str


@dataclass(frozen=True)
class IkContinuityRejection:
    sequence: int
    attempt: int
    rejected_at_sec: float
    reference: np.ndarray
    solution: np.ndarray
    joint_delta_rad: np.ndarray
    exhausted: bool


@dataclass(frozen=True)
class IkCandidate:
    sequence: int
    candidate: int
    outcome: str
    solution: np.ndarray | None
    joint_delta_rad: np.ndarray | None
    continuity_cost: float | None
    latency_sec: float


@dataclass(frozen=True)
class IkSelection:
    sequence: int
    candidates: tuple[IkCandidate, ...]
    selected_candidate: int | None
    selected_cost: float | None
    solve_latency_sec: float
    batch_latency_sec: float


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
    candidate_count: int
    rejected_candidate_count: int
    selections: tuple[IkSelection, ...]


def joint_delta(solution, reference, continuous_joints=None) -> np.ndarray:
    solution = np.asarray(solution, dtype=float)
    reference = np.asarray(reference, dtype=float)
    if solution.shape != reference.shape:
        raise ValueError("IK solution and reference must have the same shape")
    delta = solution - reference
    if continuous_joints is not None:
        continuous = np.asarray(continuous_joints, dtype=bool)
        if continuous.shape != delta.shape:
            raise ValueError("continuous_joints must match the IK solution")
        delta[continuous] = (delta[continuous] + np.pi) % (2.0 * np.pi) - np.pi
    return delta


def weighted_joint_distance(delta, weights=None) -> float:
    delta = np.asarray(delta, dtype=float)
    weight = np.ones_like(delta) if weights is None else np.asarray(weights, dtype=float)
    if weight.shape != delta.shape or np.any(~np.isfinite(weight)) or np.any(weight <= 0):
        raise ValueError("joint weights must be finite, positive, and match the solution")
    return float(np.sqrt(np.sum(weight * delta * delta)))


class LatestIkWorker:
    """Generate a bounded candidate batch off-loop and select its nearest solution."""

    def __init__(
        self,
        solve: Callable[[Pose, np.ndarray], np.ndarray],
        *,
        clock: Callable[[], float] = time.monotonic,
        max_target_jump_rad: float | None = None,
        max_continuity_attempts: int = 1,
        max_batch_latency_sec: float = 0.08,
        joint_weights: Sequence[float] | None = None,
        continuous_joints: Sequence[bool] | None = None,
    ):
        if max_target_jump_rad is not None and (
            not np.isfinite(max_target_jump_rad) or max_target_jump_rad <= 0.0
        ):
            raise ValueError("max_target_jump_rad must be finite and positive")
        if max_continuity_attempts < 1:
            raise ValueError("max_continuity_attempts must be at least one")
        if not np.isfinite(max_batch_latency_sec) or max_batch_latency_sec <= 0:
            raise ValueError("max_batch_latency_sec must be finite and positive")
        self._solve = solve
        self._clock = clock
        self._max_target_jump_rad = max_target_jump_rad
        self._max_candidates = int(max_continuity_attempts)
        self._max_batch_latency_sec = float(max_batch_latency_sec)
        self._joint_weights = None if joint_weights is None else np.asarray(joint_weights, dtype=float).copy()
        self._continuous_joints = None if continuous_joints is None else np.asarray(continuous_joints, dtype=bool).copy()
        self._condition = threading.Condition()
        self._pending = None
        self._sequence = 0
        self._target = None
        self._target_sequence = None
        self._submitted = self._succeeded = self._failed = self._superseded = 0
        self._solve_attempts = self._continuity_rejected = 0
        self._continuity_retries = self._continuity_exhausted = 0
        self._continuity_rejections = []
        self._continuity_refusal = None
        self._timings = {}
        self._selections = []
        self._closing = False
        self._thread = threading.Thread(target=self._run, name="robotctl-ik-follow", daemon=True)
        self._thread.start()

    def submit(self, pose: Pose, seed: np.ndarray) -> None:
        copied_seed = np.asarray(seed, dtype=float).copy()
        if not np.all(np.isfinite(copied_seed)):
            raise ValueError("IK seed must contain only finite values")
        copied_pose = Pose(tuple(float(v) for v in pose.position), tuple(float(v) for v in pose.orientation), pose.frame_id)
        if not np.all(np.isfinite(np.asarray(copied_pose.position + copied_pose.orientation))):
            raise ValueError("IK pose must contain only finite values")
        with self._condition:
            if self._closing:
                raise RuntimeError("IK worker is closed")
            self._sequence += 1
            if self._target is not None:
                copied_seed = self._target.copy()
            if self._pending is not None:
                self._timings[self._pending.sequence]["outcome"] = "superseded_before_start"
                self._superseded += 1
            requested = self._clock()
            self._pending = IkRequest(self._sequence, copied_pose, copied_seed, requested)
            self._timings[self._sequence] = {
                "requested_at_sec": requested,
                "started_at_sec": None,
                "completed_at_sec": None,
                "accepted_at_sec": None,
                "outcome": "pending",
            }
            self._submitted += 1
            self._condition.notify()

    def snapshot(self) -> IkStatus:
        with self._condition:
            return IkStatus(
                target=None if self._target is None else self._target.copy(),
                target_sequence=self._target_sequence,
                submitted=self._submitted,
                succeeded=self._succeeded,
                failed=self._failed,
                superseded=self._superseded,
                solve_attempts=self._solve_attempts,
                continuity_rejected=self._continuity_rejected,
                continuity_retries=self._continuity_retries,
                continuity_exhausted=self._continuity_exhausted,
                continuity_rejections=tuple(self._copy_rejection(e) for e in self._continuity_rejections),
                continuity_refusal=None if self._continuity_refusal is None else self._copy_rejection(self._continuity_refusal),
                timings=tuple(IkTiming(sequence=k, **v) for k, v in sorted(self._timings.items())),
                candidate_count=sum(len(batch.candidates) for batch in self._selections),
                rejected_candidate_count=self._continuity_rejected,
                selections=tuple(self._copy_selection(batch) for batch in self._selections),
            )

    @staticmethod
    def _copy_rejection(event):
        return IkContinuityRejection(
            event.sequence, event.attempt, event.rejected_at_sec,
            event.reference.copy(), event.solution.copy(), event.joint_delta_rad.copy(), event.exhausted,
        )

    @staticmethod
    def _copy_candidate(candidate):
        return IkCandidate(
            candidate.sequence, candidate.candidate, candidate.outcome,
            None if candidate.solution is None else candidate.solution.copy(),
            None if candidate.joint_delta_rad is None else candidate.joint_delta_rad.copy(),
            candidate.continuity_cost, candidate.latency_sec,
        )

    @classmethod
    def _copy_selection(cls, selection):
        return IkSelection(
            selection.sequence,
            tuple(cls._copy_candidate(c) for c in selection.candidates),
            selection.selected_candidate,
            selection.selected_cost,
            selection.solve_latency_sec,
            selection.batch_latency_sec,
        )

    def close(self) -> None:
        with self._condition:
            if self._closing:
                return
            self._closing = True
            if self._pending is not None:
                self._timings[self._pending.sequence]["outcome"] = "superseded_on_close"
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
                if self._target is not None:
                    request = IkRequest(
                        request.sequence,
                        request.pose,
                        self._target.copy(),
                        request.requested_at_sec,
                    )
                batch_started = self._clock()
                self._timings[request.sequence]["started_at_sec"] = batch_started
            candidates = []
            good = []
            rejections = []
            superseded = False
            for candidate_index in range(1, self._max_candidates + 1):
                if candidate_index > 1 and self._clock() - batch_started >= self._max_batch_latency_sec:
                    break
                solve_started = self._clock()
                try:
                    solution = np.asarray(self._solve(request.pose, request.seed), dtype=float).copy()
                    if solution.shape != request.seed.shape or not np.all(np.isfinite(solution)):
                        raise ValueError("IK solution must be finite and match the seed")
                    error = None
                except Exception as caught:
                    solution, error = None, caught
                completed = self._clock()
                with self._condition:
                    self._solve_attempts += 1
                    if request.sequence != self._sequence:
                        timing = self._timings[request.sequence]
                        timing["completed_at_sec"] = completed
                        timing["outcome"] = "superseded_after_complete"
                        self._superseded += 1
                        superseded = True
                        break
                if error is not None:
                    candidates.append(IkCandidate(request.sequence, candidate_index, "solve_failed", None, None, None, completed - solve_started))
                    continue
                delta = joint_delta(solution, request.seed, self._continuous_joints)
                cost = weighted_joint_distance(delta, self._joint_weights)
                discontinuous = self._max_target_jump_rad is not None and np.any(
                    np.abs(delta) >= self._max_target_jump_rad - 1e-12
                )
                if discontinuous:
                    rejection = IkContinuityRejection(
                        request.sequence, candidate_index, completed, request.seed.copy(), solution.copy(), delta.copy(), False
                    )
                    rejections.append(rejection)
                    candidates.append(IkCandidate(request.sequence, candidate_index, "continuity_rejected", solution, delta, cost, completed - solve_started))
                else:
                    candidates.append(IkCandidate(request.sequence, candidate_index, "eligible", solution, delta, cost, completed - solve_started))
                    good.append((cost, tuple(float(v) for v in solution), candidate_index, solution))
            if superseded:
                continue
            completed = self._clock()
            with self._condition:
                timing = self._timings[request.sequence]
                self._continuity_rejected += len(rejections)
                self._continuity_retries += max(0, len(candidates) - 1) if rejections else 0
                self._continuity_rejections.extend(rejections)
                selected_candidate = None
                selected_cost = None
                if good:
                    selected_cost, _lexicographic, selected_candidate, selected = min(good)
                    self._target = selected.copy()
                    self._target_sequence = request.sequence
                    self._continuity_refusal = None
                    timing["completed_at_sec"] = completed
                    timing["accepted_at_sec"] = completed
                    timing["outcome"] = "accepted"
                    self._succeeded += 1
                    candidates = [
                        IkCandidate(c.sequence, c.candidate, "selected" if c.candidate == selected_candidate else c.outcome, c.solution, c.joint_delta_rad, c.continuity_cost, c.latency_sec)
                        for c in candidates
                    ]
                elif rejections:
                    last = rejections[-1]
                    refusal = IkContinuityRejection(
                        last.sequence, len(candidates), completed, last.reference.copy(), last.solution.copy(), last.joint_delta_rad.copy(), True
                    )
                    self._continuity_rejections[-1] = refusal
                    self._continuity_refusal = refusal
                    self._continuity_exhausted += 1
                    timing["completed_at_sec"] = completed
                    timing["outcome"] = "continuity_refused"
                else:
                    self._failed += 1
                    timing["completed_at_sec"] = completed
                    timing["outcome"] = "failed"
                self._selections.append(IkSelection(
                    request.sequence,
                    tuple(candidates),
                    selected_candidate,
                    selected_cost,
                    sum(c.latency_sec for c in candidates),
                    completed - batch_started,
                ))
