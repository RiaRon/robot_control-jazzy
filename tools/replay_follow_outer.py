#!/usr/bin/env python3
"""Structurally replay the Follow outer candidate from stored diagnostics.

Recorded measured joints came from the old closed loop.  This tool therefore
compares one control-cycle candidate calculations only; it does not predict
the robot trajectory or the closed-loop performance of the changed law.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, NamedTuple

import numpy as np


# Historical offline model only.  The production Follow controller no longer
# imports or applies this bound after the outer-clamp revert.  Keeping the
# calculation here preserves reproducibility of the 2026-08-28 analysis.
OUTER_TARGET_SIGN_EPSILON_RAD = 1e-12


class _HistoricalOuterCandidateBounds(NamedTuple):
    candidate: np.ndarray
    clamp_mask: np.ndarray
    crossing_mask: np.ndarray
    target_hold_mask: np.ndarray
    outward_mask: np.ndarray
    stalled_recovery_mask: np.ndarray


def _historical_bound_outer_candidate(
    command: np.ndarray,
    ik_target: np.ndarray,
    raw_candidate: np.ndarray,
    *,
    epsilon_rad: float = OUTER_TARGET_SIGN_EPSILON_RAD,
) -> _HistoricalOuterCandidateBounds:
    """Replay the reverted clamp without exposing it to production code."""
    command_error = ik_target - command
    raw_error = ik_target - raw_candidate
    crossing_mask = (
        ((command_error > epsilon_rad) & (raw_error < -epsilon_rad))
        | ((command_error < -epsilon_rad) & (raw_error > epsilon_rad))
    )
    target_hold_mask = (
        (np.abs(command_error) <= epsilon_rad)
        & (np.abs(raw_error) > epsilon_rad)
    )
    outer_step = raw_candidate - command
    outward_mask = (
        ((command_error > epsilon_rad) & (outer_step < -epsilon_rad))
        | ((command_error < -epsilon_rad) & (outer_step > epsilon_rad))
    )
    stalled_recovery_mask = (
        (np.abs(command_error) > epsilon_rad)
        & (np.abs(outer_step) <= epsilon_rad)
    )
    clamp_mask = (
        crossing_mask
        | target_hold_mask
        | outward_mask
        | stalled_recovery_mask
    )
    return _HistoricalOuterCandidateBounds(
        candidate=np.where(clamp_mask, ik_target, raw_candidate),
        clamp_mask=clamp_mask,
        crossing_mask=crossing_mask,
        target_hold_mask=target_hold_mask,
        outward_mask=outward_mask,
        stalled_recovery_mask=stalled_recovery_mask,
    )


def _phase(sample: dict[str, Any]) -> str:
    diagnostic = sample.get("diagnostic_profile")
    if isinstance(diagnostic, dict) and diagnostic.get("phase"):
        return str(diagnostic["phase"])
    return str(sample.get("stage", "unlabeled"))


def _joint_vector(
    sample: dict[str, Any], field: str, joint_count: int
) -> np.ndarray | None:
    positions = sample.get("joint_positions_rad")
    if not isinstance(positions, dict):
        return None
    vector = np.asarray(positions.get(field), dtype=float)
    if vector.shape != (joint_count,) or not np.all(np.isfinite(vector)):
        return None
    return vector


def _limit_active(sample: dict[str, Any], kind: str) -> bool:
    limits = sample.get("limits", {})
    if kind.startswith("cartesian_"):
        return bool(limits.get(kind, False))
    joint = limits.get("joint", {})
    return bool(joint.get(kind, ()))


def summarize_outer_replay(payload: dict[str, Any]) -> dict[str, Any]:
    """Return deterministic one-step old/new candidate metrics."""
    joint_names = tuple(payload["joint_names"])
    joint_count = len(joint_names)
    settings = payload.get("settings", {})
    kp = float(settings.get("kp_per_sec", 2.0))
    nominal_dt = 1.0 / float(settings.get("command_rate_hz", 100.0))

    clamp_counts = np.zeros(joint_count, dtype=int)
    crossing_counts = np.zeros(joint_count, dtype=int)
    hold_counts = np.zeros(joint_count, dtype=int)
    outward_counts = np.zeros(joint_count, dtype=int)
    stalled_counts = np.zeros(joint_count, dtype=int)
    old_max_crossing = np.zeros(joint_count)
    old_error_sum = np.zeros(joint_count)
    new_error_sum = np.zeros(joint_count)
    old_error_max = np.zeros(joint_count)
    new_error_max = np.zeros(joint_count)
    phase_clamps: dict[str, int] = {}
    limiter_samples = {
        "cartesian_speed": 0,
        "cartesian_angular_speed": 0,
        "velocity": 0,
        "lead": 0,
        "position": 0,
    }
    compared_samples = 0
    unchanged_joint_updates = 0
    previous_time: float | None = None

    for sample in payload.get("trace", ()):
        command = _joint_vector(sample, "command", joint_count)
        target = _joint_vector(sample, "ik_target", joint_count)
        measured = _joint_vector(sample, "measured", joint_count)
        if command is None or target is None or measured is None:
            continue

        timestamp = float(
            sample.get("timestamp_sec", sample.get("elapsed_sec", np.nan))
        )
        dt = nominal_dt
        if previous_time is not None and np.isfinite(timestamp):
            observed_dt = timestamp - previous_time
            if observed_dt > 0.0 and np.isfinite(observed_dt):
                dt = observed_dt
        if np.isfinite(timestamp):
            previous_time = timestamp

        raw = command + kp * (target - measured) * dt
        bounded = _historical_bound_outer_candidate(command, target, raw)
        raw_error = np.abs(target - raw)
        bounded_error = np.abs(target - bounded.candidate)

        compared_samples += 1
        clamp_counts += bounded.clamp_mask.astype(int)
        crossing_counts += bounded.crossing_mask.astype(int)
        hold_counts += bounded.target_hold_mask.astype(int)
        outward_counts += bounded.outward_mask.astype(int)
        stalled_counts += bounded.stalled_recovery_mask.astype(int)
        old_max_crossing = np.maximum(
            old_max_crossing,
            np.where(bounded.crossing_mask, raw_error, 0.0),
        )
        old_error_sum += raw_error
        new_error_sum += bounded_error
        old_error_max = np.maximum(old_error_max, raw_error)
        new_error_max = np.maximum(new_error_max, bounded_error)
        unchanged_joint_updates += int(np.sum(~bounded.clamp_mask))
        phase = _phase(sample)
        phase_clamps[phase] = phase_clamps.get(phase, 0) + int(
            np.sum(bounded.clamp_mask)
        )
        for limiter in limiter_samples:
            limiter_samples[limiter] += int(_limit_active(sample, limiter))

    divisor = max(compared_samples, 1)
    per_joint = []
    for index, name in enumerate(joint_names):
        per_joint.append(
            {
                "name": name,
                "clamp_joint_events": int(clamp_counts[index]),
                "strict_crossing_joint_events": int(crossing_counts[index]),
                "target_hold_joint_events": int(hold_counts[index]),
                "outward_joint_events": int(outward_counts[index]),
                "stalled_recovery_joint_events": int(stalled_counts[index]),
                "old_max_crossing_rad": float(old_max_crossing[index]),
                "old_candidate_mean_abs_ik_error_rad": float(
                    old_error_sum[index] / divisor
                ),
                "new_candidate_mean_abs_ik_error_rad": float(
                    new_error_sum[index] / divisor
                ),
                "old_candidate_max_abs_ik_error_rad": float(
                    old_error_max[index]
                ),
                "new_candidate_max_abs_ik_error_rad": float(
                    new_error_max[index]
                ),
            }
        )

    return {
        "mode": "one_step_structural_replay_not_closed_loop_prediction",
        "schema_version": payload.get("schema_version"),
        "profile": payload.get("profile"),
        "compared_samples": compared_samples,
        "total_joint_updates": compared_samples * joint_count,
        "unchanged_joint_updates": unchanged_joint_updates,
        "outer_clamp_joint_events": int(np.sum(clamp_counts)),
        "old_strict_crossing_joint_events": int(np.sum(crossing_counts)),
        "new_strict_crossing_joint_events": 0,
        "old_max_crossing_rad": float(np.max(old_max_crossing, initial=0.0)),
        "new_max_crossing_rad": 0.0,
        "phase_clamp_joint_events": phase_clamps,
        "recorded_limiter_active_samples": limiter_samples,
        "per_joint": per_joint,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("json", nargs="+", type=Path)
    args = parser.parse_args()
    reports = []
    for path in args.json:
        payload = json.loads(path.read_text())
        reports.append({"source": str(path), **summarize_outer_replay(payload)})
    print(json.dumps(reports, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
