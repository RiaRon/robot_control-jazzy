"""Measured-state handoff gates, time bases, and diagnostic statistics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


# The 2026-08-24 real startup tail (last 0.5 s) had p95 values of 3.99 mm,
# 0.0201 rad, 0.0520 rad IK-to-command, 0.0583 rad command-to-measured, and
# 0.00077 rad measured sample change.  These bounds include that observed
# noise/backlog without changing any motion or continuity limit.
HANDOFF_POSITION_ERROR_M = 0.005
HANDOFF_ORIENTATION_ERROR_RAD = 0.035
HANDOFF_IK_TO_COMMAND_ERROR_RAD = 0.060
HANDOFF_COMMAND_TO_MEASURED_ERROR_RAD = 0.060
HANDOFF_MEASURED_SAMPLE_DELTA_RAD = 0.002
HANDOFF_STABLE_WINDOW_SEC = 0.5
HANDOFF_TIMEOUT_SEC = 5.0


@dataclass(frozen=True)
class ConvergenceObservation:
    marker_position_error_m: float
    marker_orientation_error_rad: float
    ik_to_command_max_error_rad: float
    command_to_measured_max_error_rad: float
    measured_sample_delta_max_rad: float

    def as_dict(self) -> dict:
        return {
            "marker_position_error_m": float(self.marker_position_error_m),
            "marker_orientation_error_rad": float(
                self.marker_orientation_error_rad
            ),
            "ik_to_command_max_error_rad": float(
                self.ik_to_command_max_error_rad
            ),
            "command_to_measured_max_error_rad": float(
                self.command_to_measured_max_error_rad
            ),
            "measured_sample_delta_max_rad": float(
                self.measured_sample_delta_max_rad
            ),
        }


class HandoffConvergenceGate:
    """Require every handoff metric to stay bounded for a finite window."""

    def __init__(
        self,
        *,
        started_sec: float,
        stable_window_sec: float | None = None,
        timeout_sec: float | None = None,
    ):
        self.started_sec = float(started_sec)
        self.stable_window_sec = float(
            HANDOFF_STABLE_WINDOW_SEC
            if stable_window_sec is None else stable_window_sec
        )
        self.timeout_sec = float(
            HANDOFF_TIMEOUT_SEC if timeout_sec is None else timeout_sec
        )
        if not math.isfinite(self.started_sec):
            raise ValueError("convergence gate start must be finite")
        if self.stable_window_sec <= 0.0 or self.timeout_sec <= 0.0:
            raise ValueError("convergence gate windows must be positive")
        if self.stable_window_sec > self.timeout_sec:
            raise ValueError("stable window must not exceed convergence timeout")
        self.stable_since_sec: float | None = None
        self.last: ConvergenceObservation | None = None
        self.samples = 0
        self.stable_samples = 0

    @staticmethod
    def thresholds() -> dict:
        return {
            "marker_position_error_m": HANDOFF_POSITION_ERROR_M,
            "marker_orientation_error_rad": HANDOFF_ORIENTATION_ERROR_RAD,
            "ik_to_command_max_error_rad": HANDOFF_IK_TO_COMMAND_ERROR_RAD,
            "command_to_measured_max_error_rad": (
                HANDOFF_COMMAND_TO_MEASURED_ERROR_RAD
            ),
            "measured_sample_delta_max_rad": (
                HANDOFF_MEASURED_SAMPLE_DELTA_RAD
            ),
            "stable_window_sec": HANDOFF_STABLE_WINDOW_SEC,
            "timeout_sec": HANDOFF_TIMEOUT_SEC,
        }

    def update(
        self, now_sec: float, observation: ConvergenceObservation
    ) -> tuple[bool, bool]:
        now = float(now_sec)
        values = np.asarray(list(observation.as_dict().values()), dtype=float)
        if not math.isfinite(now) or not np.all(np.isfinite(values)):
            raise ValueError("convergence observations must be finite")
        if now < self.started_sec:
            raise ValueError("convergence observation precedes gate start")
        self.last = observation
        self.samples += 1
        stable = (
            observation.marker_position_error_m <= HANDOFF_POSITION_ERROR_M
            and observation.marker_orientation_error_rad
            <= HANDOFF_ORIENTATION_ERROR_RAD
            and observation.ik_to_command_max_error_rad
            <= HANDOFF_IK_TO_COMMAND_ERROR_RAD
            and observation.command_to_measured_max_error_rad
            <= HANDOFF_COMMAND_TO_MEASURED_ERROR_RAD
            and observation.measured_sample_delta_max_rad
            <= HANDOFF_MEASURED_SAMPLE_DELTA_RAD
        )
        if stable:
            self.stable_samples += 1
            if self.stable_since_sec is None:
                self.stable_since_sec = now
        else:
            self.stable_since_sec = None
            self.stable_samples = 0
        passed = (
            self.stable_since_sec is not None
            and now - self.stable_since_sec >= self.stable_window_sec
        )
        timed_out = now - self.started_sec >= self.timeout_sec and not passed
        return passed, timed_out


def canonical_profile_phase(phase: str | None) -> str | None:
    if not phase or phase == "complete":
        return None
    if phase.endswith("_ramp_out") or phase == "combined_ramp_out":
        return "ramp"
    if phase.endswith("_hold") and phase != "origin_hold":
        return "hold"
    if phase.endswith("_ramp_back") or phase == "combined_ramp_back":
        return "return"
    if phase == "origin_hold":
        return "origin_hold"
    return phase


def stage_for_sample(
    *,
    alignment_complete: bool,
    gate_complete: bool,
    profile_phase: str | None,
) -> str:
    if not alignment_complete:
        return "startup_alignment"
    if not gate_complete:
        return "convergence_gate"
    canonical = canonical_profile_phase(profile_phase)
    return "profile_" + (canonical or "origin_hold")


def _percentile(values: np.ndarray, fraction: float) -> float | None:
    if values.size == 0:
        return None
    ordered = np.sort(values)
    location = fraction * (ordered.size - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return float(ordered[lower])
    weight = location - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def scalar_statistics(values: Iterable[float]) -> dict:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {
            "samples": 0,
            "mean": None,
            "rms": None,
            "max": None,
            "p95": None,
            "final_residual": None,
        }
    return {
        "samples": int(array.size),
        "mean": float(np.mean(array)),
        "rms": float(np.sqrt(np.mean(array * array))),
        "max": float(np.max(array)),
        "p95": _percentile(array, 0.95),
        "final_residual": float(array[-1]),
    }


def _joint_statistics(
    samples: list[dict],
    tolerance_rad: float,
    *,
    target_key: str = "ik_target",
) -> list[dict]:
    if not samples:
        return []
    times = np.asarray([sample["timestamp_sec"] for sample in samples], dtype=float)
    signed = np.asarray(
        [
            np.asarray(sample["joint_positions_rad"][target_key], dtype=float)
            - np.asarray(sample["joint_positions_rad"]["measured"], dtype=float)
            for sample in samples
        ]
    )
    result = []
    for index in range(signed.shape[1]):
        error = signed[:, index]
        absolute = np.abs(error)
        initial_sign = float(np.sign(error[0]))
        overshoot = (
            0.0
            if initial_sign == 0.0
            else float(np.max(np.maximum(-initial_sign * error, 0.0)))
        )
        settling = None
        within = absolute <= tolerance_rad
        for sample_index in range(within.size):
            if bool(np.all(within[sample_index:])):
                settling = float(times[sample_index] - times[0])
                break
        result.append(
            {
                "joint_index": index,
                "mean_abs_rad": float(np.mean(absolute)),
                "rms_rad": float(np.sqrt(np.mean(error * error))),
                "max_abs_rad": float(np.max(absolute)),
                "final_rad": float(error[-1]),
                "overshoot_rad": overshoot,
                "settling_time_sec": settling,
            }
        )
    return result


def window_statistics(
    trace: list[dict],
    *,
    joint_tolerance_rad: float,
    stage_trace: list[dict] | None = None,
) -> dict:
    """Return stage-separated metrics; profile-only is the comparison default."""
    windows = {
        "overall_run": lambda sample: True,
        "ready_reacquisition": lambda sample: sample.get("stage")
        == "ready_reacquisition",
        "handoff_alignment": lambda sample: sample.get("stage")
        in {"handoff_sync", "startup_alignment"},
        "convergence_gate": lambda sample: sample.get("stage")
        == "convergence_gate",
        "profile_only": lambda sample: str(sample.get("stage", "")).startswith(
            "profile_"
        ),
        "ramp": lambda sample: sample.get("stage") == "profile_ramp",
        "hold": lambda sample: sample.get("stage") == "profile_hold",
        "return": lambda sample: sample.get("stage") == "profile_return",
        "origin_hold": lambda sample: sample.get("stage")
        == "profile_origin_hold",
    }
    output = {"comparison_default": "profile_only", "windows": {}}
    for name, predicate in windows.items():
        ready_samples = []
        if name == "ready_reacquisition" and stage_trace is not None:
            ready_samples = [
                sample
                for sample in stage_trace
                if sample.get("stage") == "ready_reacquisition"
                and sample.get("joint_positions_rad") is not None
            ]
        selected = [
            sample
            for sample in trace
            if predicate(sample)
            and sample.get("position_error_m") is not None
        ]
        position = [
            sample["position_error_m"]["live_marker_to_measured"]
            for sample in selected
        ]
        orientation = [
            sample["orientation_error_rad"]["live_marker_to_measured"]
            for sample in selected
        ]
        limiter_names = (
            "cartesian_speed",
            "cartesian_angular_speed",
        )
        limiters = {}
        for limiter in limiter_names:
            count = sum(bool(sample["limits"].get(limiter)) for sample in selected)
            limiters[limiter] = {
                "samples": int(count),
                "ratio": None if not selected else float(count / len(selected)),
            }
        for limiter in (
            "velocity", "acceleration", "lead", "position",
            "position_lower", "position_upper",
        ):
            count = sum(
                bool(sample["limits"].get("joint", {}).get(limiter))
                for sample in selected
            )
            limiters[f"joint_{limiter}"] = {
                "samples": int(count),
                "ratio": None if not selected else float(count / len(selected)),
                "availability": (
                    "unavailable"
                    if limiter == "acceleration"
                    else "recorded"
                ),
            }
        joint_samples = ready_samples if ready_samples else selected
        output["windows"][name] = {
            "samples": len(joint_samples) if ready_samples else len(selected),
            "tcp_samples": len(selected),
            "tcp_translation_m": scalar_statistics(position),
            "tcp_orientation_rad": scalar_statistics(orientation),
            "joints": _joint_statistics(
                joint_samples,
                joint_tolerance_rad,
                target_key="command" if ready_samples else "ik_target",
            ),
            "joint_error_basis": (
                "command_to_measured" if ready_samples else "ik_to_measured"
            ),
            "limiters": limiters,
        }
    return output


def build_timeline(
    *,
    stage_trace: list[dict],
    trace: list[dict],
    reacquisition: dict | None,
    handoff_sync: dict | None,
    alignment: dict,
    convergence: dict,
    diagnostic: dict,
    cleanup: dict,
    termination: str,
    termination_elapsed_sec: float,
) -> dict:
    """Build one run-relative event/sample clock without changing legacy time."""
    events = [
        {"name": "run_start", "timestamp_sec": 0.0, "sample_index": None},
    ]
    if stage_trace:
        events.append(
            {
                "name": "trace_start",
                "timestamp_sec": stage_trace[0]["timestamp_sec"],
                "sample_index": stage_trace[0]["sample_index"],
            }
        )

    def add(name, timestamp, sample_index=None):
        if timestamp is not None:
            events.append(
                {
                    "name": name,
                    "timestamp_sec": float(timestamp),
                    "sample_index": sample_index,
                }
            )

    if reacquisition:
        add("ready_reacquisition_start", reacquisition.get("started_elapsed_sec"))
        add("ready_reacquisition_end", reacquisition.get("completed_elapsed_sec"))
    if handoff_sync:
        add(
            "handoff_sync",
            handoff_sync.get("elapsed_sec"),
            handoff_sync.get("sample_index"),
        )
    add("alignment_start", alignment.get("started_elapsed_sec"))
    add("alignment_end", alignment.get("completed_elapsed_sec"))
    add("convergence_gate_start", convergence.get("started_elapsed_sec"))
    add("convergence_gate_end", convergence.get("completed_elapsed_sec"))
    add("profile_start", diagnostic.get("started_elapsed_sec"))

    phase_groups: list[tuple[str, list[dict]]] = []
    for canonical in ("ramp", "hold", "return", "origin_hold"):
        selected = [
            sample
            for sample in trace
            if sample.get("stage") == f"profile_{canonical}"
        ]
        phase_groups.append((canonical, selected))
    for phase, selected in phase_groups:
        if selected:
            add(f"profile_{phase}_start", selected[0]["timestamp_sec"], selected[0]["sample_index"])
            add(f"profile_{phase}_end", selected[-1]["timestamp_sec"], selected[-1]["sample_index"])
    profile_samples = [
        sample for sample in trace
        if str(sample.get("stage", "")).startswith("profile_")
    ]
    if profile_samples:
        add("profile_end", profile_samples[-1]["timestamp_sec"], profile_samples[-1]["sample_index"])
    add("cleanup_start", cleanup.get("started_elapsed_sec"))
    add("cleanup_end", cleanup.get("completed_elapsed_sec"))
    add("termination", termination_elapsed_sec)
    events.sort(key=lambda event: event["timestamp_sec"])
    return {
        "clock": "monotonic_run_elapsed_sec",
        "run_start_timestamp_sec": 0.0,
        "trace_start_timestamp_sec": (
            None if not stage_trace else stage_trace[0]["timestamp_sec"]
        ),
        "termination": termination,
        "termination_timestamp_sec": float(termination_elapsed_sec),
        "events": events,
        "sample_index_to_timestamp": [
            {
                "sample_index": int(sample["sample_index"]),
                "timestamp_sec": float(sample["timestamp_sec"]),
                "stage": sample["stage"],
                "trace_index": sample.get("trace_index"),
            }
            for sample in stage_trace
        ],
    }
