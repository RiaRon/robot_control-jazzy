"""Canonical deterministic-experiment starting posture."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .follow_observability import (
    HANDOFF_MEASURED_SAMPLE_DELTA_RAD,
    HANDOFF_STABLE_WINDOW_SEC,
)


RIGHT_ARM_GROUP = "openarm_right_arm"
READY_POSTURE_NAME = "openarm_right_ready_v2"
READY_D_LEGACY_NAME = "openarm_right_ready_v1"
READY_TARGET_RAD = np.array([0.0, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0])
READY_D_TARGET_RAD = np.array([0.15, 0.55, 0.15, 0.8, -0.1, 0.15, 0.1])
READY_POSTURES = {
    READY_POSTURE_NAME: READY_TARGET_RAD,
    READY_D_LEGACY_NAME: READY_D_TARGET_RAD,
}

# GenericSystem reaches the target to floating-point precision. The previous
# real baselines reported a last maximum joint error of 0.0063 rad; 0.02 rad is
# more than three times that settled error while still small against the
# v2 posture's 0.375 rad limit margin.
READY_TOLERANCE_RAD = 0.02
# Keep the pose-ready acceptance criterion above unchanged. Deterministic
# follow has a separate handoff criterion because gravity-compensated hardware
# feedback can remain just outside 0.02 rad even while the controller reference
# is exactly A-prime. The 2026-08-24 run's pre-cleanup worst error was
# 0.047051 rad; the 0.05 rad follow-only bound is validated around A-prime in
# docs/pose-follow-ready-handoff-2026-08-24.md.
FOLLOW_REACQUISITION_TOLERANCE_RAD = 0.05
# Follow Ready no longer treats the target-error tolerance above as an
# accuracy requirement.  It remains useful for deciding whether the bounded
# A-prime move is needed and for legacy diagnostics.  The final measured state
# may be adopted only inside this separate safety neighbourhood.  The
# 0.060-rad bound includes the observed stationary J4 residual (0.0532 rad)
# plus encoder noise margin.
FOLLOW_READY_SAFE_NEIGHBORHOOD_RAD = 0.060
# Reuse the measured-sample bound and dwell already validated for the Cartesian
# handoff tail.  At 100 Hz, 0.002 rad/sample is conservative against the
# 2026-08-24 stationary-tail p95 of 0.00077 rad/sample, and 0.5 s requires 50
# consecutive quiet samples without changing the controller rate or gains.
FOLLOW_READY_MAX_MEASURED_SAMPLE_DELTA_RAD = (
    HANDOFF_MEASURED_SAMPLE_DELTA_RAD
)
FOLLOW_READY_STATIONARY_DWELL_SEC = HANDOFF_STABLE_WINDOW_SEC
READY_SPEED_RAD_S = 0.10
READY_ACCELERATION_RAD_S2 = 0.10
READY_SETTLE_TIMEOUT_SEC = 5.0
READY_SETTLE_WINDOW_SEC = 0.5


@dataclass(frozen=True)
class ReadyCheck:
    actual: np.ndarray
    error: np.ndarray
    passed: bool


def check_ready(
    actual: Sequence[float],
    *,
    target: Sequence[float] = READY_TARGET_RAD,
    tolerance_rad: float = READY_TOLERANCE_RAD,
) -> ReadyCheck:
    actual_array = np.asarray(actual, dtype=float).copy()
    target_array = np.asarray(target, dtype=float)
    if actual_array.shape != target_array.shape:
        raise ValueError(
            f"ready posture needs {target_array.size} joints, got {actual_array.size}"
        )
    if not np.all(np.isfinite(actual_array)):
        raise ValueError("ready posture check received non-finite joint state")
    error = actual_array - target_array
    return ReadyCheck(
        actual=actual_array,
        error=error,
        passed=bool(np.all(np.abs(error) <= tolerance_rad)),
    )


def ready_target(name: str) -> np.ndarray:
    try:
        return READY_POSTURES[name].copy()
    except KeyError as error:
        raise ValueError(
            f"unknown ready posture {name!r}; choose from {sorted(READY_POSTURES)}"
        ) from error


def ready_metadata(
    check: ReadyCheck | None = None,
    *,
    name: str = READY_POSTURE_NAME,
    target: Sequence[float] | None = None,
    tolerance_rad: float = READY_TOLERANCE_RAD,
) -> dict:
    selected = ready_target(name) if target is None else np.asarray(target, dtype=float)
    return {
        "name": name,
        "target_rad": selected.tolist(),
        "tolerance_rad": float(tolerance_rad),
        "actual_start_rad": None if check is None else check.actual.tolist(),
        "start_error_rad": None if check is None else check.error.tolist(),
        "passed": None if check is None else check.passed,
    }
