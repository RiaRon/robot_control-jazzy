"""Canonical deterministic-experiment starting posture."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


RIGHT_ARM_GROUP = "openarm_right_arm"
READY_POSTURE_NAME = "openarm_right_ready_v1"
READY_TARGET_RAD = np.array([0.15, 0.55, 0.15, 0.8, -0.1, 0.15, 0.1])

# GenericSystem reaches the target to floating-point precision. The previous
# real baselines reported a last maximum joint error of 0.0063 rad; 0.02 rad is
# more than three times that settled error while still small against the 0.30
# rad continuity boundary and the selected posture's 0.635 rad limit margin.
READY_TOLERANCE_RAD = 0.02
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


def ready_metadata(check: ReadyCheck | None = None) -> dict:
    return {
        "name": READY_POSTURE_NAME,
        "target_rad": READY_TARGET_RAD.tolist(),
        "tolerance_rad": READY_TOLERANCE_RAD,
        "actual_start_rad": None if check is None else check.actual.tolist(),
        "start_error_rad": None if check is None else check.error.tolist(),
        "passed": None if check is None else check.passed,
    }
