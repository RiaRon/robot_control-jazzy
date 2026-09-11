from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# This is only a floating-point comparison tolerance. It is deliberately the
# same scale already used by the streaming limiter and is not a physical joint
# deadband.
IK_TARGET_CHANGE_ATOL_RAD = 1e-12


@dataclass(frozen=True)
class OuterCommandRequest:
    """One outer-loop request before Cartesian and joint limiting."""

    raw_candidate: np.ndarray
    pre_limiter_target: np.ndarray
    used_ik_target_mask: np.ndarray
    target_changed_release_mask: np.ndarray


class PostCrossingTargetHold:
    """Switch crossed joints to the IK target on the following cycle.

    Crossing is observed only after the caller has applied every existing
    limiter. The crossing command is never changed here. On later cycles,
    held joints request the latest IK target as their pre-limiter target until
    that joint's accepted IK target changes by more than floating-point noise.
    """

    def __init__(
        self,
        joint_count: int,
        *,
        target_change_atol_rad: float = IK_TARGET_CHANGE_ATOL_RAD,
    ) -> None:
        self._hold_mask = np.zeros(joint_count, dtype=bool)
        self._held_target = np.full(joint_count, np.nan, dtype=float)
        self._target_change_atol_rad = float(target_change_atol_rad)

    @property
    def hold_mask(self) -> np.ndarray:
        return self._hold_mask.copy()

    def request(
        self,
        command: np.ndarray,
        measured: np.ndarray,
        ik_target: np.ndarray,
        kp_per_sec: float,
        elapsed_sec: float,
    ) -> OuterCommandRequest:
        command = np.asarray(command, dtype=float)
        measured = np.asarray(measured, dtype=float)
        ik_target = np.asarray(ik_target, dtype=float)

        target_changed = self._hold_mask & (
            np.abs(ik_target - self._held_target)
            > self._target_change_atol_rad
        )
        self._hold_mask[target_changed] = False
        self._held_target[target_changed] = np.nan

        used_ik_target = self._hold_mask.copy()
        raw_candidate = command + (
            kp_per_sec * (ik_target - measured) * elapsed_sec
        )
        pre_limiter_target = np.where(
            used_ik_target,
            ik_target,
            raw_candidate,
        )
        return OuterCommandRequest(
            raw_candidate=raw_candidate,
            pre_limiter_target=pre_limiter_target,
            used_ik_target_mask=used_ik_target,
            target_changed_release_mask=target_changed,
        )

    def observe_published_command(
        self,
        command_before: np.ndarray,
        published_command: np.ndarray,
        ik_target: np.ndarray,
    ) -> np.ndarray:
        """Latch joints whose actual post-limiter command crossed the target."""

        command_before = np.asarray(command_before, dtype=float)
        published_command = np.asarray(published_command, dtype=float)
        ik_target = np.asarray(ik_target, dtype=float)
        crossing = crossing_mask(command_before, published_command, ik_target)
        self._hold_mask[crossing] = True
        self._held_target[crossing] = ik_target[crossing]
        return crossing


def crossing_mask(
    before: np.ndarray,
    after: np.ndarray,
    target: np.ndarray,
) -> np.ndarray:
    """Observe strict sign changes around *target* without changing control."""

    error_before = np.asarray(target, dtype=float) - np.asarray(
        before, dtype=float
    )
    error_after = np.asarray(target, dtype=float) - np.asarray(
        after, dtype=float
    )
    return (
        ((error_before > 0.0) & (error_after < 0.0))
        | ((error_before < 0.0) & (error_after > 0.0))
    )
