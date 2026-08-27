from __future__ import annotations

from typing import NamedTuple

import numpy as np


# This is a numerical sign-comparison tolerance in radians, not a physical
# tracking deadband or controller gain.  It only suppresses floating-point
# chatter when command and IK target are already equal to machine precision.
OUTER_TARGET_SIGN_EPSILON_RAD = 1e-12


class OuterCandidateBounds(NamedTuple):
    candidate: np.ndarray
    clamp_mask: np.ndarray
    crossing_mask: np.ndarray
    target_hold_mask: np.ndarray
    outward_mask: np.ndarray
    stalled_recovery_mask: np.ndarray


def bound_outer_candidate(
    command: np.ndarray,
    ik_target: np.ndarray,
    raw_candidate: np.ndarray,
    *,
    epsilon_rad: float = OUTER_TARGET_SIGN_EPSILON_RAD,
) -> OuterCandidateBounds:
    """Keep a measured-error outer update on the IK-target side it started.

    Inputs have already passed the Follow state/target validation path.  The
    returned candidate still has to pass the Cartesian, joint-velocity, lead,
    and position limiters; this function never produces the actuator command.

    A crossing uses the sign of ``ik_target - command`` and
    ``ik_target - raw_candidate`` independently for every joint.  When the
    command is already at the target, measured lag cannot restart integration
    beyond it.  If a moving target leaves stale command offset and the raw
    measured-error step points farther away, the IK target becomes the bounded
    recovery candidate so the existing downstream limiters resolve that offset.
    """
    command = np.asarray(command, dtype=float)
    ik_target = np.asarray(ik_target, dtype=float)
    raw_candidate = np.asarray(raw_candidate, dtype=float)

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
    # A zero raw step can otherwise strand stale command offset forever after
    # target reversal. Diagnose it separately from genuinely outward motion,
    # but use the same bounded recovery request; downstream limiters set the
    # actuator step in either case.
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
    candidate = np.where(clamp_mask, ik_target, raw_candidate)
    return OuterCandidateBounds(
        candidate=candidate,
        clamp_mask=clamp_mask,
        crossing_mask=crossing_mask,
        target_hold_mask=target_hold_mask,
        outward_mask=outward_mask,
        stalled_recovery_mask=stalled_recovery_mask,
    )
