"""Choosing the torque a stiffness measurement should publish.

The gravity excitation had nothing to choose: it published a fraction of a
torque the model handed it. Publishing a torque of our own means deciding how
big, and the joint's stiffness — the thing being measured — is what sets that.
So it is probed: read where the joint stands under no torque, push a little,
and extrapolate from the difference. The standing reading itself is the
controller's droop, mostly gravity, and says nothing about kp.
"""

from __future__ import annotations

import numpy as np

#: Ceiling on the probe, as a fraction of the joint's rating. A seed that
#: barely moves extrapolates to an enormous torque; the joint is rated for it
#: and the experiment is not.
MAX_PROBE_FRACTION = 0.25
#: How close to a position limit the deflected joint may come. A joint pressed
#: into its stop is held by the stop, and reads as stiction.
MARGIN_RAD = 0.20
#: The first push, small enough to be safe on the stiffest joint in the arm.
SEED_TORQUE_NM = 0.05
#: How far the deflection the probed torque actually achieved may miss the one
#: asked for before the probe extrapolates a second time, from that better
#: measurement. A linear spring hits the target exactly; a miss means the seed
#: was read where the joint does not behave the way it does at the staircase's
#: ends. Re-extrapolating costs no extra motion — the reading it starts from
#: was taken anyway, to check this.
RECHECK_TOLERANCE = 0.10
#: How long the zero torque at the end of a run is held. Every other publish
#: is repeated for the time it is held, so that one dropped message cannot
#: leave the arm on a stale torque; the release is the publish where that
#: matters most, because the node is torn down straight after it and the
#: controller goes on holding whatever it last received.
RELEASE_HOLD_SEC = 0.25


class ExcitationRefused(ValueError):
    """A torque the experiment will not publish, naming the bound it broke."""


def staircase(peak_nm: float, steps: int) -> list[float]:
    """Torques from -peak to +peak and back, one list, in visiting order.

    Both directions because a joint held by dry friction sits at a different
    equilibrium depending on which way it last moved, and the distance between
    those two equilibria is the measurement.
    """
    if steps < 2:
        raise ExcitationRefused(f"a staircase needs at least 2 steps, got {steps}")
    span = 2.0 * peak_nm
    up = [-peak_nm + span * index / (steps - 1) for index in range(steps)]
    return up + up[-2::-1]


def _room_to_stop(position_rad: float, lower_rad: float, upper_rad: float) -> float:
    """How far the joint may be deflected before it comes within MARGIN_RAD of
    a stop. The staircase deflects to both sides of the pose, so the tighter of
    the two sides binds regardless of which side a given step is headed for."""
    return min(position_rad - lower_rad, upper_rad - position_rad) - MARGIN_RAD


def probe_torque(
    *,
    deflection_rad: float,
    seed_torque_nm: float,
    seed_deflection_rad: float,
    effort_limit_nm: float,
    position_rad: float,
    lower_rad: float,
    upper_rad: float,
    joint: str,
) -> float:
    """Size the torque that should produce ``deflection_rad``.

    ``deflection_rad`` is a magnitude, not a signed offset: the staircase this
    sizes a step for pushes the joint by that amount at both ends of the
    sweep, so both position limits bind on it regardless of which way a given
    step goes.
    """
    if deflection_rad <= 0:
        raise ExcitationRefused(
            f"{joint} was asked for a {deflection_rad:g} rad deflection; "
            "deflection_rad is a magnitude and must be positive"
        )
    if abs(seed_deflection_rad) < 1e-6:
        raise ExcitationRefused(
            f"{joint} did not move under {seed_torque_nm:g} N.m, so its "
            "stiffness cannot be extrapolated; raise the seed torque"
        )
    wanted = abs(deflection_rad * seed_torque_nm / seed_deflection_rad)

    # Position first: a pose with no room is a pose problem, not a torque one.
    room = _room_to_stop(position_rad, lower_rad, upper_rad)
    if deflection_rad > room:
        raise ExcitationRefused(
            f"{joint} has {room:.3f} rad to a position limit before its "
            f"{MARGIN_RAD:g} rad margin, less than the {deflection_rad:g} rad "
            "deflection asked for; move to a pose with more room"
        )
    # Raw rating next: over this, the torque isn't a probe, it's a fault.
    if wanted > effort_limit_nm:
        raise ExcitationRefused(
            f"{joint} would need {wanted:.3f} N.m for {deflection_rad:g} rad, "
            f"over its {effort_limit_nm:g} N.m effort limit; ask for a "
            "smaller deflection"
        )
    # 25% ceiling last: the tighter bound, so it is the one most probes hit.
    ceiling = MAX_PROBE_FRACTION * effort_limit_nm
    if wanted > ceiling:
        raise ExcitationRefused(
            f"{joint} would need {wanted:.3f} N.m for {deflection_rad:g} rad, "
            f"over {MAX_PROBE_FRACTION:.0%} of its {effort_limit_nm:g} N.m "
            "rating; ask for a smaller deflection"
        )
    return wanted


def _hold_and_read(adapter, gate, publish, effort, hold_sec, index) -> float:
    """Publish *effort*, hold it for *hold_sec* so the joint settles, read it.

    The baseline and the probe are held as long as a staircase round is: a
    reading taken before the joint has stopped moving is a reading of the
    transient, and the difference between two of them is what sizes the run.
    """
    publish(gate.authorize_effort(effort), hold_sec)
    return float(adapter.read_tracking_error()[index])


def _probe_peak(
    adapter, gate, publish, *,
    index, width, limit, deflection_rad, hold_sec, joint,
) -> float:
    """Size the staircase for one joint: seed it, extrapolate, re-check once.

    Every deflection here is a difference from the reading at zero torque.
    A tracking error on its own is the controller's droop — (tau_gravity -
    tau_applied)/kp plus stiction — which is dominated by a load that has
    nothing to do with the seed; only what changed when the seed arrived says
    anything about kp.
    """
    zero = np.zeros(width)
    baseline = _hold_and_read(adapter, gate, publish, zero, hold_sec, index)
    # Read where the arm stands, not where the torque has carried it: the room
    # to a stop is a property of the pose the operator chose.
    position = float(adapter.read_state()[index])
    seed = np.zeros(width)
    seed[index] = SEED_TORQUE_NM
    response = _hold_and_read(adapter, gate, publish, seed, hold_sec, index) - baseline

    bounds = dict(
        deflection_rad=deflection_rad,
        effort_limit_nm=limit.effort,
        position_rad=position,
        lower_rad=limit.lower,
        upper_rad=limit.upper,
        joint=joint,
    )
    peak = probe_torque(
        seed_torque_nm=SEED_TORQUE_NM, seed_deflection_rad=response, **bounds
    )

    probe = np.zeros(width)
    probe[index] = peak
    achieved = abs(
        _hold_and_read(adapter, gate, publish, probe, hold_sec, index) - baseline
    )
    room = _room_to_stop(position, limit.lower, limit.upper)
    if achieved > room:
        raise ExcitationRefused(
            f"{joint} moved {achieved:.3f} rad under {peak:.3f} N.m, past the "
            f"{room:.3f} rad it has before its {MARGIN_RAD:g} rad margin. The "
            f"{deflection_rad:g} rad asked for fits; the joint is softer than "
            "the seed said it was, so ask for a smaller deflection"
        )
    if abs(achieved - deflection_rad) <= RECHECK_TOLERANCE * deflection_rad:
        return peak
    # Once, and then it is used. This extrapolation starts from a deflection
    # near the one wanted rather than from a seed the width of a stiction band,
    # so checking it in turn would be a loop with nothing to make it terminate.
    return probe_torque(
        seed_torque_nm=peak, seed_deflection_rad=achieved, **bounds
    )


def measure_staircase(
    adapter, gate, group, *, joints, limits, deflection_rad, steps, hold_sec, publish
):
    """Drive each joint through its staircase and collect the rounds.

    *publish* is a callable taking (effort, seconds); the caller owns how a
    torque is held, so this stays free of ROS and of wall-clock policy.

    Every joint but the one under test receives zero. The vendor hardware runs
    MIT mode, so the position loop is still holding the whole arm while one
    joint is pushed against it; nothing here commands a position.
    """
    width = len(group.joints)
    poses, applied, errors = [], [], []
    try:
        for name in joints:
            index = group.joints.index(name)
            peak = _probe_peak(
                adapter, gate, publish,
                index=index, width=width, limit=limits[index],
                deflection_rad=deflection_rad, hold_sec=hold_sec, joint=name,
            )
            for value in staircase(peak, steps):
                effort = np.zeros(width)
                effort[index] = value
                effort = gate.authorize_effort(effort)
                publish(effort, hold_sec)
                poses.append(adapter.read_state())
                applied.append(effort)
                errors.append(adapter.read_tracking_error())
    finally:
        publish(np.zeros(width), RELEASE_HOLD_SEC)
    return np.asarray(poses), np.asarray(applied), np.asarray(errors)
