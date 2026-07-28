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

from .identification import DEFAULT_NOISE_RAD

#: Ceiling on the probe, as a fraction of the joint's rating. A seed that
#: barely moves extrapolates to an enormous torque; the joint is rated for it
#: and the experiment is not.
MAX_PROBE_FRACTION = 0.25
#: How close to a position limit the deflected joint may come. A joint pressed
#: into its stop is held by the stop, and reads as stiction.
MARGIN_RAD = 0.20
#: The first push, small enough to be safe on the stiffest joint in the arm.
#: It is also smaller than the dry friction of every joint on it (0.10 N.m at
#: the wrist, 0.30 at the shoulder), so on its own it moves nothing: a joint
#: latched inside its stiction band stays there until the seed exceeds Fc. So
#: the seed doubles until the joint moves, rather than being a flag — the
#: number an operator would have to guess is Fc, which is what the run exists
#: to measure. The escalation measures a bound on it instead: a joint sitting
#: anywhere in its band moves once the seed passes `Fc - c*kp`, which is 2 Fc
#: when it is latched at the far edge and 0 at the near one, so a seed that
#: failed puts a floor under Fc at half itself and a seed that succeeded says
#: nothing on its own. What is reported is therefore a floor, not a bracket:
#: the joint stood still under half the announced seed, so Fc is at least a
#: quarter of it. It also means the same joint at the same pose can want a
#: different seed depending on which way it was last moved.
SEED_TORQUE_NM = 0.05
#: How many times the seed may double. Bounded so that a joint which is not
#: listening at all — unpowered, braked, miswired — cannot be walked up to its
#: rating one publish at a time while it reads as very stiff. Five doublings
#: reach 1.6 N.m, which covers Fc up to 0.8 N.m for a joint latched at the far
#: edge of its band and up to 1.6 for one latched at the near edge. Every joint
#: on this arm sits at Fc <= 0.30.
MAX_SEED_DOUBLINGS = 5
#: A divide-by-zero guard, and nothing more: below this the extrapolation
#: `deflection * seed / response` has no denominator worth the name. It is far
#: under any real encoder's noise, so it must not be used to decide whether a
#: joint moved — `MOTION_NOISE_MARGIN` on the measured noise floor is what
#: decides that.
MOVED_RAD = 1e-6
#: How many times the encoder's noise a reading must change by before the
#: escalation calls it motion. The test is on the difference of two readings,
#: whose noise is sqrt(2) times one reading's, so this is 3.5 sigma on the
#: quantity actually being tested — under one false "it moved" in two thousand
#: seeds. It has to be this conservative: a seed that reads as motion when the
#: joint is stuck extrapolates a stiffness from pure noise, and the torque that
#: comes out is published before anything has measured a deflection.
#:
#: The cost is a joint whose real motion is under the floor escalating one more
#: step, which is the conservative direction.
MOTION_NOISE_MARGIN = 5.0
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
    if abs(seed_deflection_rad) < MOVED_RAD:
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


def _effort(width: int, index: int, value: float) -> np.ndarray:
    """A whole-group effort with *value* on one joint and zero on the rest."""
    effort = np.zeros(width)
    effort[index] = value
    return effort


def _hold_and_read(adapter, gate, publish, effort, hold_sec, index) -> float:
    """Publish *effort*, hold it for *hold_sec* so the joint settles, read it.

    The baseline and the probe are held as long as a staircase round is: a
    reading taken before the joint has stopped moving is a reading of the
    transient, and the difference between two of them is what sizes the run.
    """
    publish(gate.authorize_effort(effort), hold_sec)
    return float(adapter.read_tracking_error()[index])


def _seed_response(
    adapter, gate, publish, *,
    index, width, effort_limit_nm, hold_sec, joint, baseline, noise_rad,
) -> tuple[float, float]:
    """Find a seed the joint moves under, and the deflection it is worth.

    Returns the smallest seed that moved the joint and, for that same torque
    taken as an increment, the deflection it produced.

    Two stages, and the second is not optional. The escalation doubles from
    `SEED_TORQUE_NM` until the joint moves, comparing against *baseline*, the
    reading at zero torque: a seed that failed to move the joint left it
    exactly where the baseline found it, so one baseline stands for every step.
    But the seed that finally moves it is by construction only just past Fc,
    and the deflection it shows is `(seed - Fc)/kp` rather than `seed/kp` — a
    small difference of two close numbers, which extrapolates to an enormous
    torque. On the loaded shoulder (kp 63.7, Fc 0.3) the 0.4 N.m seed that
    breaks it loose asks for 12.74 N.m against a true 3.19.

    So the seed is doubled once more and the two readings are differenced. Both
    were taken moving the same way, so the Coulomb offset is the same in both
    and cancels, leaving `seed/kp` exactly — the same trick as reading against
    a zero-torque baseline to cancel gravity, one level down.
    """
    ceiling_nm = MAX_PROBE_FRACTION * effort_limit_nm

    def at_ceiling(seed_nm: float) -> str:
        return (
            f"{2.0 * seed_nm:g} N.m, over the {ceiling_nm:.3f} N.m ceiling the "
            f"probe refuses at ({MAX_PROBE_FRACTION:.0%} of its "
            f"{effort_limit_nm:g} N.m rating)"
        )

    def refuse_stuck(seed_nm: float) -> ExcitationRefused:
        return ExcitationRefused(
            f"{joint} held still under seeds up to {seed_nm:g} N.m and the "
            f"next would be {at_ceiling(seed_nm)}. Its dry friction is more "
            "than a quarter of what the joint is rated for, which is a finding "
            "about the joint rather than a run to retry with a bigger number"
        )

    def refuse_unconfirmed(seed_nm: float) -> ExcitationRefused:
        # Distinct from `refuse_stuck` because both of that message's claims
        # would be false here: this joint moved, and its friction can be well
        # under a quarter of the rating and still land on this branch.
        return ExcitationRefused(
            f"{joint} moved under {seed_nm:g} N.m, but sizing its staircase "
            f"needs a second reading at {at_ceiling(seed_nm)}. The first "
            "reading alone carries the joint's whole dry friction, and "
            "extrapolating from it divides by (seed - Fc), so there is no "
            "measurement to be had at this pose: raise the joint's rating or "
            "accept that this one cannot be probed"
        )

    def read(torque_nm: float) -> float:
        effort = np.zeros(width)
        effort[index] = torque_nm
        return _hold_and_read(adapter, gate, publish, effort, hold_sec, index)

    # Never below MOVED_RAD, so `--noise 0` still guards the division.
    moved_rad = max(MOTION_NOISE_MARGIN * noise_rad, MOVED_RAD)
    seed_nm = SEED_TORQUE_NM
    for _ in range(MAX_SEED_DOUBLINGS + 1):
        reading = read(seed_nm)
        broke_loose = abs(reading - baseline) >= moved_rad
        if 2.0 * seed_nm > ceiling_nm:
            # Whether the joint moved or not: the confirming step is as much a
            # part of the escalation as a retry is, and neither may publish
            # what the probe would reject. Which of the two happened decides
            # what the operator is told, not whether this refuses.
            raise refuse_unconfirmed(seed_nm) if broke_loose else refuse_stuck(seed_nm)
        if broke_loose:
            return seed_nm, read(2.0 * seed_nm) - reading
        seed_nm *= 2.0
    raise ExcitationRefused(
        f"{joint} moved less than {moved_rad:.2g} rad under every seed from "
        f"{SEED_TORQUE_NM:g} to {seed_nm / 2.0:g} N.m — {MAX_SEED_DOUBLINGS} "
        f"doublings, which is MAX_SEED_DOUBLINGS in excitation.py and the only "
        "place that number can be raised. A joint that has not moved by then "
        "is usually not a stiff joint but one that is not listening: check "
        "that its controller is running, that it is not braked, and that the "
        f"effort command is reaching it. If it is genuinely that stiff, its Fc "
        f"is above {seed_nm / 4.0:g} N.m and this arm cannot probe it"
    )


def _probe_peak(
    adapter, gate, publish, *,
    index, width, limit, deflection_rad, hold_sec, joint, noise_rad,
) -> tuple[float, float]:
    """Size the staircase for one joint, and report the seed that moved it.

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
    seed_nm, response = _seed_response(
        adapter, gate, publish,
        index=index, width=width, effort_limit_nm=limit.effort,
        hold_sec=hold_sec, joint=joint, baseline=baseline, noise_rad=noise_rad,
    )

    bounds = dict(
        deflection_rad=deflection_rad,
        effort_limit_nm=limit.effort,
        position_rad=position,
        lower_rad=limit.lower,
        upper_rad=limit.upper,
        joint=joint,
    )
    # The seed that moved it, not the one the escalation started from.
    peak = probe_torque(
        seed_torque_nm=seed_nm, seed_deflection_rad=response, **bounds
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
        return peak, seed_nm
    # Once, and then it is used. This extrapolation starts from a deflection
    # near the one wanted rather than from a seed the width of a stiction band,
    # so checking it in turn would be a loop with nothing to make it terminate.
    # A seed the escalation had to raise is exactly the case that lands here:
    # its response under-reports the elastic deflection by the whole band.
    return probe_torque(
        seed_torque_nm=peak, seed_deflection_rad=achieved, **bounds
    ), seed_nm


def measure_staircase(
    adapter, gate, group, *, joints, limits, deflection_rad, steps, hold_sec,
    publish, announce=lambda _line: None, noise_rad=DEFAULT_NOISE_RAD,
):
    """Drive each joint through its staircase and collect the rounds.

    *publish* is a callable taking (effort, seconds) and *announce* one taking
    a line of text; the caller owns how a torque is held and where a line goes,
    so this stays free of ROS and of wall-clock policy. *noise_rad* is the
    encoder's own noise, the same figure `identify` takes, and it is what
    decides whether a seed moved the joint at all.

    Every joint but the one under test receives zero. The vendor hardware runs
    MIT mode, so the position loop is still holding the whole arm while one
    joint is pushed against it; nothing here commands a position.
    """
    width = len(group.joints)
    poses, applied, errors = [], [], []
    try:
        for name in joints:
            index = group.joints.index(name)
            peak, seed_nm = _probe_peak(
                adapter, gate, publish,
                index=index, width=width, limit=limits[index],
                deflection_rad=deflection_rad, hold_sec=hold_sec, joint=name,
                noise_rad=noise_rad,
            )
            # The seed is worth saying out loud: the joint stood still under
            # half of it, so its dry friction is at least a quarter of what is
            # printed (see SEED_TORQUE_NM), and the ratio between joints is the
            # first look at the arm's stiction a run gives.
            announce(
                f"  {name}: moved under a {seed_nm:g} N.m seed, staircase peak "
                f"{peak:.3f} N.m"
            )
            values = staircase(peak, steps)
            # The first torque is where the joint arrives, not a round it was
            # measured at. `_probe_peak` leaves it held at +peak, so it reaches
            # -peak travelling downward and settles on the descending
            # equilibrium — a full 2 Fc/kp below the ascending line, at the
            # rising branch's most extreme torque. A point at -peak can only be
            # reached by descending, so there is no ordering of the staircase
            # that makes this first visit a rising-branch measurement; it is
            # published and held like any other, and then not recorded.
            #
            # Every round that is recorded is therefore reached moving the way
            # the branch it lands in says, which is the invariant the fitter's
            # single-argmax split assumes and cannot check for itself.
            publish(gate.authorize_effort(_effort(width, index, values[0])), hold_sec)
            for value in values[1:]:
                effort = gate.authorize_effort(_effort(width, index, value))
                publish(effort, hold_sec)
                poses.append(adapter.read_state())
                applied.append(effort)
                errors.append(adapter.read_tracking_error())
    finally:
        publish(np.zeros(width), RELEASE_HOLD_SEC)
    return np.asarray(poses), np.asarray(applied), np.asarray(errors)
