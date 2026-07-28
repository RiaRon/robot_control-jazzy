"""Choosing the torque a stiffness measurement should publish.

The gravity excitation had nothing to choose: it published a fraction of a
torque the model handed it. Publishing a torque of our own means deciding how
big, and the joint's stiffness — the thing being measured — is what sets that.
So it is probed: push a little, read the response, extrapolate.
"""

from __future__ import annotations


#: Ceiling on the probe, as a fraction of the joint's rating. A seed that
#: barely moves extrapolates to an enormous torque; the joint is rated for it
#: and the experiment is not.
MAX_PROBE_FRACTION = 0.25
#: How close to a position limit the deflected joint may come. A joint pressed
#: into its stop is held by the stop, and reads as stiction.
MARGIN_RAD = 0.20
#: The first push, small enough to be safe on the stiffest joint in the arm.
SEED_TORQUE_NM = 0.05


class ExcitationRefused(ValueError):
    """A torque the experiment will not publish, naming the bound it broke."""


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
    room = min(position_rad - lower_rad, upper_rad - position_rad) - MARGIN_RAD
    if abs(deflection_rad) > room:
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
