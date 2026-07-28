"""Sizing the excitation torque, which cannot be computed in advance.

kp is the unknown the experiment is for, so the torque that produces a wanted
deflection is found by pushing a little and extrapolating. Everything here is
arithmetic on a measured response — no robot, no ROS — so it is tested directly
rather than through the CLI.
"""

import numpy as np
import pytest

from robot_control.excitation import ExcitationRefused, measure_staircase, probe_torque

BOUNDS = dict(
    effort_limit_nm=7.0, position_rad=0.0, lower_rad=-1.57, upper_rad=1.57,
    joint="r_aj_5",
)


def test_it_extrapolates_linearly_from_the_seed_response():
    # 0.05 N.m moved it 0.004 rad, so kp is 12.5 and 0.05 rad wants 0.625.
    torque = probe_torque(
        deflection_rad=0.05, seed_torque_nm=0.05, seed_deflection_rad=0.004, **BOUNDS
    )

    assert torque == pytest.approx(0.625)


def test_it_refuses_rather_than_clamps_at_the_effort_limit():
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*effort"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=1e-5, **BOUNDS
        )


def test_it_refuses_a_torque_over_the_probe_fraction_of_the_rating():
    """A near-zero seed deflection extrapolates to a huge torque. The joint is
    rated for it; the experiment is not, and asking for a joint's full rating
    is not a measurement anyone requested."""
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*25%"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=0.001, **BOUNDS
        )


def test_it_refuses_a_deflection_that_would_reach_a_stop():
    """A joint pressed into its stop reads as stiction, so the measurement
    would be of the stop rather than of the drive."""
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*limit"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05, seed_deflection_rad=0.004,
            effort_limit_nm=7.0, position_rad=1.54, lower_rad=-1.57,
            upper_rad=1.57, joint="r_aj_5",
        )


def test_it_refuses_a_seed_the_joint_did_not_respond_to():
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*did not move"):
        probe_torque(
            deflection_rad=0.05, seed_torque_nm=0.05,
            seed_deflection_rad=0.0, **BOUNDS
        )


def test_it_refuses_a_negative_deflection_towards_the_tighter_limit():
    """-0.25 rad at a pose with only 0.10 rad of real margin on the near
    side. The staircase this sizes a step for pushes the joint by this
    magnitude at both ends, so a signed deflection must not evade the room
    check that a positive one would trip."""
    with pytest.raises(ExcitationRefused, match=r"r_aj_5"):
        probe_torque(
            deflection_rad=-0.25, seed_torque_nm=0.05, seed_deflection_rad=0.05,
            effort_limit_nm=7.0, position_rad=-1.27, lower_rad=-1.57,
            upper_rad=1.57, joint="r_aj_5",
        )


@pytest.mark.parametrize("deflection_rad", [0.0, -0.05])
def test_it_refuses_a_non_positive_deflection_rad(deflection_rad):
    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*magnitude"):
        probe_torque(
            deflection_rad=deflection_rad, seed_torque_nm=0.05,
            seed_deflection_rad=0.004, **BOUNDS
        )


def test_the_staircase_climbs_and_comes_back_without_repeating_the_peak():
    """Both directions of travel, because the gap between the ascending and
    descending branches is what measures Coulomb friction. A single direction
    traces one branch and measures nothing about it."""
    from robot_control.excitation import staircase

    values = staircase(peak_nm=0.6, steps=4)

    assert values == pytest.approx([-0.6, -0.2, 0.2, 0.6, 0.2, -0.2, -0.6])
    assert len(values) == 2 * 4 - 1


def test_the_staircase_needs_at_least_two_points_per_branch():
    from robot_control.excitation import ExcitationRefused, staircase

    with pytest.raises(ExcitationRefused, match="steps"):
        staircase(peak_nm=0.6, steps=1)


class _Limit:
    def __init__(self, effort=7.0, lower=-1.57, upper=1.57):
        self.effort, self.lower, self.upper = effort, lower, upper


class _Group:
    joints = ("r_aj_5", "r_aj_6")
    name = "openarm_right_arm"


class _Gate:
    def authorize_effort(self, effort):
        return np.asarray(effort, dtype=float)


class _Joint:
    """A joint standing at a gravity droop, deflected from it by torque.

    *deflection* maps an applied torque to how far it carries the joint off
    that droop, *droop_rad* is the tracking error the position loop stands at
    with no feedforward torque at all, and *band_rad* is Fc/kp: the joint does
    not move until its equilibrium moves further than that.

    *latch_rad* is where inside that band the joint is sitting when it is first
    read, which decides what a small push does. Latched against the boundary
    (the default) it moves by exactly the elastic amount, band cancelling band.
    Latched in the middle, a push smaller than the band moves it not at all.

    The droop is the whole difficulty. It is in every reading, it is nothing to
    do with kp, and it is typically far larger than what a 0.05 N.m seed adds.
    """

    def __init__(
        self, deflection, *, droop_rad=0.0, band_rad=0.0, latch_rad=None,
        position=0.0,
    ):
        self.deflection = deflection
        self.droop_rad = droop_rad
        self.band_rad = band_rad
        self.latch_rad = band_rad if latch_rad is None else latch_rad
        self.position = position
        self.held = None

    def error(self, torque):
        target = self.droop_rad - self.deflection(torque)
        if self.held is None:
            self.held = target + self.latch_rad
        elif abs(target - self.held) > self.band_rad:
            self.held = target + np.sign(self.held - target) * self.band_rad
        return self.held


class _Arm:
    """The two-joint group of `_Group`, with `_Joint` as its first joint."""

    def __init__(self, joint):
        self.joint = joint
        self.effort = np.zeros(2)
        self.calls = []

    def publish(self, effort, seconds):
        self.effort = np.asarray(effort, dtype=float).copy()
        self.calls.append((self.effort, seconds))

    def read_state(self, timeout_sec=None):
        return np.full(2, self.joint.position)

    def read_tracking_error(self):
        return np.array([self.joint.error(float(self.effort[0])), 0.0])


def _drive(arm, *, deflection_rad=0.05, limit=None, steps=5):
    return measure_staircase(
        arm, _Gate(), _Group(),
        joints=["r_aj_5"], limits=[limit or _Limit(), _Limit()],
        deflection_rad=deflection_rad, steps=steps, hold_sec=0.0,
        publish=arm.publish,
    )


def _softens_beyond(knee, kp_below, kp_above):
    """A joint whose stiffness changes at *knee* N.m, so that extrapolating
    from a torque on one side of it lands somewhere else entirely."""

    def deflection(torque):
        within = min(abs(torque), knee)
        beyond = max(abs(torque) - knee, 0.0)
        return np.sign(torque) * (within / kp_below + beyond / kp_above)

    return deflection


def test_the_probe_sizes_the_torque_from_what_the_seed_itself_moved():
    """kp = 15, standing at a 0.2 N.m gravity droop. 0.05 rad wants 0.75 N.m.

    Handing the whole tracking error read under the seed to `probe_torque` —
    0.0101 rad, of which 0.0133 is droop and only -0.0033 is the seed — asks
    for 0.248 N.m instead, a third of the torque, which puts the staircase's
    step inside the joint's own stiction band.
    """
    arm = _Arm(
        _Joint(lambda torque: torque / 15.0, droop_rad=0.2 / 15.0,
               band_rad=0.001 / 15.0)
    )

    _poses, applied, _errors = _drive(arm)

    assert applied[:, 0].max() == pytest.approx(0.75, rel=1e-3)


def test_a_joint_the_seed_cannot_break_loose_is_refused():
    """r_aj_2 carrying a load: kp 63.7, 5 N.m of gravity, Fc 0.3 N.m. The seed
    shifts this joint's equilibrium by 0.00078 rad and its stiction band is
    0.0047 rad wide, so unless it happens to be latched against the edge the
    joint does not move at all and there is nothing to extrapolate from.

    The droop hides that completely: the tracking error reads +0.078 rad either
    way, which is what the probe used to size a staircase from.
    """
    arm = _Arm(
        _Joint(lambda torque: torque / 63.7, droop_rad=5.0 / 63.7,
               band_rad=0.3 / 63.7, latch_rad=0.0, position=0.5)
    )

    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*did not move"):
        _drive(arm, limit=_Limit(effort=40.0))


def test_the_probe_re_extrapolates_once_from_the_torque_it_probed_with():
    """Stiffness 15 N.m/rad up to 0.1 N.m and 30 beyond it. The seed sees only
    the soft part and extrapolates 0.75 N.m for 0.05 rad; that torque achieves
    0.0283 rad, so a second extrapolation from the achieved deflection asks for
    1.324 N.m. Once, not until it converges: a third pass would ask 1.394 and
    a joint whose stiffness keeps moving would never finish.
    """
    arm = _Arm(_Joint(_softens_beyond(0.1, 15.0, 30.0)))

    _poses, applied, _errors = _drive(arm)

    assert applied[:, 0].max() == pytest.approx(1.3235, rel=1e-3)


def test_the_torque_release_is_held_rather_than_published_once():
    """`_publish_for` republishes for the seconds it is handed, so that a
    dropped message cannot silently leave the arm on a stale torque. The
    release asked for zero seconds and so went out exactly once — the one
    publish whose loss actually matters, since the adapter tears the node down
    straight after it and the controller holds its last command forever.
    """
    from robot_control.excitation import RELEASE_HOLD_SEC

    arm = _Arm(_Joint(lambda torque: torque / 15.0))

    _drive(arm)

    effort, seconds = arm.calls[-1]
    assert effort.tolist() == [0.0, 0.0]
    assert seconds == RELEASE_HOLD_SEC
    assert RELEASE_HOLD_SEC > 0.0


def test_the_torque_is_released_even_when_the_probe_refuses():
    """A refusal leaves a seed torque published; the release is what takes it
    off, so it has to be held on that path too."""
    arm = _Arm(
        _Joint(lambda torque: torque / 63.7, droop_rad=5.0 / 63.7,
               band_rad=0.3 / 63.7, latch_rad=0.0, position=0.5)
    )

    with pytest.raises(ExcitationRefused):
        _drive(arm, limit=_Limit(effort=40.0))

    effort, seconds = arm.calls[-1]
    assert effort.tolist() == [0.0, 0.0]
    assert seconds > 0.0


def test_a_deflection_that_lands_inside_the_margin_is_refused():
    """`probe_torque` checks the deflection *asked for* against the room; what
    carries the joint into its stop is the deflection it actually gets. Here
    the joint goes slack past 0.1 N.m, so the 0.05 rad request measures out at
    1.31 rad against the 0.25 rad this pose has to spare.
    """
    arm = _Arm(_Joint(_softens_beyond(0.1, 15.0, 0.5)))

    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*margin"):
        _drive(arm, limit=_Limit(lower=-0.45, upper=0.45))
