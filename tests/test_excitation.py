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
        position=0.0, jitter_rad=0.0, rng=None,
    ):
        self.deflection = deflection
        self.droop_rad = droop_rad
        self.band_rad = band_rad
        self.latch_rad = band_rad if latch_rad is None else latch_rad
        self.position = position
        self.jitter_rad = jitter_rad
        self.rng = rng
        self.held = None

    def settle(self, torque):
        """Where the torque leaves the joint. Driven by publishing, not by
        reading: a joint moves when it is pushed, whether or not anybody looks
        at it — which is the difference between a round and an arrival."""
        target = self.droop_rad - self.deflection(torque)
        if self.held is None:
            self.held = target + self.latch_rad
        elif abs(target - self.held) > self.band_rad:
            self.held = target + np.sign(self.held - target) * self.band_rad

    def reading(self):
        # Jitter on the reading, not on the latch: the joint is where it is,
        # the encoder is what is uncertain about it.
        if self.jitter_rad <= 0.0:
            return self.held
        return self.held + self.rng.normal(0.0, self.jitter_rad)


class _Arm:
    """The two-joint group of `_Group`, with `_Joint` as its first joint."""

    def __init__(self, joint):
        self.joint = joint
        self.effort = np.zeros(2)
        self.calls = []

    def publish(self, effort, seconds):
        self.effort = np.asarray(effort, dtype=float).copy()
        self.calls.append((self.effort, seconds))
        self.joint.settle(float(self.effort[0]))

    def read_state(self, timeout_sec=None):
        return np.full(2, self.joint.position)

    def read_tracking_error(self):
        return np.array([self.joint.reading(), 0.0])


def _drive(arm, *, deflection_rad=0.05, limit=None, steps=5, announce=None,
           noise_rad=0.0):
    """Drive the stub. *noise_rad* defaults to 0: these joints are exact, and a
    threshold derived from an encoder they do not have would only obscure what
    each test is about. `test_encoder_noise_...` passes the real figure."""
    return measure_staircase(
        arm, _Gate(), _Group(),
        joints=["r_aj_5"], limits=[limit or _Limit(), _Limit()],
        deflection_rad=deflection_rad, steps=steps, hold_sec=0.0,
        publish=arm.publish, noise_rad=noise_rad,
        **({} if announce is None else {"announce": announce}),
    )


def _fit(poses, applied, errors):
    """Fit what a run recorded, the way `identify` will."""
    from robot_control.identification import GravitySweep, fit_staircase

    return fit_staircase([
        GravitySweep(
            group="openarm_right_arm", joint_names=_Group.joints, poses=poses,
            modelled_torque=np.zeros_like(applied), scales=np.zeros_like(applied),
            applied_torque=applied, errors=errors,
        )
    ])


def _immovable(kp):
    """A joint whose stiction band no seed in the escalation can cross."""
    return _Joint(lambda torque: torque / kp, band_rad=100.0 / kp, latch_rad=0.0)


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


def test_the_loaded_shoulder_the_seed_cannot_break_loose_is_escalated_to():
    """r_aj_2 carrying a load: kp 63.7, 5 N.m of gravity, Fc 0.3 N.m. The
    0.05 N.m seed shifts its equilibrium by 0.00078 rad against a 0.0047 rad
    stiction band, so it does not move at all — and the droop hides that
    completely, reading +0.078 rad either way, which is what the probe used to
    size a staircase from.

    Reading the seed against a zero-torque baseline turns that into an honest
    refusal, and the escalation turns the refusal into a measurement: 0.4 N.m
    is the seed this joint needs. The true torque for 0.05 rad of real travel
    is kp*d + Fc = 3.485 N.m — the friction has to be broken as well as the
    spring bent — and the probe lands within a tenth of it.
    """
    arm = _Arm(
        _Joint(lambda torque: torque / 63.7, droop_rad=5.0 / 63.7,
               band_rad=0.3 / 63.7, latch_rad=0.0, position=0.5)
    )

    _poses, applied, _errors = _drive(arm, limit=_Limit(effort=40.0))

    published = [float(effort[0]) for effort, _seconds in arm.calls]
    assert published[:5] == [0.0, 0.05, 0.10, 0.20, 0.40]
    assert applied[:, 0].max() == pytest.approx(3.485, rel=0.1)


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


def test_the_seed_doubles_until_the_joint_breaks_loose():
    """A seed inside the stiction band moves the joint not at all, and the
    0.05 N.m seed is inside every band on this arm. Fc 0.12 N.m here (a shade
    over the 0.10 the review measured at the wrist, to keep the 0.10 seed off
    the knife edge): 0.05 and 0.10 leave it standing, 0.20 breaks it loose.

    0.40 follows as the confirming step: differencing the two readings that
    both moved cancels the Coulomb offset the first one carries on its own.
    The staircase then wants kp*d + Fc = 0.875 N.m, since a joint at rest has
    to have its friction broken as well as its spring bent to travel 0.05 rad.
    """
    arm = _Arm(
        _Joint(lambda torque: torque / 15.1, droop_rad=0.03 / 15.1,
               band_rad=0.12 / 15.1, latch_rad=0.0)
    )

    _poses, applied, _errors = _drive(arm, limit=_Limit(effort=10.0))

    published = [float(effort[0]) for effort, _seconds in arm.calls]
    assert published[:5] == [0.0, 0.05, 0.10, 0.20, 0.40]
    assert applied[:, 0].max() == pytest.approx(0.875, rel=0.05)


def test_encoder_noise_does_not_read_as_a_joint_that_moved():
    """One count of the 14-bit encoder is `DEFAULT_NOISE_RAD`, and it clears a
    1e-6 rad "did it move" test every time. This joint is stuck — r_aj_2's Fc
    0.30 is six times the seed — but on dither alone the escalation returns at
    the first seed and extrapolates a stiffness from noise.

    Two things then go wrong, and the second is the serious one. The escalation
    never runs, so every joint announces a 0.05 N.m seed and reports nothing
    about its stiction. And the failure is not fail-safe: a noise reading can
    size the probe publish anywhere up to a quarter of the joint's rating, 10
    N.m here, before any deflection has actually been measured. The re-check
    only corrects the torque after that one has gone out.
    """
    from robot_control.identification import DEFAULT_NOISE_RAD

    rng = np.random.default_rng(20260728)
    for _ in range(12):
        arm = _Arm(
            _Joint(lambda torque: torque / 63.7, droop_rad=5.0 / 63.7,
                   band_rad=0.3 / 63.7, latch_rad=0.0, position=0.5,
                   jitter_rad=DEFAULT_NOISE_RAD, rng=rng)
        )
        said = []

        _drive(arm, limit=_Limit(effort=40.0), announce=said.append,
               noise_rad=DEFAULT_NOISE_RAD)

        assert "0.05 N.m seed" not in said[0], said[0]
        loudest = max(abs(float(effort[0])) for effort, _seconds in arm.calls)
        assert loudest < 2 * 3.485, f"published {loudest:.3f} N.m"


def test_the_smallest_seed_that_moved_the_joint_is_announced():
    """It is the run's first measurement of the joint's stiction: the joint
    stood still at half this torque and moved at this one."""
    arm = _Arm(
        _Joint(lambda torque: torque / 15.1, droop_rad=0.03 / 15.1,
               band_rad=0.12 / 15.1, latch_rad=0.0)
    )
    said = []

    _drive(arm, limit=_Limit(effort=10.0), announce=said.append)

    assert len(said) == 1
    assert "r_aj_5" in said[0] and "0.2 N.m seed" in said[0]


def test_the_escalation_stops_at_the_ceiling_the_probe_itself_refuses_at():
    """The escalation may never ask for a torque `probe_torque` would reject,
    so it stops at the same 25% of the joint's rating. Rated 2 N.m, that
    ceiling is 0.5: the seed reaches 0.4 and the next double would clear it.

    The refusal names both, because a joint whose stiction is more than a
    quarter of its rating is a finding about the joint rather than a run to
    retry with a bigger number.
    """
    arm = _Arm(_immovable(15.1))

    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*0\.500 N\.m ceiling"):
        _drive(arm, limit=_Limit(effort=2.0))

    published = [float(effort[0]) for effort, _seconds in arm.calls]
    assert published[:5] == [0.0, 0.05, 0.10, 0.20, 0.40]


def test_the_escalation_is_capped_before_it_reaches_a_generous_ceiling():
    """Rated 40 N.m the ceiling is 10, which doubling would take a while to
    reach — and a joint that has not moved by the cap is not a stiff joint,
    it is one that is not listening. The cap is what stops a wiring fault
    walking a joint up to its rating one publish at a time.
    """
    from robot_control.excitation import MAX_SEED_DOUBLINGS

    arm = _Arm(_immovable(63.7))

    with pytest.raises(ExcitationRefused, match=r"r_aj_5.*doubling"):
        _drive(arm, limit=_Limit(effort=40.0))

    seeds = [float(effort[0]) for effort, _seconds in arm.calls if effort[0] > 0]
    assert len(seeds) == MAX_SEED_DOUBLINGS + 1
    assert seeds[-1] == pytest.approx(0.05 * 2**MAX_SEED_DOUBLINGS)


@pytest.mark.parametrize(
    "kp, coulomb, gravity, limit, position",
    [
        (63.7, 0.30, 5.0, _Limit(effort=40.0, lower=-0.175, upper=3.316), 0.5),
        (15.1, 0.10, 0.03, _Limit(effort=7.0), 0.0),
    ],
)
def test_what_a_run_records_inverts_back_to_the_joint_it_drove(
    kp, coulomb, gravity, limit, position
):
    """The whole point of the stage: the sweep must fit back to the joint.

    The probe leaves the joint held at +peak and the staircase starts at -peak,
    so the joint arrives at the first round travelling *downward* and sits on
    the descending equilibrium — one full 2 Fc/kp hysteresis gap off the
    ascending line the fitter reads it on, at the most extreme torque of the
    rising branch, where a point has the most leverage.

    Recording that arrival: kp 65.689 (+3.1%) and Fc 0.2652 (-11.6%) for the
    shoulder, 15.679 (+3.8%) and 0.0890 (-11.0%) for the wrist. Fc is what
    hdgp_export writes out as joint_friction, so the -11% lands in the number
    the run exists to produce. It is deterministic, and more steps do not wash
    it out.
    """
    arm = _Arm(
        _Joint(lambda torque, kp=kp: torque / kp, droop_rad=gravity / kp,
               band_rad=coulomb / kp, latch_rad=0.0, position=position)
    )

    estimate = _fit(*_drive(arm, limit=limit, steps=7))

    # r_aj_6 is in the group and was never driven, so it is legitimately
    # unidentified; the joint under test must not be.
    assert "r_aj_5" not in [name for name, _reason in estimate.unidentifiable]
    assert estimate.stiffness[0] == pytest.approx(kp, rel=1e-9)
    assert estimate.coulomb_nm[0] == pytest.approx(coulomb, rel=1e-9)


def test_the_arrival_at_the_first_torque_is_held_but_not_recorded():
    """A staircase of `steps` visits 2*steps - 1 torques and records one fewer,
    because the first is where the joint arrives from the probe rather than a
    round it was measured at. Both branches keep steps - 1 rounds, so the
    fitter's two-per-branch minimum still admits --steps 3.
    """
    arm = _Arm(_Joint(lambda torque: torque / 15.0))

    _poses, applied, _errors = _drive(arm, steps=5)

    assert len(applied) == 2 * 5 - 2
    published = [float(effort[0]) for effort, _seconds in arm.calls]
    # ... but it is published: the joint has to travel there.
    assert published.count(pytest.approx(-applied[:, 0].max())) == 2


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
    """A refusal leaves the last seed of the escalation published; the release
    is what takes it off, so it has to be held on that path too."""
    arm = _Arm(_immovable(63.7))

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
