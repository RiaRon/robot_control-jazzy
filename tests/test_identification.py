from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from robot_control.identification import (
    FitError,
    GravitySweep,
    build_excitation,
    fit_second_order,
    fit_staircase,
    fit_static_gravity,
    split_repetitions,
    validate_holdout,
)
from robot_control.profile import load_builtin_profile

REAL_SWEEPS = Path(__file__).parent / "data" / "sweeps"
#: Measured on the right arm, 2026-07-28, and recorded here because a
#: generalised regression that changes them is a regression, not a refactor.
EXPECTED_STIFFNESS = {
    "r_aj_1": 67.23, "r_aj_2": 63.73, "r_aj_3": 89.86, "r_aj_4": 70.26,
    "r_aj_5": 15.09, "r_aj_6": 11.30, "r_aj_7": 13.40,
}


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def test_excitation_contains_step_ramp_hold_and_multisine():
    time, command, phases = build_excitation(
        neutral=np.array([0.0, 0.2]), amplitude=np.array([0.2, 0.1]), rate_hz=100
    )

    assert command.shape == (len(time), 2)
    assert {"step", "ramp", "hold", "multisine"} <= set(phases)
    assert np.max(np.abs(command[:, 0])) <= 0.2 + 1e-12


class TestSlewLimitedExcitation:
    """An excitation the arm can actually be commanded through.

    The phases are designed as shapes — hold, step, ramp, multisine — and the
    joins between them are discontinuities. Published as position commands at
    100 Hz, the join into the multisine asks for 7x the profile's velocity limit
    on the real arm, so the gate refuses the whole track. Shrinking the amplitude
    until the discontinuity fits would shrink the excitation to nothing; bridging
    the joins at the limit keeps the amplitude and costs a few samples.
    """

    RATE = 100.0
    NEUTRAL = np.array([0.0, 0.2])
    # What the real profile gives at the default --amplitude-scale: 5% of a
    # +-3.14 and a +-2.0 joint's range, at 0.3. The budget below is the arms'
    # 2.0 rad/s over one 100 Hz period.
    AMPLITUDE = np.array([0.0942, 0.06])

    def _build(self, max_step=None):
        return build_excitation(
            self.NEUTRAL, self.AMPLITUDE, self.RATE, max_step=max_step
        )

    def test_without_a_limit_nothing_changes(self):
        """The old three-argument form still builds the shapes it always did."""
        _time, command, phases = self._build()

        assert {"step", "ramp", "hold", "multisine"} <= set(phases)
        assert np.max(np.abs(np.diff(command, axis=0))) > 0.1

    def test_every_step_fits_the_budget(self):
        budget = np.array([0.02, 0.02])

        _time, command, _phases = self._build(max_step=budget)

        largest = np.max(np.abs(np.diff(command, axis=0)), axis=0)
        assert np.all(largest <= budget + 1e-12), largest

    def test_the_amplitude_is_kept_rather_than_shrunk(self):
        budget = np.array([0.02, 0.02])

        _time, command, _phases = self._build(max_step=budget)

        # Both extremes of the designed swing are still reached.
        reached = np.ptp(command, axis=0)
        np.testing.assert_allclose(reached, 2 * self.AMPLITUDE, rtol=1e-6)

    def test_bridging_costs_samples_but_keeps_the_phases(self):
        budget = np.array([0.02, 0.02])

        _t0, plain, phases0 = self._build()
        time, bridged, phases = self._build(max_step=budget)

        assert len(bridged) > len(plain)
        assert len(time) == len(bridged) == len(phases)
        assert {"step", "ramp", "hold", "multisine", "bridge"} == set(phases)

    def test_the_clock_still_matches_the_samples(self):
        budget = np.array([0.02, 0.02])

        time, command, _phases = self._build(max_step=budget)

        assert len(time) == len(command)
        np.testing.assert_allclose(np.diff(time), 1.0 / self.RATE, rtol=1e-9)

    def test_a_multisine_too_fast_to_slew_is_refused(self):
        """Extra time cannot fix it: the peak slew is frequency times amplitude,
        and stretching the track changes neither."""
        with pytest.raises(FitError, match="multisine"):
            self._build(max_step=np.array([1e-4, 1e-4]))

    def test_the_refusal_names_the_scale_that_would_fit(self):
        with pytest.raises(FitError, match="amplitude"):
            build_excitation(
                self.NEUTRAL, self.AMPLITUDE, self.RATE,
                max_step=np.array([1e-4, 1.0]),
            )

    def test_a_mismatched_budget_is_refused(self):
        with pytest.raises(FitError, match="max_step"):
            self._build(max_step=np.array([0.02]))


def test_three_repetitions_split_two_fit_one_holdout():
    assert split_repetitions(["run-1", "run-2", "run-3"]) == (
        ("run-1", "run-2"),
        ("run-3",),
    )
    with pytest.raises(FitError, match="exactly three"):
        split_repetitions(["run-1", "run-2"])


def test_second_order_fit_recovers_synthetic_stiffness_and_damping():
    rate = 500.0
    dt = 1.0 / rate
    time, command, _ = build_excitation(
        neutral=np.array([0.0]), amplitude=np.array([0.2]), rate_hz=rate
    )
    stiffness, damping = 35.0, 4.0
    measured = np.zeros_like(command)
    velocity = np.zeros(1)
    for index in range(1, len(time)):
        acceleration = stiffness * (command[index - 1] - measured[index - 1]) - damping * velocity
        velocity = velocity + acceleration * dt
        measured[index] = measured[index - 1] + velocity * dt

    estimate = fit_second_order(time, command, measured)

    assert abs(estimate.stiffness[0] - stiffness) / stiffness < 0.1
    assert abs(estimate.damping[0] - damping) / damping < 0.1


def test_holdout_gates_and_model_inadequate_status():
    assert validate_holdout(
        openarm_rmse_rad=0.02,
        tesollo_rmse_rad=0.04,
        delay_residual_sec=0.009,
        command_period_sec=0.01,
        improvement_fraction=0.31,
    ).status == "validated"

    failed = validate_holdout(
        openarm_rmse_rad=0.02,
        tesollo_rmse_rad=0.06,
        delay_residual_sec=0.009,
        command_period_sec=0.01,
        improvement_fraction=0.31,
    )
    assert failed.status == "model_inadequate"
    assert "tesollo_rmse" in failed.failures


def test_a_sweep_records_the_torque_it_published():
    """The regression's second column is the applied torque, and it has to be
    able to come from somewhere other than a multiple of the model — that is
    the whole point of driving a joint gravity does not reach."""
    sweep = GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_1", "r_aj_2"),
        poses=np.zeros((2, 2)),
        modelled_torque=np.array([[1.0, 2.0], [1.0, 2.0]]),
        scales=np.zeros((2, 2)),
        applied_torque=np.array([[0.3, -0.3], [-0.3, 0.3]]),
        errors=np.zeros((2, 2)),
    )

    assert sweep.applied_torque.tolist() == [[0.3, -0.3], [-0.3, 0.3]]


def test_a_sweep_refuses_an_applied_torque_of_the_wrong_shape():
    with pytest.raises(FitError, match="applied_torque"):
        GravitySweep(
            group="openarm_right_arm",
            joint_names=("r_aj_1", "r_aj_2"),
            poses=np.zeros((2, 2)),
            modelled_torque=np.zeros((2, 2)),
            scales=np.zeros((2, 2)),
            applied_torque=np.zeros((2, 3)),
            errors=np.zeros((2, 2)),
        )


def test_the_fit_reads_the_applied_torque_rather_than_recomputing_it():
    """A sweep whose applied torque is not a multiple of its model — which is
    what a direct-torque excitation produces — must still fit. Recomputing
    scale times model would read this sweep as having published nothing."""
    rounds = 5
    applied = np.linspace(-0.6, 0.6, rounds).reshape(rounds, 1)
    kp, gravity, offset = 20.0, 0.15, 0.001
    errors = (gravity - applied) / kp + offset

    sweep = GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_5",),
        poses=np.zeros((rounds, 1)),
        # The model says this joint carries nothing, and it is wrong.
        modelled_torque=np.zeros((rounds, 1)),
        scales=np.zeros((rounds, 1)),
        applied_torque=applied,
        errors=errors,
    )

    estimate = fit_static_gravity([sweep])

    assert estimate.stiffness[0] == pytest.approx(kp, rel=1e-6)
    # alpha multiplies a column of exact zeros here: no value of it changes
    # the prediction, so it is genuinely undefined rather than merely 0.
    assert np.isnan(estimate.torque_scale[0])


def test_the_generalised_regression_reproduces_the_measured_estimate(profile):
    from robot_control.artifacts import read_sweep

    sweeps = [read_sweep(path, profile) for path in sorted(REAL_SWEEPS.glob("pose*.json"))]

    estimate = fit_static_gravity(sweeps)

    for index, name in enumerate(estimate.joint_names):
        assert estimate.stiffness[index] == pytest.approx(
            EXPECTED_STIFFNESS[name], rel=1e-3
        ), f"{name} moved; the regression change was not an identity"


def _staircase_sweep(kp, coulomb, gravity, peak=0.6, steps=5):
    """A joint that behaves exactly like a spring with dry friction."""
    from robot_control.excitation import staircase

    values = staircase(peak, steps)
    errors, held = [], None
    for index, applied in enumerate(values):
        target = (gravity - applied) / kp
        if held is None or abs(target - held) > coulomb / kp:
            direction = 1.0 if index < steps else -1.0
            held = target + direction * coulomb / kp
        errors.append(held)
    return GravitySweep(
        group="openarm_right_arm",
        joint_names=("r_aj_5",),
        poses=np.zeros((len(values), 1)),
        modelled_torque=np.zeros((len(values), 1)),
        scales=np.zeros((len(values), 1)),
        applied_torque=np.array(values).reshape(-1, 1),
        errors=np.array(errors).reshape(-1, 1),
    )


def test_the_staircase_recovers_stiffness_friction_and_bias():
    """Rounds inside the stiction band are the measurement here, not noise.
    The gravity fit drops them; seventeen of the wrist's twenty-seven rounds
    were dropped that way, and they are exactly the ones that carry Fc."""
    estimate = fit_staircase([_staircase_sweep(kp=15.0, coulomb=0.08, gravity=0.2)])

    assert estimate.stiffness[0] == pytest.approx(15.0, rel=0.05)
    assert estimate.coulomb_nm[0] == pytest.approx(0.08, rel=0.15)
    assert estimate.bias_nm[0] == pytest.approx(0.2, rel=0.15)


def test_a_joint_whose_branches_disagree_in_slope_is_not_identified():
    """Two branches that are not parallel did not come from a spring with dry
    friction, and a number fitted to them would describe nothing."""
    sweep = _staircase_sweep(kp=15.0, coulomb=0.08, gravity=0.2)
    bent = sweep.errors.copy()
    bent[len(bent) // 2:] *= 3.0
    sweep = replace(sweep, errors=bent)

    estimate = fit_staircase([sweep])

    assert np.isnan(estimate.stiffness[0])
    assert any("r_aj_5" == name for name, _ in estimate.unidentifiable)


def test_a_branch_whose_applied_torque_never_varies_is_not_identified():
    """A branch built entirely from one applied torque carries no
    torque-deflection information at all: `np.polyfit` on a constant-x
    column is rank deficient and returns its minimum-norm solution — a
    number, not a measurement — instead of refusing. This must be caught
    before the fit runs, not left to be caught by luck downstream (the
    slopes-must-be-negative and branches-must-agree checks that follow).

    Two sweeps are needed to build it: within one sweep the peak round is
    always the last rising point and the first falling point, so a single
    sweep cannot make both branches constant. Here every sweep's peak sits
    at round 0 (applied[0] = 0.5 is already that sweep's maximum), so its
    rising branch is the single point (0.5, error) and its falling branch
    is the two points at -0.3; stacking two such sweeps for the same joint
    gives a rising branch of two rounds both at 0.5 N.m and a falling
    branch of four rounds all at -0.3 N.m — two branches, neither carrying
    any spread in torque at all.
    """
    applied = np.array([0.5, -0.3, -0.3]).reshape(-1, 1)
    kwargs = dict(
        group="openarm_right_arm",
        joint_names=("r_aj_5",),
        poses=np.zeros((3, 1)),
        modelled_torque=np.zeros((3, 1)),
        scales=np.zeros((3, 1)),
        applied_torque=applied,
    )
    sweep_a = GravitySweep(errors=np.array([0.05, 0.04, 0.03]).reshape(-1, 1), **kwargs)
    sweep_b = GravitySweep(errors=np.array([0.06, 0.02, 0.01]).reshape(-1, 1), **kwargs)

    estimate = fit_staircase([sweep_a, sweep_b])

    assert np.isnan(estimate.stiffness[0])
    assert any("r_aj_5" == name for name, _ in estimate.unidentifiable)


def test_the_staircase_reports_the_bias_from_genuinely_stuck_rounds():
    """The gravity fit's docstring warns that rounds inside the stiction
    band are dropped as noise; this fitter's whole reason to exist is that
    for a staircase they are not noise, they are the measurement. This
    fixture (kp=15, coulomb=0.08, gravity=0.2, peak=0.24, steps=7) has a
    torque step of 2*0.24/6 = 0.08 N.m, exactly the Coulomb friction, so two
    consecutive falling rounds hold the same 0.002667 rad error while the
    applied torque steps from +0.16 to +0.08 N.m — a horizontal run in the
    torque/deflection plane, not a point on either branch's line.

    A straight OLS fit through a partly-horizontal branch is measurably
    biased, and the point of this test is to measure it rather than assume
    it away. Run directly (not tuned to pass): stiffness comes out 16.154
    against a true 15 (+7.7%), coulomb 0.0759 against 0.08 (-5.1%), bias
    0.2256 against 0.2 (+12.8%). The tolerances below sit just above those
    measured numbers, so this pins the bias that exists.

    Only the falling branch can carry stuck rounds here: on a monotonic
    ramp the joint is already at the friction boundary from the previous
    round, so every round's torque change moves it again (the algebra is in
    fit_staircase's own rising-branch reasoning) — freezing needs a
    direction reversal to open the band back up, and a single staircase
    cycle has exactly one reversal, at the start of the falling branch.
    """
    sweep = _staircase_sweep(kp=15.0, coulomb=0.08, gravity=0.2, peak=0.24, steps=7)

    estimate = fit_staircase([sweep])

    assert not np.isnan(estimate.stiffness[0]), "expected a biased fit, not a refusal"
    assert estimate.stiffness[0] == pytest.approx(15.0, rel=0.08)
    assert estimate.coulomb_nm[0] == pytest.approx(0.08, rel=0.06)
    assert estimate.bias_nm[0] == pytest.approx(0.2, rel=0.13)
    # The clean fixture (no in-band rounds) fits to float precision; a
    # genuinely stuck branch does not, and residual_rmse says so.
    assert estimate.residual_rmse[0] > 5e-4
