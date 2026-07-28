"""The dynamic fit with its gravity term, and the parameters the two fits share.

    J qdd = kp (q_cmd - q) - b qd - tau_f sign(qd) - tau_g(q)

`fit_second_order` divides through by J, so on its own it returns every
parameter scaled by an inertia it cannot separate. The static fit returns kp with
no inertia in it. Together the inertia falls out.
"""

import numpy as np
import pytest

from robot_control.identification import (
    FRICTION_SHARPNESS,
    FitError,
    SecondOrderEstimate,
    StaticEstimate,
    combine,
    fit_second_order,
)


KP = np.array([20.0, 8.0])
DAMPING = np.array([1.5, 0.6])
FRICTION = np.array([0.4, 0.15])
#: Fo. Small next to FRICTION on purpose: at the wrist this is what makes it
#: visible at all — a bias comparable to or larger than the Coulomb term.
BIAS = np.array([0.05, 0.02])
INERTIA = np.array([0.35, 0.08])
LOAD = np.array([2.5, 0.9])
STEP = 1e-3
SAMPLES = 4000


def _track():
    """Simulate the physical equation, sampled the way the fitter differentiates.

    Semi-implicit Euler on purpose: it makes the finite differences the fitter
    takes exactly the accelerations the simulation used, so a correct fit is
    exact and any bias in it shows up as a bias rather than as discretisation.
    """
    width = len(KP)
    command = np.zeros((SAMPLES, width))
    for joint in range(width):
        phase = np.arange(SAMPLES) * STEP
        command[:, joint] = 0.4 * np.sin(2 * np.pi * (0.7 + joint) * phase)

    measured = np.zeros((SAMPLES, width))
    torque = np.zeros((SAMPLES, width))
    position = np.zeros(width)
    velocity = np.zeros(width)
    for index in range(SAMPLES):
        measured[index] = position
        torque[index] = LOAD * np.cos(position)
        acceleration = (
            KP * (command[index] - position)
            - DAMPING * velocity
            - FRICTION * np.tanh(FRICTION_SHARPNESS * velocity)
            - BIAS
            - torque[index]
        ) / INERTIA
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    time = np.arange(SAMPLES) * STEP
    return time, command, measured, torque


def test_the_gravity_term_recovers_the_inverse_inertia():
    time, command, measured, torque = _track()

    estimate = fit_second_order(time, command, measured, gravity_torque=torque)

    np.testing.assert_allclose(estimate.stiffness, KP / INERTIA, rtol=1e-3)
    np.testing.assert_allclose(estimate.damping, DAMPING / INERTIA, rtol=1e-2)
    np.testing.assert_allclose(estimate.friction, FRICTION / INERTIA, rtol=1e-2)
    np.testing.assert_allclose(estimate.bias, BIAS / INERTIA, rtol=1e-2)
    np.testing.assert_allclose(estimate.inverse_inertia, 1.0 / INERTIA, rtol=1e-2)


def test_without_the_gravity_term_the_standing_load_lands_in_the_bias():
    """The bug this term fixes: the regression used to have nowhere but the
    stiffness column to put a standing load. Now it has a better sink — a
    constant column — so a near-constant load (this one varies only mildly,
    by cos(position) around a small angle) lands there instead, leaving
    stiffness only mildly distorted. The bias this produces is not the true
    Fo, though: it is Fo plus the load's mean, and the two are not separable
    without the gravity column that tells them apart.
    """
    time, command, measured, _torque = _track()

    blind = fit_second_order(time, command, measured)

    assert blind.inverse_inertia is None
    truth = BIAS / INERTIA
    assert np.all(np.abs(blind.bias - truth) / truth > 5.0), (
        "the load has to distort something, and the bias column is where it "
        "goes now; if this passes, the track no longer carries a standing load"
    )
    assert np.all(blind.residual_rmse > 0)


def test_the_gravity_term_reduces_the_residual():
    time, command, measured, torque = _track()

    blind = fit_second_order(time, command, measured)
    seeing = fit_second_order(time, command, measured, gravity_torque=torque)

    assert np.all(seeing.residual_rmse < blind.residual_rmse)


def test_a_gravity_column_of_the_wrong_shape_is_refused():
    time, command, measured, torque = _track()

    with pytest.raises(FitError, match="gravity"):
        fit_second_order(time, command, measured, gravity_torque=torque[:, :1])


def _static(stiffness=KP):
    width = len(stiffness)
    return StaticEstimate(
        joint_names=("a", "b"),
        stiffness=np.asarray(stiffness, dtype=float),
        torque_scale=np.ones(width),
        offset=np.zeros(width),
        residual_rmse=np.full(width, 1e-4),
        condition=np.full(width, 5.0),
        used=np.full(width, 12, dtype=int),
        excluded=np.zeros(width, dtype=int),
        unidentifiable=(),
    )


def test_combining_the_two_fits_recovers_the_physical_parameters():
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)

    combined = combine(_static(), dynamic, ("a", "b"))

    np.testing.assert_allclose(combined.inertia, INERTIA, rtol=1e-2)
    np.testing.assert_allclose(combined.damping, DAMPING, rtol=2e-2)
    np.testing.assert_allclose(combined.friction, FRICTION, rtol=2e-2)
    np.testing.assert_allclose(combined.bias, BIAS, rtol=2e-2)
    np.testing.assert_allclose(combined.stiffness, KP, rtol=1e-9)


def test_the_two_paths_to_the_inertia_agree():
    """kp/k from the static fit, and 1/J straight out of the gravity column.

    They come from different columns of different experiments, so agreeing is
    evidence rather than arithmetic.
    """
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)

    combined = combine(_static(), dynamic, ("a", "b"))

    np.testing.assert_allclose(combined.inertia_from_gravity, INERTIA, rtol=1e-2)
    assert np.all(combined.disagreement < 0.02)


def test_a_stiffness_from_the_wrong_robot_shows_up_as_disagreement():
    """The cross-check earns its keep here: nothing else would catch this."""
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)

    combined = combine(_static(KP * 2.0), dynamic, ("a", "b"))

    assert np.all(combined.disagreement > 0.4)


def test_without_a_gravity_column_there_is_nothing_to_cross_check():
    time, command, measured, _torque = _track()
    dynamic = fit_second_order(time, command, measured)

    combined = combine(_static(), dynamic, ("a", "b"))

    assert np.all(np.isnan(combined.inertia_from_gravity))
    assert np.all(np.isnan(combined.disagreement))
    # The primary path still works: it only ever needed kp and k.
    assert np.all(combined.inertia > 0)


def test_combine_carries_the_static_fits_own_coulomb_and_bias_through():
    """kp/k gives the physical Fc and Fo from the dynamic side; these two are
    the static fit's own numbers, passed through unscaled rather than derived
    from anything `combine` itself computes.
    """
    import dataclasses

    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)
    static = dataclasses.replace(
        _static(), coulomb_nm=np.array([0.08, 0.05]), bias_nm=np.array([0.3, -0.1])
    )

    combined = combine(static, dynamic, ("a", "b"))

    np.testing.assert_allclose(combined.coulomb_nm, [0.08, 0.05])
    np.testing.assert_allclose(combined.static_bias_nm, [0.3, -0.1])


def test_combine_carries_nothing_when_the_static_fit_never_measured_them():
    """A gravity-sweep static estimate never touches a staircase at all."""
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)

    combined = combine(_static(), dynamic, ("a", "b"))

    assert combined.coulomb_nm is None
    assert combined.static_bias_nm is None


def test_combining_fits_of_different_widths_is_refused():
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)
    narrow = SecondOrderEstimate(
        dynamic.stiffness[:1],
        dynamic.damping[:1],
        dynamic.friction[:1],
        dynamic.bias[:1],
        dynamic.residual_rmse[:1],
        dynamic.inverse_inertia[:1],
    )

    with pytest.raises(FitError, match="joint"):
        combine(_static(), narrow, ("a", "b"))


def test_combining_against_an_incomplete_static_estimate_is_refused():
    time, command, measured, torque = _track()
    dynamic = fit_second_order(time, command, measured, gravity_torque=torque)
    import dataclasses

    partial = dataclasses.replace(_static(), unidentifiable=(("b", "frozen"),))

    with pytest.raises(FitError, match="b"):
        combine(partial, dynamic, ("a", "b"))
