"""Scoring a fitted model against a run it never saw.

`validate_holdout` has always taken three numbers. Nothing computed them, so
`r2s validate --metrics` read a file somebody wrote by hand, and the holdout half
of validation had never run against anything measured.
"""

import numpy as np
import pytest

from robot_control.identification import (
    FRICTION_SHARPNESS,
    FitError,
    SecondOrderEstimate,
    fit_second_order,
    predict_second_order,
    score_holdout,
)


KP = np.array([20.0, 8.0])
DAMPING = np.array([1.5, 0.6])
FRICTION = np.array([0.4, 0.15])
#: Fo, small next to FRICTION as it is at the wrist this stands in for.
BIAS = np.array([0.05, 0.02])
INERTIA = np.array([0.35, 0.08])
LOAD = np.array([2.5, 0.9])
STEP = 1e-3
SAMPLES = 3000


def _run(seed=0):
    """The same simulation the fit's integrator assumes, so an exact model
    reproduces it exactly and any error is the model's rather than the maths'."""
    rng = np.random.default_rng(seed)
    clock = np.arange(SAMPLES) * STEP
    command = np.column_stack(
        [
            0.4 * np.sin(2 * np.pi * (0.7 + joint) * clock + rng.uniform(0, 1))
            for joint in range(2)
        ]
    )
    measured = np.zeros((SAMPLES, 2))
    torque = np.zeros((SAMPLES, 2))
    position = np.zeros(2)
    velocity = np.zeros(2)
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
    return clock, command, measured, torque


@pytest.fixture(scope="module")
def fitted():
    clock, command, measured, torque = _run(seed=0)
    return fit_second_order(clock, command, measured, gravity_torque=torque)


def test_an_exact_model_reproduces_the_run_it_was_fitted_on(fitted):
    clock, command, measured, torque = _run(seed=0)

    predicted = predict_second_order(
        fitted, clock, command, measured, gravity_torque=torque
    )

    np.testing.assert_allclose(predicted, measured, atol=1e-6)


def test_an_exact_model_predicts_a_run_it_never_saw(fitted):
    clock, command, measured, torque = _run(seed=3)

    metrics = score_holdout(fitted, clock, command, measured, gravity_torque=torque)

    assert metrics.rmse_rad < 1e-4
    assert metrics.delay_residual_sec == 0.0
    assert metrics.improvement_fraction > 0.99


def test_the_baseline_is_assuming_the_arm_reached_its_command(fitted):
    """What you would believe with no model at all, which is what a model has to
    beat to be worth carrying."""
    clock, command, measured, torque = _run(seed=3)

    metrics = score_holdout(fitted, clock, command, measured, gravity_torque=torque)

    expected = float(np.sqrt(np.mean((command - measured) ** 2)))
    assert metrics.baseline_rmse_rad == pytest.approx(expected)
    assert metrics.baseline_rmse_rad > metrics.rmse_rad


def test_a_wrong_model_scores_badly_rather_than_raising(fitted):
    clock, command, measured, torque = _run(seed=3)
    wrong = SecondOrderEstimate(
        fitted.stiffness * 0.3,
        fitted.damping * 3.0,
        fitted.friction,
        fitted.bias,
        fitted.residual_rmse,
        fitted.inverse_inertia,
    )

    metrics = score_holdout(wrong, clock, command, measured, gravity_torque=torque)

    assert metrics.rmse_rad > 1e-3
    assert metrics.improvement_fraction < 0.99


def test_unmodelled_delay_shows_up_as_a_residual(fitted):
    """Shift the measurement and the best alignment is no longer zero."""
    clock, command, measured, torque = _run(seed=3)
    lag = 12
    delayed = np.vstack([np.tile(measured[0], (lag, 1)), measured[:-lag]])

    metrics = score_holdout(fitted, clock, command, delayed, gravity_torque=torque)

    assert metrics.delay_residual_sec == pytest.approx(lag * STEP, abs=STEP)


def test_a_model_of_the_wrong_width_is_refused(fitted):
    clock, command, measured, torque = _run(seed=3)

    with pytest.raises(FitError, match="joint"):
        score_holdout(
            fitted, clock, command[:, :1], measured[:, :1], gravity_torque=torque[:, :1]
        )


def test_a_gravity_fitted_model_needs_its_gravity_column(fitted):
    """Scoring it without one would silently score a different model."""
    clock, command, measured, _torque = _run(seed=3)

    with pytest.raises(FitError, match="gravity"):
        score_holdout(fitted, clock, command, measured)


def test_a_model_fitted_without_gravity_scores_without_it():
    clock, command, measured, _torque = _run(seed=0)
    level = np.zeros_like(measured)
    position, velocity = np.zeros(2), np.zeros(2)
    for index in range(SAMPLES):
        level[index] = position
        acceleration = (
            KP * (command[index] - position) - DAMPING * velocity
        ) / INERTIA
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    estimate = fit_second_order(clock, command, level)

    metrics = score_holdout(estimate, clock, command, level)

    assert metrics.rmse_rad < 1e-4
