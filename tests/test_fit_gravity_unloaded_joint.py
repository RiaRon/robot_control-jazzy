"""A joint the gravity column says nothing about must not sink the whole fit.

A forearm-roll axis spends the collect nearly parallel to gravity, so its
modelled torque is ~zero along the entire track. The column then identifies
nothing for that joint — its coefficient is noise-signed — which is not the
same thing as the column having the wrong sign on a joint it does load.
"""

import numpy as np
import pytest

from robot_control.identification import FitError, fit_second_order_runs


KP = np.array([20.0, 12.0])
DAMPING = np.array([1.5, 0.8])
FRICTION = np.array([0.4, 0.2])
INERTIA = np.array([0.35, 0.10])
STEP = 1e-3
SAMPLES = 3000


def _run(load_scale=(1.0, 0.0)):
    """Two spring joints; the second carries no standing load at all."""
    clock = np.arange(SAMPLES) * STEP
    command = np.column_stack(
        [0.3 * np.sin(2 * np.pi * (0.6 + 0.2 * joint) * clock) for joint in range(2)]
    )
    gravity = np.zeros((SAMPLES, 2))
    measured = np.zeros((SAMPLES, 2))
    position = np.zeros(2)
    velocity = np.zeros(2)
    scale = np.asarray(load_scale, dtype=float)
    for index in range(SAMPLES):
        measured[index] = position
        load = scale * 2.0 * np.cos(position)
        gravity[index] = load
        acceleration = (
            KP * (command[index] - position)
            - DAMPING * velocity
            - FRICTION * np.sign(velocity)
            - load
        ) / INERTIA
        velocity = velocity + STEP * acceleration
        position = position + STEP * velocity
    return clock, command, measured, gravity


def test_an_unloaded_joint_fits_without_an_inertia_cross_check():
    estimate = fit_second_order_runs([_run()])

    assert estimate.inverse_inertia is not None
    np.testing.assert_allclose(1.0 / estimate.inverse_inertia[0], INERTIA[0], rtol=0.05)
    assert np.isnan(estimate.inverse_inertia[1])
    # The rest of the model is still identified for both joints.
    np.testing.assert_allclose(estimate.stiffness, KP / INERTIA, rtol=0.05)


def test_a_wholly_sign_flipped_gravity_column_is_still_refused():
    """Every loaded joint wrong means the chain is wrong — refuse the fit."""
    clock, command, measured, gravity = _run(load_scale=(1.0, 1.0))

    with pytest.raises(FitError, match="accelerate towards their load"):
        fit_second_order_runs([(clock, command, measured, -gravity)])


def test_one_wrong_joint_drops_its_column_instead_of_killing_the_fit():
    """A minority of joints wrong is a local model error, not a wrong chain.

    A hand's centre of mass can be off in a way that flips one wrist axis
    while the shoulder and elbow stay right. Refusing the whole fit would
    throw away six good joints to punish one.
    """
    clock, command, measured, gravity = _run(load_scale=(1.0, 1.0))
    flipped = gravity.copy()
    flipped[:, 1] *= -1.0

    estimate = fit_second_order_runs([(clock, command, measured, flipped)])

    np.testing.assert_allclose(1.0 / estimate.inverse_inertia[0], INERTIA[0], rtol=0.05)
    assert np.isnan(estimate.inverse_inertia[1])
    assert estimate.gravity_disagreed == (1,)
    # The joint keeps the rest of its model, at the price of dropping the
    # column: its standing load is position-dependent, so what the column no
    # longer explains is absorbed into stiffness. Loose here for that reason.
    np.testing.assert_allclose(estimate.stiffness[1], KP[1] / INERTIA[1], rtol=0.1)


def test_a_column_buried_under_friction_is_dropped_not_convicted():
    """A wiggle track around one pose varies gravity less than dry friction.

    Motion then cannot carry the column's information — friction eats it — so
    even a locally sign-flipped model must not be convicted on it. The joint
    keeps its dynamic model and offers no gravity cross-check.
    """
    clock, command, measured, gravity = _run(load_scale=(1.0, 0.0))
    # Joint B: a constant load plus a wiggle far smaller than its friction,
    # sign-flipped against how the (zero) real load actually moved it.
    buried = gravity.copy()
    buried[:, 1] = 0.5 + 0.01 * np.sin(np.arange(SAMPLES) * STEP * 2 * np.pi)

    estimate = fit_second_order_runs(
        [(clock, command, measured, buried)],
        coulomb_nm=np.array([FRICTION[0] * INERTIA[0] * 100, 0.2]),
    )

    # Joint A's column span is under its (huge) floor too — dropped, not used.
    assert np.isnan(estimate.inverse_inertia[0])
    assert np.isnan(estimate.inverse_inertia[1])


def test_the_friction_floor_spares_an_informative_column():
    clock, command, measured, gravity = _run()

    estimate = fit_second_order_runs(
        [(clock, command, measured, gravity)],
        coulomb_nm=np.array([1e-3, 1e-3]),
    )

    np.testing.assert_allclose(1.0 / estimate.inverse_inertia[0], INERTIA[0], rtol=0.05)
