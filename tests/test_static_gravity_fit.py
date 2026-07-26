"""Fitting stiffness, stiction and a torque correction from gravity sweeps.

The model, per joint, over rounds indexed by pose and scale:

    error = (alpha - s) * tau_model / kp + c

`alpha` corrects the modelled torque for masses the URDF does not know about,
`c` absorbs the scale-independent offset where stiction lives, and `kp` is the
stiffness the hardware applies — which is the parameter the dynamic fit cannot
separate from inertia on its own.
"""

import numpy as np
import pytest

from robot_control.identification import (
    FitError,
    GravitySweep,
    fit_static_gravity,
)


NAMES = ("a", "b")
SCALES = (0.0, 0.4, 0.8, 1.2)


def _sweep(kp, alpha, offset, torques, scales=SCALES, names=NAMES, frozen=None):
    """One pose held at several scales, with the errors the model predicts.

    *torques* is the modelled gravity torque at this pose, one per joint.
    *frozen* names joints whose error is pinned, as a joint inside its stiction
    band is: the torque changes and the joint does not move.
    """
    kp = np.asarray(kp, dtype=float)
    alpha = np.asarray(alpha, dtype=float)
    offset = np.asarray(offset, dtype=float)
    torques = np.asarray(torques, dtype=float)
    grid = np.array([[scale] * len(names) for scale in scales])
    modelled = np.tile(torques, (len(scales), 1))
    errors = (alpha - grid) * modelled / kp + offset
    for name in frozen or ():
        errors[:, names.index(name)] = errors[0, names.index(name)]
    return GravitySweep(
        group="arm",
        joint_names=names,
        poses=np.zeros_like(modelled),
        modelled_torque=modelled,
        scales=grid,
        errors=errors,
    )


KP = np.array([12.0, 25.0])
ALPHA = np.array([1.15, 0.92])
OFFSET = np.array([0.002, -0.001])


def _three_poses(**kwargs):
    return [
        _sweep(KP, ALPHA, OFFSET, torques, **kwargs)
        for torques in ([1.0, 4.0], [3.0, 1.5], [5.5, 6.0])
    ]


def test_the_fit_recovers_the_parameters_it_was_generated_from():
    estimate = fit_static_gravity(_three_poses())

    np.testing.assert_allclose(estimate.stiffness, KP, rtol=1e-6)
    np.testing.assert_allclose(estimate.torque_scale, ALPHA, rtol=1e-6)
    np.testing.assert_allclose(estimate.offset, OFFSET, atol=1e-9)
    assert estimate.unidentifiable == ()
    np.testing.assert_allclose(estimate.residual_rmse, 0.0, atol=1e-12)


def test_the_fit_survives_measurement_noise():
    sweeps = _three_poses()
    rng = np.random.default_rng(0)
    noisy = [
        GravitySweep(
            group=sweep.group,
            joint_names=sweep.joint_names,
            poses=sweep.poses,
            modelled_torque=sweep.modelled_torque,
            scales=sweep.scales,
            errors=sweep.errors + rng.normal(0.0, 2e-4, sweep.errors.shape),
        )
        for sweep in sweeps
    ]

    estimate = fit_static_gravity(noisy)

    np.testing.assert_allclose(estimate.stiffness, KP, rtol=0.05)
    np.testing.assert_allclose(estimate.torque_scale, ALPHA, rtol=0.05)
    assert np.all(estimate.residual_rmse < 1e-3)


def test_a_single_pose_cannot_separate_stiffness_from_the_torque_model():
    """One pose is one equation in three unknowns, however many scales it holds."""
    estimate = fit_static_gravity([_sweep(KP, ALPHA, OFFSET, [1.0, 4.0])])

    assert set(name for name, _reason in estimate.unidentifiable) == set(NAMES)
    for _name, reason in estimate.unidentifiable:
        assert "condition" in reason
    assert np.all(np.isnan(estimate.stiffness))


def test_a_joint_frozen_in_its_stiction_band_is_excluded_and_named():
    estimate = fit_static_gravity(_three_poses(frozen=("b",)))

    # 'a' moved at every pose and is identified from the same rounds as before.
    assert estimate.stiffness[0] == pytest.approx(KP[0], rel=1e-6)
    # 'b' never moved, so every one of its samples carries no gradient.
    assert estimate.excluded[1] == 3 * len(SCALES)
    assert estimate.excluded[0] == 0
    assert [name for name, _ in estimate.unidentifiable] == ["b"]
    assert np.isnan(estimate.stiffness[1])


def test_a_joint_frozen_at_only_one_pose_is_still_identified():
    """Excluding the frozen pose is what makes the remaining two usable."""
    sweeps = _three_poses()
    sweeps[1] = _sweep(KP, ALPHA, OFFSET, [3.0, 1.5], frozen=("b",))

    estimate = fit_static_gravity(sweeps)

    assert estimate.excluded[1] == len(SCALES)
    assert estimate.unidentifiable == ()
    assert estimate.stiffness[1] == pytest.approx(KP[1], rel=1e-6)


def test_sweeps_of_different_joints_cannot_be_fitted_together():
    other = _sweep(KP, ALPHA, OFFSET, [1.0, 4.0], names=("a", "c"))

    with pytest.raises(FitError, match="joint"):
        fit_static_gravity([*_three_poses(), other])


def test_sweeps_of_different_groups_cannot_be_fitted_together():
    sweeps = _three_poses()
    mixed = GravitySweep(
        group="other_arm",
        joint_names=sweeps[0].joint_names,
        poses=sweeps[0].poses,
        modelled_torque=sweeps[0].modelled_torque,
        scales=sweeps[0].scales,
        errors=sweeps[0].errors,
    )

    with pytest.raises(FitError, match="group"):
        fit_static_gravity([*sweeps, mixed])


def test_no_sweeps_is_refused():
    with pytest.raises(FitError, match="sweep"):
        fit_static_gravity([])


def test_a_negative_stiffness_is_reported_rather_than_returned():
    """Nothing in least squares stops it; a spring that pushes the wrong way is
    a sign the sweep measured something other than what the model describes."""
    sweeps = [
        _sweep(np.array([-12.0, 25.0]), ALPHA, OFFSET, torques)
        for torques in ([1.0, 4.0], [3.0, 1.5], [5.5, 6.0])
    ]

    estimate = fit_static_gravity(sweeps)

    assert [name for name, _ in estimate.unidentifiable] == ["a"]
    assert np.isnan(estimate.stiffness[0])
