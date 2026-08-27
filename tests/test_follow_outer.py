from __future__ import annotations

import numpy as np
import pytest

from robot_control.follow_outer import (
    OUTER_TARGET_SIGN_EPSILON_RAD,
    bound_outer_candidate,
)
from robot_control.safety import CommandGate


def _raw(command, target, measured, *, kp=2.0, dt=0.01):
    return command + kp * (target - measured) * dt


def test_positive_direction_crossing_clamps_at_ik_target():
    result = bound_outer_candidate(
        np.array([0.9]), np.array([1.0]), np.array([1.1])
    )

    np.testing.assert_array_equal(result.candidate, [1.0])
    np.testing.assert_array_equal(result.crossing_mask, [True])
    np.testing.assert_array_equal(result.clamp_mask, [True])


def test_negative_direction_crossing_clamps_at_ik_target():
    result = bound_outer_candidate(
        np.array([1.1]), np.array([1.0]), np.array([0.9])
    )

    np.testing.assert_array_equal(result.candidate, [1.0])
    np.testing.assert_array_equal(result.crossing_mask, [True])


def test_non_crossing_update_is_bit_for_bit_unchanged():
    raw = np.array([0.93, -0.41, 0.2])
    result = bound_outer_candidate(
        np.array([0.9, -0.5, 0.2]),
        np.array([1.0, -0.2, 0.2]),
        raw,
    )

    np.testing.assert_array_equal(result.candidate, raw)
    assert not np.any(result.clamp_mask)


def test_target_hold_blocks_repeated_measured_lag_integration():
    target = np.array([1.0])
    command = target.copy()
    measured = np.array([0.8])

    for _ in range(1000):
        raw = _raw(command, target, measured)
        result = bound_outer_candidate(command, target, raw)
        command = result.candidate
        np.testing.assert_array_equal(command, target)
        np.testing.assert_array_equal(result.target_hold_mask, [True])


def test_only_crossing_joint_is_clamped():
    command = np.array([0.9, 0.0, -0.5, 0.2, 0.3, -0.4, 0.0])
    target = np.array([1.0, 1.0, -0.2, 0.2, 0.0, -0.8, 0.5])
    raw = np.array([1.1, 0.1, -0.4, 0.2, 0.2, -0.5, 0.01])

    result = bound_outer_candidate(command, target, raw)

    np.testing.assert_array_equal(
        result.clamp_mask, [True, False, False, False, False, False, False]
    )
    np.testing.assert_array_equal(result.candidate[1:], raw[1:])


def test_crossing_boundary_and_exact_target_are_stable():
    epsilon = OUTER_TARGET_SIGN_EPSILON_RAD
    command = np.array([0.5])
    target = np.array([1.0])

    before = bound_outer_candidate(command, target, target - 10.0 * epsilon)
    exact = bound_outer_candidate(command, target, target.copy())
    after = bound_outer_candidate(command, target, target + 10.0 * epsilon)

    assert not before.clamp_mask[0]
    assert not exact.clamp_mask[0]
    assert after.crossing_mask[0]
    np.testing.assert_array_equal(exact.candidate, target)
    np.testing.assert_array_equal(after.candidate, target)


def test_machine_precision_noise_does_not_toggle_clamp_direction():
    epsilon = OUTER_TARGET_SIGN_EPSILON_RAD
    target = np.array([1.0])
    result = bound_outer_candidate(
        target + 0.25 * epsilon,
        target,
        target - 0.25 * epsilon,
    )

    assert not np.any(result.clamp_mask)
    assert not np.any(result.crossing_mask)
    assert not np.any(result.outward_mask)


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_positive_and_negative_joint_angles_are_symmetric(sign):
    command = sign * np.array([0.9, 1.1, 0.4])
    target = sign * np.array([1.0, 1.0, 0.8])
    raw = sign * np.array([1.1, 0.9, 0.5])

    result = bound_outer_candidate(command, target, raw)
    mirrored = bound_outer_candidate(-command, -target, -raw)

    np.testing.assert_array_equal(result.clamp_mask, mirrored.clamp_mask)
    np.testing.assert_allclose(result.candidate, -mirrored.candidate)


def test_moving_ik_target_preserves_normal_measured_error_updates():
    command = np.array([0.0])
    measured = np.array([0.0])

    for target_value in np.linspace(0.01, 0.2, 20):
        target = np.array([target_value])
        raw = _raw(command, target, measured)
        result = bound_outer_candidate(command, target, raw)
        assert not result.clamp_mask[0]
        np.testing.assert_array_equal(result.candidate, raw)
        command = result.candidate
        measured = command.copy()


def test_target_reversal_blocks_outward_step_and_requests_bounded_recovery():
    command = np.array([1.2])
    new_target = np.array([1.0])
    lagging_measured = np.array([0.8])
    raw = _raw(command, new_target, lagging_measured)

    result = bound_outer_candidate(command, new_target, raw)

    assert raw[0] > command[0]
    np.testing.assert_array_equal(result.outward_mask, [True])
    np.testing.assert_array_equal(result.candidate, new_target)


def test_target_reversal_stalled_step_requests_limiter_bounded_recovery():
    command = np.array([1.2])
    new_target = np.array([1.0])

    result = bound_outer_candidate(command, new_target, command.copy())

    np.testing.assert_array_equal(result.outward_mask, [False])
    np.testing.assert_array_equal(result.stalled_recovery_mask, [True])
    np.testing.assert_array_equal(result.candidate, new_target)


@pytest.mark.parametrize("dt", [1e-9, 0.01, 0.006, 0.017])
def test_policy_is_stable_for_small_normal_and_jittered_dt(dt):
    command = np.array([1.0])
    target = np.array([1.0])
    measured = np.array([0.8])
    raw = _raw(command, target, measured, dt=dt)

    result = bound_outer_candidate(command, target, raw)

    np.testing.assert_array_equal(result.candidate, target)
    assert result.target_hold_mask[0]


def _gate(*, velocity=100.0, max_lead=10.0, lower=-10.0, upper=10.0):
    return CommandGate(
        execute=True,
        lower=np.array([lower]),
        upper=np.array([upper]),
        velocity=np.array([velocity]),
        command_period_sec=0.01,
        max_lead=np.array([max_lead]),
        names=("joint",),
    )


def test_crossing_clamp_does_not_bypass_joint_velocity_limiter():
    gate = _gate(velocity=0.2)
    command, _ = gate.follow(np.array([0.0]), np.array([0.0]), 0.01)
    outer = bound_outer_candidate(command, np.array([1.0]), np.array([2.0]))

    final, _ = gate.follow(outer.candidate, np.array([0.0]), 0.01)

    np.testing.assert_allclose(final, [0.002])
    assert gate.last_follow_limits["velocity"] == ("joint",)


def test_crossing_clamp_does_not_bypass_measured_lead_limiter():
    gate = _gate(max_lead=0.1)
    command, _ = gate.follow(np.array([0.0]), np.array([0.0]), 0.01)
    outer = bound_outer_candidate(command, np.array([1.0]), np.array([2.0]))

    final, _ = gate.follow(outer.candidate, np.array([0.0]), 0.01)

    np.testing.assert_allclose(final, [0.1])
    assert gate.last_follow_limits["lead"] == ("joint",)


def test_crossing_clamp_does_not_bypass_position_limiter():
    gate = _gate(upper=1.0)
    command, _ = gate.follow(np.array([0.9]), np.array([0.9]), 0.01)
    outer = bound_outer_candidate(command, np.array([1.5]), np.array([2.0]))

    final, _ = gate.follow(outer.candidate, np.array([0.9]), 0.01)

    np.testing.assert_allclose(final, [1.0])
    assert gate.last_follow_limits["position"] == ("joint",)


def test_seeded_property_invariants_and_sign_symmetry():
    rng = np.random.default_rng(20260826)
    for _ in range(2000):
        command = rng.uniform(-2.0, 2.0, 7)
        target = rng.uniform(-2.0, 2.0, 7)
        measured = rng.uniform(-2.0, 2.0, 7)
        dt = rng.uniform(1e-8, 0.03)
        raw = _raw(command, target, measured, dt=dt)

        result = bound_outer_candidate(command, target, raw)
        mirrored = bound_outer_candidate(-command, -target, -raw)

        assert np.isfinite(result.candidate).all()
        np.testing.assert_array_equal(
            result.candidate[~result.clamp_mask], raw[~result.clamp_mask]
        )
        np.testing.assert_array_equal(
            result.candidate[result.clamp_mask], target[result.clamp_mask]
        )
        command_error = target - command
        bounded_error = target - result.candidate
        assert not np.any(
            (command_error > OUTER_TARGET_SIGN_EPSILON_RAD)
            & (bounded_error < -OUTER_TARGET_SIGN_EPSILON_RAD)
        )
        assert not np.any(
            (command_error < -OUTER_TARGET_SIGN_EPSILON_RAD)
            & (bounded_error > OUTER_TARGET_SIGN_EPSILON_RAD)
        )
        np.testing.assert_array_equal(result.clamp_mask, mirrored.clamp_mask)
        np.testing.assert_allclose(result.candidate, -mirrored.candidate)
