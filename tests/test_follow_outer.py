from __future__ import annotations

import numpy as np
import pytest

from robot_control.follow_outer import (
    IK_TARGET_CHANGE_ATOL_RAD,
    PostCrossingTargetHold,
    crossing_mask,
)
from robot_control.safety import CommandGate


@pytest.mark.parametrize(
    ("target", "published"),
    [
        pytest.param(0.5, 0.7, id="positive-direction"),
        pytest.param(-0.5, -0.7, id="negative-direction"),
    ],
)
def test_post_limiter_crossing_is_preserved_then_uses_target_next_cycle(
    target, published
):
    control = PostCrossingTargetHold(1)
    command = np.array([0.0])
    measured = np.array([0.0])
    ik_target = np.array([target])

    first = control.request(command, measured, ik_target, 2.0, 1.0)
    post_limiter = np.array([published])
    crossed = control.observe_published_command(
        command, post_limiter, ik_target
    )

    assert crossed.tolist() == [True]
    # Detection does not alter the post-limiter command sent by the caller.
    np.testing.assert_array_equal(post_limiter, [published])

    following = control.request(
        post_limiter, measured, ik_target, 2.0, 1.0
    )
    assert following.used_ik_target_mask.tolist() == [True]
    np.testing.assert_array_equal(following.pre_limiter_target, ik_target)
    assert following.raw_candidate[0] != pytest.approx(target)


def test_command_crossing_does_not_require_measured_crossing():
    target = np.array([0.5])
    command_crossed = crossing_mask([0.4], [0.6], target)
    measured_crossed = crossing_mask([0.2], [0.3], target)

    assert command_crossed.tolist() == [True]
    assert measured_crossed.tolist() == [False]


def test_raw_crossing_without_post_limiter_crossing_does_not_hold():
    control = PostCrossingTargetHold(1)
    target = np.array([0.5])
    request = control.request([0.4], [0.0], target, 2.0, 1.0)
    assert request.raw_candidate[0] > target[0]

    crossed = control.observe_published_command([0.4], [0.49], target)
    following = control.request([0.49], [0.0], target, 2.0, 1.0)

    assert crossed.tolist() == [False]
    assert following.used_ik_target_mask.tolist() == [False]
    assert following.pre_limiter_target[0] > target[0]


def test_same_target_with_persistent_lag_does_not_restart_accumulation():
    control = PostCrossingTargetHold(1)
    target = np.array([0.5])
    control.observe_published_command([0.4], [0.6], target)

    for command in (np.array([0.6]), np.array([0.55]), np.array([0.5])):
        request = control.request(command, [0.1], target, 2.0, 0.1)
        assert request.raw_candidate[0] > command[0]
        np.testing.assert_array_equal(request.pre_limiter_target, target)
        assert request.used_ik_target_mask.tolist() == [True]
        control.observe_published_command(command, target, target)


def test_crossed_joint_does_not_block_other_joint_outer_update():
    control = PostCrossingTargetHold(2)
    target = np.array([0.5, -0.5])
    control.observe_published_command([0.4, -0.4], [0.6, -0.45], target)

    request = control.request(
        [0.6, -0.45], [0.1, -0.1], target, 2.0, 0.1
    )

    assert request.used_ik_target_mask.tolist() == [True, False]
    assert request.pre_limiter_target[0] == pytest.approx(target[0])
    assert request.pre_limiter_target[1] == pytest.approx(
        -0.45 + 2.0 * (-0.5 - -0.1) * 0.1
    )


def test_float_noise_keeps_hold_but_meaningful_new_target_releases_it():
    control = PostCrossingTargetHold(1)
    target = np.array([0.5])
    control.observe_published_command([0.4], [0.6], target)

    noise_target = target + 0.5 * IK_TARGET_CHANGE_ATOL_RAD
    noise = control.request([0.6], [0.1], noise_target, 2.0, 0.1)
    assert noise.used_ik_target_mask.tolist() == [True]
    assert noise.target_changed_release_mask.tolist() == [False]
    np.testing.assert_array_equal(noise.pre_limiter_target, noise_target)

    new_target = np.array([0.65])
    changed = control.request([0.6], [0.1], new_target, 2.0, 0.1)
    assert changed.target_changed_release_mask.tolist() == [True]
    assert changed.used_ik_target_mask.tolist() == [False]
    assert changed.pre_limiter_target[0] == pytest.approx(
        0.6 + 2.0 * (0.65 - 0.1) * 0.1
    )


def test_target_reversal_and_moving_targets_resume_outer_tracking_immediately():
    control = PostCrossingTargetHold(1)
    control.observe_published_command([0.4], [0.6], [0.5])

    reversal = control.request([0.6], [0.2], [-0.5], 2.0, 0.1)
    assert reversal.target_changed_release_mask.tolist() == [True]
    assert reversal.used_ik_target_mask.tolist() == [False]
    assert reversal.pre_limiter_target[0] < 0.6

    control.observe_published_command([0.6], [-0.6], [-0.5])
    moving = control.request([-0.6], [-0.8], [-0.4], 2.0, 0.1)
    assert moving.target_changed_release_mask.tolist() == [True]
    assert moving.used_ik_target_mask.tolist() == [False]
    assert moving.pre_limiter_target[0] > -0.6


def test_held_target_still_passes_through_existing_joint_limiters():
    control = PostCrossingTargetHold(1)
    target = np.array([0.5])
    crossed_command = np.array([0.7])
    control.observe_published_command([0.4], crossed_command, target)
    request = control.request(crossed_command, [0.0], target, 2.0, 0.1)

    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([0.1]),
        max_lead=np.array([0.05]),
        command_period_sec=0.1,
        names=("joint",),
    )
    # Prime the existing limiter's previous-command state at the crossing
    # command, just as the production publish path did one cycle earlier.
    gate._last = crossed_command.copy()
    limited, reason = gate.follow(
        request.pre_limiter_target, np.array([0.0]), 0.1
    )

    assert request.pre_limiter_target[0] == pytest.approx(target[0])
    assert reason == "velocity and lead limit"
    assert limited[0] == pytest.approx(0.05)
    assert limited[0] != pytest.approx(target[0])


def test_perfect_tracking_path_is_unchanged():
    control = PostCrossingTargetHold(2)
    target = np.array([0.25, -0.25])
    request = control.request(target, target, target, 2.0, 0.01)
    crossed = control.observe_published_command(
        target, request.pre_limiter_target, target
    )

    np.testing.assert_array_equal(request.raw_candidate, target)
    np.testing.assert_array_equal(request.pre_limiter_target, target)
    assert request.used_ik_target_mask.tolist() == [False, False]
    assert crossed.tolist() == [False, False]
    assert control.hold_mask.tolist() == [False, False]
