"""Designing a set of poses that can actually separate the parameters.

A set that swings the shoulder while leaving the wrist in the same orientation
identifies the shoulder and tells you nothing new about the wrist. The designer's
whole job is to make that visible before the arm moves, not after the fit has
produced a confident wrong number.
"""

import numpy as np
import pytest

from robot_control.identification import (
    FitError,
    MAX_CONDITION,
    design_pose_set,
)


LOWER = np.array([-3.14, -2.0, -3.14])
UPPER = np.array([3.14, 2.0, 3.14])
SCALES = (0.0, 0.5, 1.0)


def _planar(q):
    """Gravity torque of a three-link planar arm, unit masses a unit apart."""
    q = np.asarray(q, dtype=float)
    angles = np.cumsum(q)
    return np.array([float(np.sum(np.cos(angles[index:]))) for index in range(len(q))])


def test_the_designed_set_conditions_every_joint():
    design = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4)

    assert design.worst_condition < MAX_CONDITION, design.condition
    assert design.poses.shape == (4, 3)


def test_the_designed_set_stays_inside_the_limits():
    design = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=6)

    assert np.all(design.poses >= LOWER)
    assert np.all(design.poses <= UPPER)


def test_reach_keeps_the_set_away_from_the_hard_stops():
    """A pose against a stop cannot droop, and a joint that cannot droop looks
    exactly like one held by stiction."""
    design = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4, reach=0.5)

    middle = (LOWER + UPPER) / 2
    span = (UPPER - LOWER) / 2
    assert np.all(np.abs(design.poses - middle) <= span * 0.5 + 1e-9)


def test_the_same_seed_designs_the_same_set():
    """A dry run is only a review if --execute visits the poses that were shown."""
    first = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4, seed=7)
    again = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4, seed=7)
    other = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4, seed=8)

    np.testing.assert_array_equal(first.poses, again.poses)
    assert not np.array_equal(first.poses, other.poses)


def test_the_second_pose_is_what_makes_the_fit_possible():
    """One pose is one equation in three unknowns; the second one separates them."""
    one = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=1)
    two = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=2)

    assert one.worst_condition > MAX_CONDITION
    assert two.worst_condition < MAX_CONDITION


def test_more_poses_stay_well_conditioned():
    """Extra poses buy redundancy against noise rather than conditioning.

    They can nudge the number either way: it is a ratio of singular values of a
    column-normalised matrix, not a monotone function of added rows.
    """
    four = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=4, seed=3)
    seven = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=7, seed=3)

    assert seven.worst_condition < MAX_CONDITION
    assert seven.worst_condition <= four.worst_condition * 1.1


def test_a_joint_whose_load_never_varies_is_named_as_the_worst():
    def stuck(q):
        torque = _planar(q)
        torque[1] = 2.0  # constant however the arm is posed
        return torque

    design = design_pose_set(stuck, LOWER, UPPER, scales=SCALES, poses=5)

    assert design.worst_joint == 1
    assert not np.isfinite(design.condition[1]) or design.condition[1] > MAX_CONDITION


def test_one_pose_is_never_conditioned():
    design = design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=1)

    assert design.worst_condition > MAX_CONDITION


def test_a_set_needs_at_least_one_pose():
    with pytest.raises(FitError, match="pose"):
        design_pose_set(_planar, LOWER, UPPER, scales=SCALES, poses=0)


def test_a_set_needs_more_than_one_distinct_scale():
    """One scale gives one column of feedforward torque, all of it zero if the
    scale is zero, and no way to see the joint respond to torque at all."""
    with pytest.raises(FitError, match="scale"):
        design_pose_set(_planar, LOWER, UPPER, scales=(1.0,), poses=4)


def test_limits_that_do_not_bracket_are_refused():
    with pytest.raises(FitError, match="limit"):
        design_pose_set(_planar, UPPER, LOWER, scales=SCALES, poses=4)


def test_the_designed_set_conditions_a_real_chain():
    """The same designer against forward kinematics rather than a formula."""
    from robot_control import kinematics

    joints = tuple(
        kinematics.Revolute(
            f"j{index}",
            np.array([0.0, 1.0, 0.0]),
            np.zeros(3) if index == 0 else np.array([0.3, 0.0, 0.0]),
            np.eye(3),
            f"l{index}",
        )
        for index in range(3)
    )
    links = tuple(
        kinematics.Link(f"l{index}", 1.5, np.array([0.15, 0.0, 0.0]))
        for index in range(3)
    )
    chain = kinematics.Chain(joints, links)

    design = design_pose_set(
        chain.gravity_torque, LOWER, UPPER, scales=SCALES, poses=4
    )

    assert design.worst_condition < MAX_CONDITION, design.condition
