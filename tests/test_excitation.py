"""Sizing the excitation torque, which cannot be computed in advance.

kp is the unknown the experiment is for, so the torque that produces a wanted
deflection is found by pushing a little and extrapolating. Everything here is
arithmetic on a measured response — no robot, no ROS — so it is tested directly
rather than through the CLI.
"""

import pytest

from robot_control.excitation import ExcitationRefused, probe_torque

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
