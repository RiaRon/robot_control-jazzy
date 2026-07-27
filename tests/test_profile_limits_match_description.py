"""The profile's joint limits, against the description the robot is built from.

The profile bounds what may be commanded, and `design_pose_set` samples poses
inside those bounds. When they were placeholders — a round +-3.14 and +-2.0 on
every arm joint — the designer produced poses past the hard stops. The arm
juddered against them, could not droop, and the sweep read as stiction: the
tracking error came back equal to the whole commanded angle, and every gravity
scale measured identically.

The effort numbers mattered more. The profile authorized 20 N.m on wrist joints
the description rates at 7, so the gate would have approved roughly three times
what the hardware is built for on the one path that publishes torque.

Widths rather than endpoints, because the bimanual xacro mirrors each arm and
offsets joint 2 by a quarter turn per side. Mirroring and offsetting both
preserve a range's width, so this compares the invariant instead of restating
the transform and drifting from it.
"""

from pathlib import Path

import pytest
import yaml

from robot_control.profile import load_builtin_profile
from robot_control.srdf import repository_root


LIMITS = "ros_ws/src/openarm_description/config/arm/v10/joint_limits.yaml"
ARM_GROUPS = ("openarm_right_arm", "openarm_left_arm")
TOLERANCE_RAD = 1e-3


@pytest.fixture(scope="module")
def described():
    path = repository_root() / LIMITS
    if not path.is_file():
        pytest.skip(f"vendored joint limits not found: {path}")
    raw = yaml.safe_load(path.read_text())
    return {
        name: body["limit"]
        for name, body in raw.items()
        if isinstance(body, dict) and "limit" in body
    }


@pytest.fixture(scope="module")
def profile():
    return load_builtin_profile("openarm_tesollo")


def _arm_joints(profile):
    """Each arm joint, paired with the description entry it is built from."""
    by_canonical = {joint.canonical: joint for joint in profile.joints}
    for group_name in ARM_GROUPS:
        for index, canonical in enumerate(profile.groups[group_name].joints, start=1):
            yield by_canonical[canonical], f"joint{index}"


def test_every_arm_joint_spans_the_range_the_description_gives_it(profile, described):
    for joint, key in _arm_joints(profile):
        limit = described[key]
        expected = float(limit["upper"]) - float(limit["lower"])
        actual = joint.upper - joint.lower

        assert actual == pytest.approx(expected, abs=TOLERANCE_RAD), (
            f"{joint.canonical} spans {actual:.4f} rad against the description's "
            f"{expected:.4f}; a wider profile designs poses past the hard stops"
        )


def test_no_joint_may_be_driven_past_what_the_hardware_is_rated_for(profile, described):
    for joint, key in _arm_joints(profile):
        rated = float(described[key]["effort"])

        assert joint.effort <= rated, (
            f"{joint.canonical} authorizes {joint.effort:g} N.m against a rated "
            f"{rated:g}; r2s identify publishes torque through this bound"
        )


def test_commanded_speed_stays_within_the_description(profile, described):
    for joint, key in _arm_joints(profile):
        rated = float(described[key]["velocity"])

        assert joint.velocity <= rated, (
            f"{joint.canonical} allows {joint.velocity:g} rad/s against a rated "
            f"{rated:g}"
        )


#: A step between identification poses, in radians. Not a limit — a typical
#: move, used to state the default duration as the speed an operator sees.
TYPICAL_REPOSITIONING_RAD = 1.5
REACTABLE_RAD_PER_SEC = 0.2


def test_a_commanded_move_is_slow_enough_to_be_stopped_by_hand():
    """The bound that actually governs how fast the arm goes, and why here.

    The joint velocity limit cannot serve this. The excitation is a small fast
    dither whose peak slew is frequency times amplitude, so a cap low enough to
    slow a large repositioning refuses the measurement outright — and at 3 s a
    1.5 rad pose change already ran at 0.5 rad/s, which a lower cap would not
    have caught either. The duration is what the operator experiences.
    """
    from robot_control.cli import DEFAULT_DURATION_SEC

    speed = TYPICAL_REPOSITIONING_RAD / DEFAULT_DURATION_SEC

    assert speed <= REACTABLE_RAD_PER_SEC, (
        f"a {TYPICAL_REPOSITIONING_RAD:g} rad move over the default "
        f"{DEFAULT_DURATION_SEC:g} s runs at {speed:.2f} rad/s, faster than an "
        "operator can react to"
    )
