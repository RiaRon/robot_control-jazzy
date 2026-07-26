"""The profile against the vendored MoveIt configuration.

Not part of the neutral core. These assertions are about a *vendored tree*, and
the two long-lived branches vendor different upstream trees — so unlike the rest
of the suite, what these tests can assert depends on which branch they run on.

They skip, loudly, when the vendored configuration is of a generation that
predates the frames the profile names. A skip says "this branch's vendor
snapshot cannot support the pose commands yet"; it does not say the profile is
fine. Partial agreement still fails, so a real regression cannot hide behind it.
"""

from pathlib import Path
import re

import pytest
import yaml

from robot_control.profile import load_profile


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / "src/robot_control/profiles/openarm_tesollo.yaml"


def _moveit_config():
    """Wherever the vendored SRDF lives is the configuration directory."""
    from robot_control.srdf import find_srdf

    return find_srdf().parent


@pytest.fixture(scope="module")
def srdf():
    text = _moveit_config().joinpath("openarm_bimanual.srdf").read_text()
    tips = {group.tip_link for group in load_profile(PROFILE).groups.values()}
    tips.discard(None)
    present = {tip for tip in tips if tip in text}
    if not present:
        pytest.skip(
            "the vendored MoveIt configuration declares none of the profile's "
            f"tip links ({sorted(tips)}); this branch's snapshot predates them, "
            "so the pose commands cannot resolve an end-effector marker here"
        )
    return text



    return find_srdf().parent


def test_group_contract_matches_vendored_moveit_configuration(srdf):
    """Declared names must exist in the vendored MoveIt configuration.

    This fails if a vendor snapshot bump renames a controller or a planning
    group, which would otherwise only surface as a runtime action timeout.
    """
    profile = load_profile(PROFILE)
    source_by_canonical = {joint.canonical: joint.source for joint in profile.joints}


    srdf_groups = set(re.findall(r'<group name="([^"]+)"', srdf))
    srdf_tips = set(re.findall(r'<end_effector [^>]*parent_link="([^"]+)"', srdf))
    controllers = yaml.safe_load(
        _moveit_config().joinpath("moveit_controllers.yaml").read_text()
    )
    controllers = controllers["moveit_simple_controller_manager"]

    openarm = {
        name: group
        for name, group in profile.executable_groups().items()
        if name.startswith("openarm_")
    }
    assert set(openarm) == {
        "openarm_right_arm",
        "openarm_left_arm",
        "openarm_left_gripper",
    }
    for name, group in openarm.items():
        assert group.moveit_group in srdf_groups, name
        assert group.controller in controllers["controller_names"], name
        declared = tuple(controllers[group.controller]["joints"])
        assert declared == tuple(source_by_canonical[j] for j in group.joints), name
        if group.tip_link is not None:
            assert group.tip_link in srdf_tips, name


def test_tip_link_is_the_tool_centre_point_rviz_anchors_its_marker_to(srdf):
    """RViz and robotctl must mean the same frame by "the end effector".

    MoveIt anchors the interactive end-effector marker at the end effector's
    parent link, but only once it can resolve that end effector's parent group,
    which needs both `parent_group` and the link being inside that group. Until
    both hold it silently falls back to the group's last joint-bearing link,
    which is 0.18 m short of the tool centre point here: an operator would drag
    one frame while robotctl commanded another.
    """
    profile = load_profile(PROFILE)


    end_effectors = dict(
        re.findall(
            r'<end_effector [^>]*parent_link="([^"]+)"[^>]*parent_group="([^"]+)"',
            srdf,
        )
    )
    groups = dict(re.findall(r'<group name="([^"]+)">(.*?)</group>', srdf, re.S))

    for name in ("openarm_right_arm", "openarm_left_arm"):
        group = profile.groups[name]
        tip = group.tip_link
        assert tip is not None and tip.endswith("_hand_tcp"), name
        assert end_effectors.get(tip) == group.moveit_group, name
        assert f'<link name="{tip}"/>' in groups[group.moveit_group], name
