"""Canonical-named asset URDFs as the gravity model, alongside bringup naming.

The generated RL assets (urdf/generated/rl/*) name joints canonically
(r_aj_1...) while the bringup description names them at the source
(openarm_right_joint1...). The chain builder must accept either file for the
same group, so the asset URDF — whose masses include the real hand — can serve
as the gravity model everywhere the bringup dump did.
"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_control.cli import _gravity_chain, _group_chain, _parser
from robot_control.kinematics import KinematicsError
from robot_control.profile import ProfileError, load_builtin_profile, load_profile


GROUP = "openarm_right_arm"
SOURCE_TIP = "openarm_right_hand_tcp"
ASSET_TIP = "r_hl_palm_ee"


def _urdf(joint_of, link_of, tip):
    """A seven-joint arm about +y, in whichever naming *joint_of* produces."""
    parts = ['<robot name="stub">', '<link name="base"/>']
    parent = "base"
    for index in range(1, 8):
        link = link_of(index)
        origin = "0 0 0" if index == 1 else "0.12 0 0"
        parts.append(
            f'<joint name="{joint_of(index)}" type="revolute">'
            f'<parent link="{parent}"/><child link="{link}"/>'
            f'<origin xyz="{origin}" rpy="0 0 0"/><axis xyz="0 1 0"/>'
            f'<limit lower="-3.2" upper="3.2" effort="50" velocity="2"/></joint>'
        )
        parts.append(
            f'<link name="{link}"><inertial>'
            f'<origin xyz="0.06 0 0" rpy="0 0 0"/><mass value="0.35"/>'
            f'<inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>'
            f"</inertial></link>"
        )
        parent = link
    parts.append(
        f'<joint name="stub_tcp" type="fixed">'
        f'<parent link="{parent}"/><child link="{tip}"/>'
        f'<origin xyz="0.08 0 0" rpy="0 0 0"/></joint>'
    )
    parts.append(f'<link name="{tip}"/>')
    parts.append("</robot>")
    return "".join(parts)


def _source_urdf():
    return _urdf(
        lambda i: f"openarm_right_joint{i}",
        lambda i: f"openarm_right_link{i}",
        SOURCE_TIP,
    )


def _asset_urdf():
    return _urdf(lambda i: f"r_aj_{i}", lambda i: f"r_al_{i}", ASSET_TIP)


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


@pytest.fixture
def group(profile):
    return profile.groups[GROUP]


def test_profile_declares_the_asset_urdf_and_canonical_tips(profile):
    assert profile.asset_urdf_path is not None
    assert profile.asset_urdf_path.is_file()
    assert profile.groups["openarm_right_arm"].asset_tip_link == "r_hl_palm_ee"
    assert profile.groups["openarm_left_arm"].asset_tip_link == "l_hl_gripper_tcp"


def test_the_declared_asset_urdf_builds_every_arm_chain(profile):
    """The real generated URDF, not a stub: canonical names must resolve."""
    text = profile.asset_urdf_path.read_text()
    for name in ("openarm_right_arm", "openarm_left_arm"):
        chain = _group_chain(text, profile, profile.groups[name])
        torque = chain.gravity_torque(np.zeros(7))
        assert torque.shape == (7,)
        # A real arm with the hand on carries a standing load somewhere.
        assert np.any(np.abs(torque) > 1e-3)


def test_group_chain_reads_source_naming(profile, group):
    chain = _group_chain(_source_urdf(), profile, group)
    assert chain.gravity_torque(np.zeros(7)).shape == (7,)


def test_group_chain_reads_canonical_naming(profile, group):
    source = _group_chain(_source_urdf(), profile, group)
    asset = _group_chain(_asset_urdf(), profile, group)

    q = np.linspace(-0.5, 0.5, 7)
    np.testing.assert_allclose(
        asset.gravity_torque(q), source.gravity_torque(q), atol=1e-12
    )


def test_group_chain_without_asset_tip_link_names_the_gap(profile, group):
    bare = replace(group, asset_tip_link=None)
    with pytest.raises(KinematicsError, match="asset_tip_link"):
        _group_chain(_asset_urdf(), profile, bare)


def test_group_chain_rejects_a_urdf_matching_neither_naming(profile, group):
    stranger = _urdf(lambda i: f"who_{i}", lambda i: f"where_{i}", "tip")
    with pytest.raises(KinematicsError) as caught:
        _group_chain(stranger, profile, group)
    message = str(caught.value)
    assert "openarm_right_joint1" in message
    assert "r_aj_1" in message


def test_gravity_chain_prefers_the_override_file(profile, group, tmp_path):
    """With a --urdf override the running stack is never consulted."""

    class NoAdapter:
        def read_robot_description(self):
            raise AssertionError("the override should keep the stack out of it")

    path = tmp_path / "asset.urdf"
    path.write_text(_asset_urdf())
    chain = _gravity_chain(NoAdapter(), profile, group, path)
    assert chain.gravity_torque(np.zeros(7)).shape == (7,)


@pytest.mark.parametrize(
    "argv",
    [
        ["pose", "gravity", "--group", GROUP, "--scale", "1.0", "--urdf", "a.urdf"],
        ["pose", "torque", "--group", GROUP, "--urdf", "a.urdf"],
        ["pose", "follow", "--group", GROUP, "--urdf", "a.urdf"],
        ["r2s", "identify", "--collect", "--group", GROUP, "--urdf", "a.urdf"],
    ],
)
def test_live_gravity_commands_accept_a_urdf_override(argv):
    args = _parser().parse_args(argv)
    assert Path(args.urdf) == Path("a.urdf")


def test_profile_rejects_a_missing_asset_urdf(tmp_path):
    source = Path(load_builtin_profile("openarm_tesollo").manifest_path)
    manifest = tmp_path / source.name
    manifest.write_text(source.read_text())
    raw = yaml.safe_load(
        (Path(__file__).parents[1] / "src/robot_control/profiles/openarm_tesollo.yaml")
        .read_text()
    )
    raw["asset"]["manifest"] = str(manifest)
    raw["asset"]["urdf"] = str(tmp_path / "nowhere.urdf")
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(raw))

    with pytest.raises(ProfileError, match="asset urdf"):
        load_profile(path)
