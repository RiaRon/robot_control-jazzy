from pathlib import Path
import re

import pytest
import yaml

from robot_control.profile import ProfileError, load_profile


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / "src/robot_control/profiles/openarm_tesollo.yaml"




def test_openarm_tesollo_profile_has_complete_canonical_contract():
    profile = load_profile(PROFILE)

    assert profile.name == "openarm_tesollo"
    assert profile.ros["humble"].command_rate_hz == 100
    assert profile.ros["jazzy"].command_rate_hz == 100
    assert len(profile.joints) == 35
    assert set(profile.groups) == {
        "openarm_right_arm",
        "openarm_left_arm",
        "openarm_left_gripper",
        "tesollo_abduction",
        "tesollo_curl",
        "tesollo_pip",
        "tesollo_dip",
    }
    assert set().union(*(set(group.joints) for group in profile.groups.values())) == set(
        profile.joint_names
    )


def test_group_contract_declares_openarm_controllers_and_moveit_groups():
    groups = load_profile(PROFILE).groups

    assert groups["openarm_right_arm"].controller == "right_joint_trajectory_controller"
    assert groups["openarm_right_arm"].moveit_group == "right_arm"
    assert groups["openarm_right_arm"].action == "follow_joint_trajectory"
    assert groups["openarm_right_arm"].tip_link == "openarm_right_hand_tcp"

    assert groups["openarm_left_arm"].controller == "left_joint_trajectory_controller"
    assert groups["openarm_left_arm"].moveit_group == "left_arm"
    assert groups["openarm_left_arm"].action == "follow_joint_trajectory"
    assert groups["openarm_left_arm"].tip_link == "openarm_left_hand_tcp"

    # The gripper is driven by parallel_gripper_action_controller, not a
    # trajectory controller, so the action must be declared rather than assumed.
    assert groups["openarm_left_gripper"].controller == "left_gripper_controller"
    assert groups["openarm_left_gripper"].moveit_group == "left_gripper"
    assert groups["openarm_left_gripper"].action == "parallel_gripper_command"


def test_group_contract_marks_tesollo_groups_executable_without_moveit():
    profile = load_profile(PROFILE)

    tesollo = [name for name in profile.groups if name.startswith("tesollo_")]
    assert tesollo
    for name in tesollo:
        group = profile.groups[name]
        # The DG5F hand has no IK solver configured, so it is reachable by
        # direct joint values only.
        assert group.controller == "joint_trajectory_controller"
        assert group.moveit_group is None
        assert name in profile.executable_groups()




def test_profile_rejects_manifest_hash_mismatch(tmp_path):
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text("control_joint_order: [r_aj_1]\n")
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        """
name: bad
components: [openarm]
asset:
  id: asset
  manifest: manifest.yaml
  manifest_sha256: deadbeef
joints:
  - {canonical: r_aj_1, source: joint1, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}
groups:
  arm: {joints: [r_aj_1]}
ros:
  humble: {command_topic: /cmd, state_topic: /state, controller: c, command_rate_hz: 100}
"""
    )

    with pytest.raises(ProfileError, match="manifest hash mismatch"):
        load_profile(profile)


def test_profile_resolves_hdgp_manifest_from_workspace_ancestor(tmp_path):
    workspace = tmp_path / "workspace"
    profile_dir = workspace / "robot_control/.worktrees/jazzy/src/robot_control/profiles"
    manifest = workspace / "hdgp/assets/robot/example/manifest.yaml"
    profile_dir.mkdir(parents=True)
    manifest.parent.mkdir(parents=True)
    manifest.write_text("control_joint_order: [j1]\n")

    import hashlib

    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    profile = profile_dir / "profile.yaml"
    profile.write_text(
        f"""
name: worktree
components: [openarm]
asset:
  id: example
  manifest: ../../../../hdgp/assets/robot/example/manifest.yaml
  manifest_sha256: {digest}
joints:
  - {{canonical: j1, source: j1, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
groups:
  arm: {{joints: [j1]}}
ros:
  jazzy: {{command_topic: /cmd, state_topic: /state, controller: c, command_rate_hz: 100}}
"""
    )

    assert load_profile(profile).manifest_path == manifest


def test_profile_rejects_unknown_or_duplicate_group_coverage(tmp_path):
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text("control_joint_order: [j1, j2]\n")
    import hashlib

    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        f"""
name: bad
components: [openarm]
asset: {{id: asset, manifest: manifest.yaml, manifest_sha256: {digest}}}
joints:
  - {{canonical: j1, source: a, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
  - {{canonical: j2, source: b, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
groups:
  one: {{joints: [j1, j2]}}
  two: {{joints: [j2]}}
ros:
  jazzy: {{command_topic: /cmd, state_topic: /state, controller: c, command_rate_hz: 100}}
"""
    )

    with pytest.raises(ProfileError, match="exactly one actuator group"):
        load_profile(profile)


def test_group_contract_excludes_groups_without_a_controller(tmp_path):
    profile = _write_two_group_profile(tmp_path, extra="")

    loaded = load_profile(profile)
    assert set(loaded.groups) == {"one", "two"}
    assert set(loaded.executable_groups()) == {"one"}


def test_group_contract_rejects_moveit_group_without_controller(tmp_path):
    profile = _write_two_group_profile(tmp_path, extra="    moveit_group: orphan\n")

    with pytest.raises(ProfileError, match="moveit_group.*without a controller"):
        load_profile(profile)


def test_group_contract_rejects_tip_link_without_moveit_group(tmp_path):
    profile = _write_two_group_profile(
        tmp_path, extra="    controller: c2\n    tip_link: hand\n"
    )

    with pytest.raises(ProfileError, match="tip_link.*without a moveit_group"):
        load_profile(profile)


def test_group_contract_rejects_unknown_action(tmp_path):
    profile = _write_two_group_profile(
        tmp_path, extra="    controller: c2\n    action: teleport\n"
    )

    with pytest.raises(ProfileError, match="unsupported action"):
        load_profile(profile)


def _write_two_group_profile(tmp_path: Path, extra: str) -> Path:
    """Write a minimal two-group profile; ``extra`` is appended to group ``two``."""
    import hashlib

    manifest = tmp_path / "manifest.yaml"
    manifest.write_text("control_joint_order: [j1, j2]\n")
    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        f"""
name: groups
components: [openarm]
asset: {{id: asset, manifest: manifest.yaml, manifest_sha256: {digest}}}
joints:
  - {{canonical: j1, source: a, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
  - {{canonical: j2, source: b, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
groups:
  one:
    joints: [j1]
    controller: c1
  two:
    joints: [j2]
{extra}ros:
  jazzy: {{command_topic: /cmd, state_topic: /state, controller: c, command_rate_hz: 100}}
"""
    )
    return profile
