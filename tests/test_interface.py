import numpy as np
import pytest

from robot_control.interface import CanonicalInterface, InterfaceError
from robot_control.profile import load_builtin_profile


def test_fake_hardware_canonical_source_round_trip():
    profile = load_builtin_profile("openarm_tesollo")
    interface = CanonicalInterface(profile)
    canonical = np.linspace(-0.01, 0.01, len(profile.joints))

    source = interface.command_to_source(canonical)
    restored = interface.state_to_canonical(source)

    assert list(source) == [joint.source for joint in profile.joints]
    np.testing.assert_allclose(restored, canonical)


def test_state_requires_exact_joint_coverage():
    interface = CanonicalInterface(load_builtin_profile("openarm_tesollo"))
    with pytest.raises(InterfaceError, match="coverage"):
        interface.state_to_canonical({"openarm_right_joint1": 0.0})


def test_group_source_names_follow_profile_order():
    interface = CanonicalInterface(load_builtin_profile("openarm_tesollo"))

    assert interface.group_source_names("openarm_right_arm") == tuple(
        f"openarm_right_joint{index}" for index in range(1, 8)
    )
    with pytest.raises(InterfaceError, match="unknown group"):
        interface.group_source_names("openarm_third_arm")


def test_group_command_to_source_round_trips_one_group():
    interface = CanonicalInterface(load_builtin_profile("openarm_tesollo"))
    canonical = np.linspace(-0.03, 0.03, 7)

    source = interface.group_command_to_source("openarm_left_arm", canonical)

    assert list(source) == [f"openarm_left_joint{index}" for index in range(1, 8)]
    np.testing.assert_allclose(
        interface.group_state_to_canonical("openarm_left_arm", source), canonical
    )


def test_group_state_ignores_other_groups_but_requires_its_own():
    """A /joint_states message carries the whole robot, not one group."""
    interface = CanonicalInterface(load_builtin_profile("openarm_tesollo"))
    whole_robot = {f"openarm_right_joint{index}": 0.1 for index in range(1, 8)}
    whole_robot.update({f"rj_dg_1_{index}": 0.2 for index in range(1, 5)})

    canonical = interface.group_state_to_canonical("openarm_right_arm", whole_robot)
    np.testing.assert_allclose(canonical, np.full(7, 0.1))

    del whole_robot["openarm_right_joint4"]
    with pytest.raises(InterfaceError, match="missing.*openarm_right_joint4"):
        interface.group_state_to_canonical("openarm_right_arm", whole_robot)


def test_group_conversion_applies_the_declared_sign(tmp_path):
    import hashlib

    manifest = tmp_path / "manifest.yaml"
    manifest.write_text("control_joint_order: [j1, j2]\n")
    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    path = tmp_path / "profile.yaml"
    path.write_text(
        f"""
name: signed
components: [openarm]
asset: {{id: asset, manifest: manifest.yaml, manifest_sha256: {digest}}}
joints:
  - {{canonical: j1, source: a, sign: 1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
  - {{canonical: j2, source: b, sign: -1, unit: rad, lower: -1, upper: 1, velocity: 1, effort: 1}}
groups:
  arm: {{joints: [j1, j2], controller: c1}}
ros:
  jazzy: {{command_topic: /cmd, state_topic: /state, controller: c, command_rate_hz: 100}}
"""
    )
    from robot_control.profile import load_profile

    interface = CanonicalInterface(load_profile(path))

    assert interface.group_command_to_source("arm", [0.5, 0.5]) == {"a": 0.5, "b": -0.5}
    np.testing.assert_allclose(
        interface.group_state_to_canonical("arm", {"a": 0.5, "b": -0.5}), [0.5, 0.5]
    )
