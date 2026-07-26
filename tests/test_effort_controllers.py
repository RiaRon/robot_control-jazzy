"""The effort controller configuration must match the profile it will drive.

A joint list that drifts from the profile would publish torque into the wrong
joint, which is worse than publishing none.
"""

from pathlib import Path
import stat

import pytest
import yaml

from robot_control.profile import load_builtin_profile

ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "ros_ws/config/effort_controllers.yaml"
SCRIPT = ROOT / "ros_ws/load_effort_controllers.sh"


@pytest.fixture(scope="module")
def config() -> dict:
    assert CONFIG.is_file(), f"{CONFIG} is required for gravity compensation"
    return yaml.safe_load(CONFIG.read_text())


def test_effort_controllers_are_declared_as_forward_command_controllers(config):
    declared = config["controller_manager"]["ros__parameters"]
    for side in ("right", "left"):
        name = f"{side}_forward_effort_controller"
        assert name in declared, name
        assert declared[name]["type"] == (
            "forward_command_controller/ForwardCommandController"
        )


def test_effort_controllers_drive_the_effort_interface(config):
    for side in ("right", "left"):
        parameters = config[f"{side}_forward_effort_controller"]["ros__parameters"]
        # Anything else and this would fight the trajectory controller for
        # position rather than adding feedforward torque to it.
        assert parameters["interface_name"] == "effort"


def test_effort_controller_joints_match_the_profile_in_order(config):
    """Order matters: the command is a bare Float64MultiArray with no names."""
    profile = load_builtin_profile("openarm_tesollo")
    source_by_canonical = {joint.canonical: joint.source for joint in profile.joints}

    for side, group_name in (
        ("right", "openarm_right_arm"),
        ("left", "openarm_left_arm"),
    ):
        group = profile.groups[group_name]
        expected = [source_by_canonical[canonical] for canonical in group.joints]
        declared = config[f"{side}_forward_effort_controller"]["ros__parameters"]
        assert declared["joints"] == expected, group_name


def test_loader_script_is_executable_and_refuses_a_non_jazzy_environment():
    assert SCRIPT.is_file(), f"{SCRIPT} is required"
    assert SCRIPT.stat().st_mode & stat.S_IXUSR, "the loader must be executable"
    body = SCRIPT.read_text()
    assert 'ROS_DISTRO:-}" != "jazzy"' in body
    # The type has to be set on the running manager; without this the load fails
    # with "The 'type' param was not defined".
    assert "ros2 param set /controller_manager" in body
