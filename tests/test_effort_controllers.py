"""The effort controllers the profile names, and the script that loads them.

`r2s identify` measures stiffness by publishing gravity feedforward while the
trajectory controller keeps holding position. The hardware takes both in one
MIT-mode packet — tau = kp(q_des - q) + kd(qd_des - qd) + tau_ff — so this needs
no controller switching, only a controller claiming the effort interface. The
profile declares one per arm; nothing provided it, and the failure surfaced
three stages into a run against a real robot.

They are defined but not spawned. A torque path that comes up with the robot is
one nobody chose to start; `load_effort_controllers.sh` is that choice.
"""

import stat
import subprocess
from pathlib import Path

import pytest
import yaml

from robot_control.profile import load_builtin_profile
from robot_control.srdf import repository_root


CONTROLLERS = (
    "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_bimanual_controllers.yaml"
)
LOADER = "ros_ws/load_effort_controllers.sh"


@pytest.fixture(scope="module")
def controllers():
    path = repository_root() / CONTROLLERS
    if not path.is_file():
        pytest.skip(f"vendored controllers file not found: {path}")
    return yaml.safe_load(path.read_text())


@pytest.fixture(scope="module")
def profile():
    return load_builtin_profile("openarm_tesollo")


def _compensable(profile):
    return {
        name: group
        for name, group in profile.groups.items()
        if group.compensable
    }


def test_every_effort_controller_the_profile_names_is_declared(controllers, profile):
    declared = controllers["controller_manager"]["ros__parameters"]
    named = {group.effort_controller for group in _compensable(profile).values()}

    assert named, "no group declares an effort_controller; this test is vacuous"
    for controller in sorted(named):
        assert controller in declared, controller
        assert declared[controller]["type"] == (
            "forward_command_controller/ForwardCommandController"
        )


def test_each_claims_the_effort_interface_of_its_own_joints(controllers, profile):
    source = {joint.canonical: joint.source for joint in profile.joints}

    for group in _compensable(profile).values():
        body = controllers[group.effort_controller]["ros__parameters"]
        # The interface, not just the name: a controller called "effort" that
        # writes positions would drive the arm instead of loading it.
        assert body["interface_name"] == "effort"
        assert body["command_interfaces"] == ["effort"]
        assert body["joints"] == [source[name] for name in group.joints]


def test_the_torque_path_does_not_come_up_with_the_robot(controllers):
    """Declared, not spawned. Loading it is a separate, deliberate act."""
    launch = (
        repository_root()
        / "ros_ws/src/openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py"
    )
    if not launch.is_file():
        pytest.skip(f"bringup launch not found: {launch}")

    assert "forward_effort_controller" not in launch.read_text()


def test_the_loader_the_error_message_points_at_exists_and_parses():
    # ros_adapter tells the user to run this by name when nothing is subscribed
    # to the effort topic. It pointed at a file that was never written.
    path = repository_root() / LOADER

    assert path.is_file(), f"{LOADER} does not exist"
    assert path.stat().st_mode & stat.S_IXUSR, f"{LOADER} is not executable"
    subprocess.run(["bash", "-n", str(path)], check=True)


def test_the_loader_names_the_controllers_the_profile_declares(profile):
    path = repository_root() / LOADER
    if not path.is_file():
        pytest.skip(f"{LOADER} does not exist")
    text = path.read_text()

    for group in _compensable(profile).values():
        assert group.effort_controller in text, group.effort_controller
