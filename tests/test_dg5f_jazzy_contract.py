from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
PROFILE = ROOT / "src/robot_control/profiles/openarm_tesollo.yaml"
DG5F = ROOT / "ros_ws/src/delto_m_ros2/dg5f_gz"
XACROS = (
    DG5F / "urdf/dg5f_right_gz.xacro",
    DG5F / "urdf/dg5f_left_gz.xacro",
    DG5F / "urdf/dg5f_both_gz.xacro",
)
LAUNCH_FILES = tuple(sorted((DG5F / "launch").glob("dg5f_*_gz.launch.py")))
LEGACY_IDENTIFIERS = (
    "ign_ros2_control",
    "IgnitionSystem",
    "IgnitionROS2ControlPlugin",
    "libign_ros2_control",
    "IGN_GAZEBO_RESOURCE_PATH",
)
XACRO_NAMESPACE = {"xacro": "http://www.ros.org/wiki/xacro"}


def _right_joint_names() -> set[str]:
    profile = yaml.safe_load(PROFILE.read_text())
    return {
        joint["source"]
        for joint in profile["joints"]
        if joint["source"].startswith("rj_dg_")
    }


def _ros2_control_joint_names(xacro: Path) -> list[str]:
    root = ET.parse(xacro).getroot()
    return [joint.attrib["name"] for joint in root.findall(".//ros2_control/joint")]


def _assert_xacro_hardware_selection(xacro: Path) -> ET.Element:
    root = ET.parse(xacro).getroot()
    argument = root.find("xacro:arg", XACRO_NAMESPACE)
    assert argument is not None
    assert argument.attrib == {"name": "use_fake_hardware", "default": "false"}

    hardware = root.find(".//ros2_control/hardware")
    assert hardware is not None
    fake_hardware = hardware.find("xacro:if", XACRO_NAMESPACE)
    gazebo_hardware = hardware.find("xacro:unless", XACRO_NAMESPACE)
    assert fake_hardware is not None
    assert fake_hardware.attrib == {"value": "$(arg use_fake_hardware)"}
    assert fake_hardware.findtext("plugin") == "mock_components/GenericSystem"
    assert gazebo_hardware is not None
    assert gazebo_hardware.attrib == {"value": "$(arg use_fake_hardware)"}
    assert gazebo_hardware.findtext("plugin") == "gz_ros2_control/GazeboSimSystem"
    return root


def test_right_dg5f_joint_contract_matches_xacros_and_controller():
    """Fails if a canonical right DG5F joint loses simulation coverage."""
    expected_right_joints = _right_joint_names()
    assert len(expected_right_joints) == 20

    right_xacro_joints = _ros2_control_joint_names(XACROS[0])
    assert set(right_xacro_joints) == expected_right_joints
    assert len(right_xacro_joints) == len(expected_right_joints)

    both_xacro_joints = _ros2_control_joint_names(XACROS[2])
    both_right_joints = [
        joint for joint in both_xacro_joints if joint.startswith("rj_dg_")
    ]
    assert set(both_right_joints) == expected_right_joints
    assert len(both_right_joints) == len(expected_right_joints)

    controller = yaml.safe_load(
        (DG5F / "config/dg5f_right_gz_controller.yaml").read_text()
    )
    controller_joints = controller["joint_trajectory_controller"]["ros__parameters"][
        "joints"
    ]
    assert set(controller_joints) == expected_right_joints
    assert len(controller_joints) == len(expected_right_joints)


def test_all_dg5f_xacros_select_fake_or_gazebo_jazzy_plugins():
    """Fails if either safe fake hardware or Jazzy Gazebo control is removed."""
    for xacro in XACROS:
        root = _assert_xacro_hardware_selection(xacro)
        assert "gz_ros2_control::GazeboSimROS2ControlPlugin" in {
            element.attrib["name"] for element in root.findall(".//gazebo/plugin")
        }


def test_dg5f_package_declares_single_gz_ros2_control_dependency():
    """Fails if the Jazzy control dependency is omitted or duplicated."""
    package = ET.parse(DG5F / "package.xml").getroot()
    package_dependencies = [dependency.text for dependency in package.findall("depend")]
    assert package_dependencies.count("gz_ros2_control") == 1


def test_dg5f_sources_exclude_legacy_ignition_identifiers():
    """Fails if DG5F simulation sources return to an unsupported Ignition API."""
    sources = [DG5F / "package.xml", *XACROS, *LAUNCH_FILES]
    text = "\n".join(source.read_text() for source in sources)
    for identifier in LEGACY_IDENTIFIERS:
        assert identifier not in text


def test_all_dg5f_launches_forward_use_fake_hardware_to_xacro():
    """Fails if an operator's fake-hardware request stops reaching xacro."""
    assert len(LAUNCH_FILES) == 3
    for launch_file in LAUNCH_FILES:
        text = launch_file.read_text()
        assert 'DeclareLaunchArgument(\n            "use_fake_hardware"' in text
        assert '"use_fake_hardware",\n            default_value="false"' in text
        assert 'LaunchConfiguration("use_fake_hardware")' in text
        assert '"use_fake_hardware:="' in text


def test_all_dg5f_launches_isolate_fake_control_from_gazebo():
    """Fails if fake mode can start Gazebo or omit its standalone control node."""
    for launch_file in LAUNCH_FILES:
        text = launch_file.read_text()
        assert "OnProcessStart" in text
        assert "condition=IfCondition(use_fake_hardware)" in text
        assert text.count("condition=UnlessCondition(use_fake_hardware)") >= 2
        assert "target_action=control_node" in text
        assert '"--controller-manager-timeout", "30"' in text


def test_all_dg5f_launches_make_gui_false_headless():
    """Fails if the gui argument stops selecting Gazebo server-only mode."""
    for launch_file in LAUNCH_FILES:
        text = launch_file.read_text()
        assert "PythonExpression" in text
        assert '" -s -r empty.sdf -v 0"' in text
        assert '" -r empty.sdf -v 0"' in text


def test_all_dg5f_launches_register_handlers_before_target_actions():
    """Fails if a fast target process can emit before its handler is registered."""
    for launch_file in LAUNCH_FILES:
        nodes = launch_file.read_text().split("    nodes = [", 1)[1]
        first_handler = nodes.index("RegisterEventHandler(")
        assert first_handler < nodes.index("\n        control_node,")
        assert first_handler < nodes.index("\n        gz_spawn_entity,")
