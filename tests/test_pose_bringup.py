"""The bringup wrapper inverts the vendor default: fake hardware unless asked."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).parents[1]
BRINGUP = ROOT / "ros_ws/pose_bringup.sh"
RECORD = "ros2_arguments.txt"


@pytest.fixture
def workspace(tmp_path):
    """A throwaway workspace with a fake ros2 that records its arguments."""
    ros_ws = tmp_path / "ros_ws"
    (ros_ws / "install").mkdir(parents=True)
    # Real colcon and ament setup files dereference these without a default,
    # which aborts under `set -u`.
    (ros_ws / "install/setup.bash").write_text(
        'if [ -n "$COLCON_TRACE" ]; then echo "# trace"; fi\n'
        'if [ -n "$AMENT_TRACE_SETUP_FILES" ]; then echo "# trace"; fi\n'
        "export ROBOT_CONTROL_POSE_SETUP_SOURCED=1\n"
    )
    shutil.copy2(BRINGUP, ros_ws)

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        'test -n "${ROBOT_CONTROL_POSE_SETUP_SOURCED:-}" || exit 3\n'
        f'printf "%s\\n" "$@" > "{tmp_path / RECORD}"\n'
    )
    fake_ros2.chmod(0o755)
    return tmp_path


def _run(workspace, *arguments, distro="jazzy"):
    return subprocess.run(
        ["bash", str(workspace / "ros_ws/pose_bringup.sh"), *arguments],
        env=dict(
            os.environ,
            ROS_DISTRO=distro,
            PATH=f"{workspace / 'bin'}:{os.environ['PATH']}",
        ),
        text=True,
        capture_output=True,
    )


def _recorded(workspace) -> list[str]:
    record = workspace / RECORD
    return record.read_text().splitlines() if record.is_file() else []


def test_bringup_defaults_to_fake_hardware(workspace):
    result = _run(workspace)

    assert result.returncode == 0, result.stderr
    arguments = _recorded(workspace)
    assert "use_fake_hardware:=true" in arguments
    assert "demo.launch.py" in arguments
    # The vendor default would open can0 and can1; nothing here may name a
    # real bus.
    assert not any(argument.endswith(("can0", "can1")) for argument in arguments)


def test_bringup_never_invokes_ros2_outside_jazzy(workspace):
    result = _run(workspace, distro="humble")

    assert result.returncode == 2
    assert "Jazzy" in result.stderr
    assert _recorded(workspace) == []


def test_bringup_requires_a_built_workspace(workspace):
    (workspace / "ros_ws/install/setup.bash").unlink()

    result = _run(workspace)

    assert result.returncode == 2
    assert "setup.bash" in result.stderr
    assert _recorded(workspace) == []


def test_real_hardware_requires_both_can_interfaces(workspace):
    for arguments in ([], ["--right-can", "can0"], ["--left-can", "can1"]):
        result = _run(workspace, "--real", *arguments)

        assert result.returncode == 2, arguments
        assert "--right-can" in result.stderr or "--left-can" in result.stderr
        assert _recorded(workspace) == []


def test_real_hardware_passes_the_named_buses(workspace):
    result = _run(workspace, "--real", "--right-can", "can0", "--left-can", "can1")

    assert result.returncode == 0, result.stderr
    arguments = _recorded(workspace)
    assert "use_fake_hardware:=false" in arguments
    assert "right_can_interface:=can0" in arguments
    assert "left_can_interface:=can1" in arguments


def test_bringup_rejects_an_unknown_option(workspace):
    result = _run(workspace, "--turbo")

    assert result.returncode == 2
    assert "--turbo" in result.stderr
    assert _recorded(workspace) == []


def test_bringup_forwards_extra_launch_arguments(workspace):
    result = _run(workspace, "--", "arm_prefix:=demo_")

    assert result.returncode == 0, result.stderr
    assert "arm_prefix:=demo_" in _recorded(workspace)
