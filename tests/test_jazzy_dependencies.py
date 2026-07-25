import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "ros_ws/install_dependencies_jazzy.sh"


def _environment(tmp_path: Path) -> tuple[dict[str, str], Path]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    calls = tmp_path / "dependency-calls"
    for command in ("sudo", "rosdep"):
        executable = fake_bin / command
        executable.write_text(
            "#!/usr/bin/env bash\n"
            f"printf '%s\\n' '{command}' >> '{calls}'\n"
        )
        executable.chmod(0o755)
    env = dict(os.environ, ROS_DISTRO="humble")
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    return env, calls


def test_dependency_helper_rejects_non_jazzy_before_running_commands(tmp_path):
    env, calls = _environment(tmp_path)
    result = subprocess.run(
        ["bash", str(SCRIPT)], env=env, text=True, capture_output=True
    )

    assert result.returncode == 2
    assert "ROS 2 Jazzy" in result.stderr
    assert not calls.exists()
