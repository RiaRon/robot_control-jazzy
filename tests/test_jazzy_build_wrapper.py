import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "ros_ws/build.sh"


def _environment(tmp_path: Path, distro: str) -> tuple[dict[str, str], Path]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    calls = tmp_path / "colcon-calls"
    colcon = fake_bin / "colcon"
    colcon.write_text(
        "#!/usr/bin/env bash\n"
        f"printf '%s\\n' \"$@\" > '{calls}'\n"
    )
    colcon.chmod(0o755)
    env = dict(os.environ, ROS_DISTRO=distro)
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    return env, calls


def test_build_wrapper_rejects_non_jazzy_without_calling_colcon(tmp_path):
    env, calls = _environment(tmp_path, "humble")
    result = subprocess.run(
        [str(SCRIPT)], env=env, text=True, capture_output=True
    )

    assert result.returncode == 2
    assert "ROS 2 Jazzy" in result.stderr
    assert not calls.exists()


def test_build_wrapper_uses_branch_local_products(tmp_path):
    env, calls = _environment(tmp_path, "jazzy")
    subprocess.run([str(SCRIPT)], env=env, check=True)
    arguments = calls.read_text().splitlines()
    workspace = ROOT / "ros_ws"

    assert arguments[:3] == ["--log-base", str(workspace / "log"), "build"]
    assert arguments[3] == "--base-paths"
    assert str(workspace / "src") in arguments
    assert str(workspace / "build") in arguments
    assert str(workspace / "install") in arguments
    assert str(workspace / "log") in arguments
    packages_up_to = arguments.index("--packages-up-to")
    assert arguments[packages_up_to + 1 :] == [
        "openarm",
        "openarm_bimanual_moveit_config",
        "dg5f_driver",
        "dg5f_gz",
    ]
    assert "dg3f_m_gz" not in arguments
    assert "dg4f_gz" not in arguments
