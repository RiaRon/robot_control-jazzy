import os
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "ros_ws/build.sh"
SUPPORTED_PACKAGES = ROOT / "ros_ws/supported-packages.txt"
DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "buildtool_export_depend",
    "exec_depend",
}


def _local_package_graph() -> dict[str, set[str]]:
    declared_dependencies = {}
    for manifest in sorted((ROOT / "ros_ws/src").rglob("package.xml")):
        package = ET.parse(manifest).getroot()
        name = package.findtext("name")
        assert name is not None
        declared_dependencies[name.strip()] = {
            element.text.strip()
            for element in package
            if element.tag in DEPENDENCY_TAGS and element.text
        }

    local_names = set(declared_dependencies)
    return {
        package: dependencies & local_names
        for package, dependencies in declared_dependencies.items()
    }


def _dependency_order(graph: dict[str, set[str]], roots: list[str]) -> list[str]:
    ordered = []
    visiting = set()
    visited = set()

    def visit(package: str) -> None:
        assert package in graph, f"unknown local package root: {package}"
        assert package not in visiting, f"local package dependency cycle: {package}"
        if package in visited:
            return
        visiting.add(package)
        for dependency in sorted(graph[package]):
            visit(dependency)
        visiting.remove(package)
        visited.add(package)
        ordered.append(package)

    for root in roots:
        visit(root)
    return ordered


def _supported_roots() -> list[str]:
    return [
        line.strip()
        for line in SUPPORTED_PACKAGES.read_text().splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


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


def test_supported_roots_resolve_required_local_dependencies_in_order():
    order = _dependency_order(_local_package_graph(), _supported_roots())

    assert {"openarm_can", "dg_description"} <= set(order)
    assert order.index("openarm_can") < order.index("openarm_hardware")
    assert order.index("dg_description") < order.index("dg5f_gz")


def test_unsupported_gazebo_packages_are_outside_declared_local_closure():
    order = _dependency_order(_local_package_graph(), _supported_roots())

    assert {"dg3f_m_gz", "dg4f_gz"}.isdisjoint(order)
