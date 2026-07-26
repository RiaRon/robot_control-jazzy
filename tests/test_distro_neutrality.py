"""The core must not know which ROS distribution it is running under.

The two long-lived branches exist because the *environment* conflicts: different
message types, a renamed controller topic, different Gazebo plugins, different
vendored C++. None of that reaches the pure-numpy core, so the core should be
the same file on both branches — and it stays that way only if nothing in it
names a distro.
"""

import ast
from pathlib import Path

import pytest


SOURCE = Path(__file__).parents[1] / "src/robot_control"
DISTROS = ("humble", "jazzy", "iron", "rolling")

#: profile.py names the distros once, as the vocabulary its endpoint validator
#: accepts. That is a spelling list, not a branch in the code.
DECLARED = {"profile.py"}


def _modules():
    return sorted(path for path in SOURCE.rglob("*.py") if "__pycache__" not in str(path))


def _string_literals(path):
    """Every string in the module that is not a docstring.

    Prose is where the branch split *should* be explained, so a docstring saying
    "Humble" is documentation rather than a distro-dependent code path. What
    matters is a distro reaching a comparison, a dict key or a message — and
    those are all literals.
    """
    tree = ast.parse(path.read_text())
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            first = node.body[0] if node.body else None
            if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
                if isinstance(first.value.value, str):
                    docstrings.add(id(first.value))
    return [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and id(node) not in docstrings
    ]


def test_there_are_modules_to_check():
    assert len(_modules()) > 5


@pytest.mark.parametrize("name", [p.name for p in _modules()])
def test_no_module_selects_behaviour_by_ros_distro(name):
    if name in DECLARED:
        return
    literals = " ".join(_string_literals(SOURCE / name)).lower()

    found = [distro for distro in DISTROS if distro in literals.split()]

    assert not found, (
        f"{name} names {found} in a string literal. A distro name outside "
        ".rosdistro and the profile's endpoints means code that does one thing "
        "on Humble and another on Jazzy, which is what the branch split is for."
    )


def test_the_profile_names_distros_only_as_a_vocabulary():
    literals = _string_literals(SOURCE / "profile.py")

    for distro in ("humble", "jazzy"):
        assert literals.count(distro) == 1, (
            f"profile.py has {literals.count(distro)} {distro!r} literals; the "
            "only place it belongs is KNOWN_DISTROS"
        )
    assert "KNOWN_DISTROS" in (SOURCE / "profile.py").read_text()


def test_the_declared_distro_matches_this_branch():
    from robot_control.layout import declared_distro, repository_root

    assert declared_distro() == (repository_root() / ".rosdistro").read_text().strip()


def test_the_declared_distro_has_an_endpoint_in_the_profile():
    """What the branch says it is, the profile has to be able to reach."""
    from robot_control.layout import declared_distro
    from robot_control.profile import load_builtin_profile

    profile = load_builtin_profile("openarm_tesollo")

    assert declared_distro() in profile.ros
    assert profile.endpoint().command_rate_hz > 0


def test_an_undeclared_distro_is_refused(tmp_path, monkeypatch):
    from robot_control import layout
    from robot_control.profile import ProfileError, load_builtin_profile

    profile = load_builtin_profile("openarm_tesollo")
    monkeypatch.setattr(layout, "declared_distro", lambda: "noble")

    with pytest.raises(ProfileError, match="noble"):
        profile.endpoint()
