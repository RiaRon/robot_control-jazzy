"""Where this checkout is, and which ROS distribution it declares.

Two long-lived branches target two distributions, because the environments
genuinely conflict: message types that exist on one and not the other, a
controller state topic that was renamed, different Gazebo plugins, different
vendored C++. What does *not* conflict is the pure-numpy core, which is the same
code either way.

Keeping that true needs the distro to be a value read from one place rather than
a word written into the code. This module is that place.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path


class LayoutError(RuntimeError):
    pass


def repository_root() -> Path:
    """Return the branch checkout this package was imported from."""
    return Path(__file__).resolve().parents[2]


@lru_cache(maxsize=1)
def declared_distro() -> str:
    """The ROS distribution this branch targets, from ``.rosdistro``.

    Read from the file rather than from ``ROS_DISTRO`` in the environment. The
    file is what the branch *is*; the environment is what happens to be sourced,
    and sourcing the wrong one is exactly the mistake worth catching rather than
    following.
    """
    path = repository_root() / ".rosdistro"
    try:
        distro = path.read_text().strip()
    except OSError as error:
        raise LayoutError(
            f"no .rosdistro at {path}; every branch declares the distribution "
            "it targets"
        ) from error
    if not distro:
        raise LayoutError(f"{path} is empty")
    return distro
