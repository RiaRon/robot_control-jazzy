"""Read named poses out of a MoveIt SRDF.

An SRDF ``group_state`` is the robot's own vocabulary for a pose: ``home``,
``hands_up``, ``open``. Reading it here rather than asking ``move_group`` keeps
``robotctl pose joints --named`` usable with no ROS running, which is what
makes a dry run genuinely offline.

The values are keyed by source joint name, which is exactly what
:meth:`~robot_control.interface.CanonicalInterface.group_state_to_canonical`
consumes. Nothing here checks limits; the SRDF is a vendor file and the profile
is the authority.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ElementTree

from .layout import repository_root

# Where the bimanual SRDF lives, most-built first: the installed share
# directory is what a sourced workspace actually uses.
#
# The `openarm_v1.0` entries are not an older layout to be cleaned up: the
# vendored MoveIt configuration keeps its SRDF under a hardware-version
# directory on some upstream branches and directly under `config/` on others,
# and the two branches of this repository vendor different ones. A candidate
# list is what that difference costs, rather than a distro check.
_SRDF_CANDIDATES = (
    "ros_ws/install/openarm_bimanual_moveit_config/share/"
    "openarm_bimanual_moveit_config/config/openarm_bimanual.srdf",
    "ros_ws/install/openarm_bimanual_moveit_config/share/"
    "openarm_bimanual_moveit_config/config/openarm_v1.0/openarm_bimanual.srdf",
    "ros_ws/src/openarm_ros2/openarm_bimanual_moveit_config/config/"
    "openarm_bimanual.srdf",
    "ros_ws/src/openarm_ros2/openarm_bimanual_moveit_config/config/"
    "openarm_v1.0/openarm_bimanual.srdf",
)


class SrdfError(ValueError):
    pass


#: Re-exported: callers have always reached the checkout through this module,
#: and where it lives is layout's business rather than the SRDF reader's.
repository_root = repository_root


def find_srdf(root: Path | None = None) -> Path:
    root = repository_root() if root is None else Path(root)
    for candidate in _SRDF_CANDIDATES:
        path = root / candidate
        if path.is_file():
            return path
    raise SrdfError(
        f"no SRDF found under {root}; looked for {', '.join(_SRDF_CANDIDATES)}. "
        "Build the workspace with ros_ws/build.sh."
    )


def named_states(path: Path) -> dict[tuple[str, str], dict[str, float]]:
    """Map (planning group, state name) to source joint values."""
    try:
        root = ElementTree.parse(path).getroot()
    except ElementTree.ParseError as error:
        raise SrdfError(f"{path} is not readable as XML: {error}") from error

    states: dict[tuple[str, str], dict[str, float]] = {}
    for state in root.findall("group_state"):
        group, name = state.get("group"), state.get("name")
        if group is None or name is None:
            continue
        values = {}
        for joint in state.findall("joint"):
            joint_name, value = joint.get("name"), joint.get("value")
            if joint_name is None or value is None:
                continue
            values[joint_name] = float(value)
        states[(group, name)] = values
    return states


def named_state(
    moveit_group: str, name: str, path: Path | None = None
) -> dict[str, float]:
    """Return one named state's source joint values, or explain what exists."""
    path = find_srdf() if path is None else path
    states = named_states(path)
    if (moveit_group, name) in states:
        return states[(moveit_group, name)]
    available = sorted(
        state for group, state in states if group == moveit_group
    )
    if not available:
        raise SrdfError(
            f"{path.name} declares no named states for planning group "
            f"{moveit_group!r}"
        )
    raise SrdfError(
        f"{path.name} has no state {name!r} for planning group "
        f"{moveit_group!r}; it declares {', '.join(available)}"
    )
