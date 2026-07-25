import math
import os
from pathlib import Path
import subprocess

import pytest

from tools.ros_smoke import smoke_plan, validate_joint_state, wait_for


ROOT = Path(__file__).parents[1]
DG5F_JOINTS = (
    "rj_dg_1_1",
    "rj_dg_1_2",
    "rj_dg_1_3",
    "rj_dg_1_4",
    "rj_dg_2_1",
    "rj_dg_2_2",
    "rj_dg_2_3",
    "rj_dg_2_4",
    "rj_dg_3_1",
    "rj_dg_3_2",
    "rj_dg_3_3",
    "rj_dg_3_4",
    "rj_dg_4_1",
    "rj_dg_4_2",
    "rj_dg_4_3",
    "rj_dg_4_4",
    "rj_dg_5_1",
    "rj_dg_5_2",
    "rj_dg_5_3",
    "rj_dg_5_4",
)
DG5F_TARGET = dict.fromkeys(DG5F_JOINTS, 0.05)
SMOKE_SCRIPTS = (
    ROOT / "ros_ws/smoke_openarm_fake.sh",
    ROOT / "ros_ws/smoke_dg5f_fake.sh",
    ROOT / "ros_ws/smoke_dg5f_gazebo.sh",
)


def test_wait_for_returns_after_predicate_succeeds():
    attempts = 0

    def succeeds_on_third_attempt() -> bool:
        nonlocal attempts
        attempts += 1
        return attempts == 3

    wait_for(succeeds_on_third_attempt, timeout_s=0.1, interval_s=0)

    assert attempts == 3


def test_wait_for_raises_at_deadline():
    with pytest.raises(TimeoutError):
        wait_for(lambda: False, timeout_s=0, interval_s=0)


def test_validate_joint_state_accepts_exact_finite_target():
    validate_joint_state(
        DG5F_TARGET,
        DG5F_JOINTS,
        [0.05] * 20,
        tolerance=0.01,
    )


def test_validate_joint_state_rejects_missing_joint():
    with pytest.raises(ValueError, match="missing"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS[:-1],
            [0.05] * 19,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_extra_joint():
    with pytest.raises(ValueError, match="extra"):
        validate_joint_state(
            DG5F_TARGET,
            (*DG5F_JOINTS, "unexpected_joint"),
            [0.05] * 21,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_name_position_length_mismatch():
    with pytest.raises(ValueError, match="length"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            [0.05] * 19,
            tolerance=0.01,
        )


@pytest.mark.parametrize("bad_position", [math.nan, math.inf, -math.inf])
def test_validate_joint_state_rejects_non_finite_position(bad_position):
    positions = [0.05] * 20
    positions[7] = bad_position

    with pytest.raises(ValueError, match="non-finite"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            positions,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_position_error_above_tolerance():
    positions = [0.05] * 20
    positions[12] = 0.071

    with pytest.raises(ValueError, match="position error"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            positions,
            tolerance=0.02,
        )


def test_validate_joint_state_rejects_duplicate_joint_name():
    duplicate_names = (*DG5F_JOINTS[:-1], DG5F_JOINTS[0])

    with pytest.raises(ValueError, match="duplicate"):
        validate_joint_state(
            DG5F_TARGET,
            duplicate_names,
            [0.05] * 20,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_non_finite_target():
    targets = dict(DG5F_TARGET)
    targets[DG5F_JOINTS[0]] = math.nan

    with pytest.raises(ValueError, match="target"):
        validate_joint_state(
            targets,
            DG5F_JOINTS,
            [0.05] * 20,
            tolerance=0.01,
        )


@pytest.mark.parametrize("tolerance", [-0.01, math.nan, math.inf])
def test_validate_joint_state_rejects_invalid_tolerance(tolerance):
    with pytest.raises(ValueError, match="tolerance"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            [0.05] * 20,
            tolerance=tolerance,
        )


@pytest.mark.parametrize("robot", ["openarm", "dg5f"])
def test_smoke_plan_never_commands_more_than_point_zero_five_radians(robot):
    plan = smoke_plan(robot)

    assert plan.command_targets
    assert max(abs(position) for position in plan.command_targets.values()) <= 0.05
    assert plan.command_targets.keys() <= plan.state_targets.keys()


@pytest.mark.parametrize("script", SMOKE_SCRIPTS)
def test_smoke_script_rejects_non_jazzy_before_starting_ros(script):
    result = subprocess.run(
        ["bash", str(script)],
        env=dict(os.environ, ROS_DISTRO="humble"),
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "ROS 2 Jazzy" in result.stderr
