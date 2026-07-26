import sys

import pytest

from robot_control.cli import main

RIGHT_ARM = ["--group", "openarm_right_arm"]


@pytest.fixture
def no_ros(monkeypatch):
    """Make every rclpy import fail, as it does with no ROS sourced."""
    monkeypatch.setitem(sys.modules, "rclpy", None)


def test_preflight_reports_profile(capsys):
    assert main(["r2s", "preflight", "--profile", "openarm_tesollo"]) == 0
    output = capsys.readouterr().out
    assert "openarm_tesollo" in output
    assert "publish_enabled: false" in output


def test_collect_defaults_to_dry_run(capsys):
    assert main(["r2s", "collect", "--profile", "openarm_tesollo"]) == 0
    assert "DRY RUN" in capsys.readouterr().out


def test_pose_joints_dry_run_prints_the_target_without_touching_ros(no_ros, capsys):
    """A dry run must resolve entirely offline, so it cannot reach the robot."""
    values = ",".join(["0.1"] * 7)

    assert main(["pose", "joints", *RIGHT_ARM, "--values", values]) == 0

    output = capsys.readouterr().out
    assert "DRY RUN" in output
    assert "right_joint_trajectory_controller" in output
    assert "openarm_right_joint1" in output
    assert "0.1" in output


def test_pose_joints_execute_without_an_adapter_exits_2(no_ros, capsys):
    values = ",".join(["0.1"] * 7)

    assert main(["pose", "joints", *RIGHT_ARM, "--values", values, "--execute"]) == 2

    assert "rclpy" in capsys.readouterr().out


def test_pose_rejects_an_unknown_group(capsys):
    assert main(["pose", "joints", "--group", "openarm_third_arm", "--values", "0"]) == 2
    assert "unknown group" in capsys.readouterr().out


def test_pose_joints_rejects_a_value_count_mismatch(capsys):
    assert main(["pose", "joints", *RIGHT_ARM, "--values", "0.1,0.2"]) == 2

    output = capsys.readouterr().out
    assert "7" in output and "2" in output


def test_pose_joints_rejects_a_non_numeric_value(capsys):
    assert main(["pose", "joints", *RIGHT_ARM, "--values", "0.1,x,0,0,0,0,0"]) == 2
    assert "x" in capsys.readouterr().out


def test_pose_joints_requires_values_or_a_named_state(capsys):
    with pytest.raises(SystemExit) as exit_info:
        main(["pose", "joints", *RIGHT_ARM])
    assert exit_info.value.code == 2


def test_pose_joints_resolves_a_named_state_from_the_srdf(capsys):
    assert main(["pose", "joints", *RIGHT_ARM, "--named", "hands_up"]) == 0

    output = capsys.readouterr().out
    assert "DRY RUN" in output
    # hands_up sets joint4 to 2 rad and every other arm joint to zero.
    assert "2.0" in output


def test_pose_joints_rejects_a_named_state_absent_from_the_srdf(capsys):
    assert main(["pose", "joints", *RIGHT_ARM, "--named", "tea_pot"]) == 2

    output = capsys.readouterr().out
    assert "tea_pot" in output
    assert "hands_up" in output  # the error lists what is available


def test_pose_joints_refuses_a_target_outside_the_profile_limits(capsys):
    """Profile limits are authoritative, and a dry run still enforces them."""
    assert main(["pose", "joints", *RIGHT_ARM, "--values", "99,0,0,0,0,0,0"]) == 3
    assert "position limit" in capsys.readouterr().out


def test_pose_joints_named_state_can_violate_the_profile_limits(capsys):
    """The SRDF opens the gripper to 0.044; the profile stops at 0.04."""
    code = main(["pose", "joints", "--group", "openarm_left_gripper", "--named", "open"])

    assert code == 3
    assert "position limit" in capsys.readouterr().out


def test_pose_ee_requires_a_target():
    with pytest.raises(SystemExit) as exit_info:
        main(["pose", "ee", *RIGHT_ARM])
    assert exit_info.value.code == 2


def test_pose_ee_rejects_both_a_typed_and_a_dragged_target():
    """--from-marker is a whole pose, so it cannot be combined with --xyz."""
    with pytest.raises(SystemExit) as exit_info:
        main(["pose", "ee", *RIGHT_ARM, "--xyz", "0,0,0.03", "--from-marker"])
    assert exit_info.value.code == 2


def test_pose_ee_from_marker_rejects_relative(no_ros, capsys):
    """The marker already carries an absolute pose; an offset from it is a
    request for something the operator cannot see on screen."""
    assert main(["pose", "ee", *RIGHT_ARM, "--from-marker", "--relative"]) == 2
    assert "--relative" in capsys.readouterr().out


def test_pose_ee_from_marker_rejects_rpy(no_ros, capsys):
    assert main(["pose", "ee", *RIGHT_ARM, "--from-marker", "--rpy", "0,0,0"]) == 2
    assert "--rpy" in capsys.readouterr().out


def test_pose_ee_from_marker_needs_ros(no_ros, capsys):
    assert main(["pose", "ee", *RIGHT_ARM, "--from-marker"]) == 2
    assert "rclpy" in capsys.readouterr().out


def test_pose_ee_needs_ros_even_for_a_dry_run(no_ros, capsys):
    """IK lives in move_group, so there is no offline form of pose ee."""
    assert main(["pose", "ee", *RIGHT_ARM, "--xyz", "0,0,0.03", "--relative"]) == 2
    assert "rclpy" in capsys.readouterr().out


def test_pose_ee_rejects_a_group_without_a_planning_group(no_ros, capsys):
    code = main(["pose", "ee", "--group", "tesollo_curl", "--xyz", "0,0,0.03"])

    assert code == 2
    # The error points at the command that does work for this group.
    assert "pose joints --values" in capsys.readouterr().out


def test_pose_show_reports_every_executable_group_offline(capsys):
    """Without ROS, show still reports the contract it would read against."""
    assert main(["pose", "show"]) == 2

    output = capsys.readouterr().out
    assert "openarm_right_arm" in output
    assert "right_joint_trajectory_controller" in output
