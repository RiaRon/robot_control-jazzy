"""Standard right-arm ready posture and legacy table-rest safety."""

import json
import sys

import numpy as np
import pytest

from robot_control.cli import (
    ELBOW_INDEX,
    READY_ELBOW_RAD,
    READY_WRIST_RAD,
    SEED_SLACK_RAD,
    WRIST_INDEX,
    main,
)
from robot_control.ready import (
    READY_ACCELERATION_RAD_S2,
    READY_POSTURE_NAME,
    READY_SPEED_RAD_S,
    READY_TARGET_RAD,
)
from robot_control.ros_adapter import Pose

ARM_JOINTS = 7


@pytest.fixture
def no_ros(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)


class ParkableArm:
    def __init__(self, state):
        self.joints = np.asarray(state, dtype=float)
        self.trajectories = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def read_pose(self, timeout_sec=None):
        return Pose(tuple(self.joints[:3]), (0.0, 0.0, 0.0, 1.0), "world")

    def send_trajectory(self, points, period_sec):
        self.trajectories.append(
            [np.asarray(point, dtype=float).copy() for point in points]
        )
        self.joints = np.asarray(points[-1], dtype=float).copy()
        self.period_sec = period_sec


@pytest.fixture
def arm(monkeypatch):
    from robot_control import cli, ros_adapter

    stub = ParkableArm(np.zeros(ARM_JOINTS))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    monkeypatch.setattr(cli, "READY_SETTLE_WINDOW_SEC", 0.0)
    monkeypatch.setattr(cli, "READY_SETTLE_TIMEOUT_SEC", 0.1)
    return stub


def test_ready_dry_run_is_right_only_and_offline(no_ros, capsys):
    assert main(["pose", "ready", "--group", "openarm_right_arm"]) == 0
    output = capsys.readouterr().out
    assert READY_POSTURE_NAME in output
    assert "right arm only" in output
    assert "openarm_left_arm" not in output
    assert "no ROS connection was opened" in output


def test_ready_requires_explicit_group():
    with pytest.raises(SystemExit):
        main(["pose", "ready"])


def test_ready_rejects_left_arm_before_ros(no_ros, capsys):
    assert main(["pose", "ready", "--group", "openarm_left_arm"]) == 2
    assert "left arm is not activated" in capsys.readouterr().out


def test_ready_lifts_elbow_then_reaches_selected_target_with_bounded_motion(arm):
    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 0
    lift, settle = arm.trajectories
    np.testing.assert_allclose(settle[-1], READY_TARGET_RAD)
    for trajectory in (lift, settle):
        points = np.asarray(trajectory)
        velocity = np.diff(points, axis=0) / arm.period_sec
        acceleration = np.diff(velocity, axis=0) / arm.period_sec
        assert np.max(np.abs(velocity)) <= READY_SPEED_RAD_S + 1e-4
        assert np.max(np.abs(acceleration)) <= READY_ACCELERATION_RAD_S2 + 1e-3


def test_ready_writes_before_and_after_snapshots(arm, tmp_path):
    before = tmp_path / "before.json"
    after = tmp_path / "after.json"
    assert main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--before-output", str(before), "--after-output", str(after), "--execute",
    ]) == 0
    before_payload = json.loads(before.read_text())
    after_payload = json.loads(after.read_text())
    assert before_payload["kind"] == "pose_snapshot"
    assert after_payload["ready_posture"]["name"] == READY_POSTURE_NAME
    assert after_payload["ready_posture"]["passed"]
    assert after_payload["ready_result"]["max_abs_joint_error_rad"] == pytest.approx(0.0)


def test_rest_keeps_wrist_lifted_while_elbow_comes_down(arm):
    arm.joints = np.array([0.0, 0.0, 0.0, READY_ELBOW_RAD, 0.0, 0.0, 0.3])
    assert main(["pose", "rest", "--group", "openarm_right_arm", "--execute"]) == 0
    settle, lower = arm.trajectories
    assert all(point[ELBOW_INDEX] == pytest.approx(READY_ELBOW_RAD) for point in settle)
    expected = np.zeros(ARM_JOINTS)
    expected[WRIST_INDEX] = READY_WRIST_RAD
    np.testing.assert_allclose(lower[-1], expected)


def test_ready_accepts_small_seed_droop_but_refuses_gross_limit_error(arm, capsys):
    arm.joints[ELBOW_INDEX] = -0.003
    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 0
    arm.trajectories.clear()
    arm.joints[ELBOW_INDEX] = -(SEED_SLACK_RAD * 4)
    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 3
    assert "r_aj_4" in capsys.readouterr().out


def test_pose_joints_still_starts_from_clamped_seed(arm):
    arm.joints[ELBOW_INDEX] = -0.003
    code = main([
        "pose", "joints", "--group", "openarm_right_arm",
        "--values", "0,0,0,0.8,0,0,0", "--execute",
    ])
    assert code == 0
    assert arm.trajectories[-1][-1][ELBOW_INDEX] == pytest.approx(0.8)
