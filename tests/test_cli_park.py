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
    READY_D_LEGACY_NAME,
    READY_D_TARGET_RAD,
    READY_ACCELERATION_RAD_S2,
    READY_POSTURE_NAME,
    READY_SPEED_RAD_S,
    READY_TARGET_RAD,
)
from robot_control.ros_adapter import AdapterUnavailable, ControllerInfo, ControllerTracking, Pose

ARM_JOINTS = 7


@pytest.fixture
def no_ros(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)


class ParkableArm:
    def __init__(self, state):
        self.joints = np.asarray(state, dtype=float)
        self.reference = self.joints.copy()
        self.trajectories = []
        self.streamed = []
        self.efforts = []
        self.events = []
        self.controller_error = None
        self.read_count = 0
        self.read_fail_after = None
        self.follow_gain = 1.0
        self.droop = np.zeros(ARM_JOINTS)

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def require_position_effort_controllers_active(self, timeout_sec=None):
        self.events.append("controllers_checked")
        if self.controller_error is not None:
            raise self.controller_error
        names = [f"openarm_right_joint{i}" for i in range(1, 8)]
        return (
            ControllerInfo(
                "right_joint_trajectory_controller",
                "active",
                tuple(f"{name}/position" for name in names),
            ),
            ControllerInfo(
                "right_forward_effort_controller",
                "active",
                tuple(f"{name}/effort" for name in names),
            ),
        )

    def read_state(self, timeout_sec=None):
        self.read_count += 1
        if self.read_fail_after is not None and self.read_count >= self.read_fail_after:
            raise AdapterUnavailable("simulated joint-state loss")
        compensated = np.minimum(np.abs(self.efforts[-1]), self.droop) if self.efforts else 0.0
        equilibrium = self.reference - self.droop + compensated
        self.joints += self.follow_gain * (equilibrium - self.joints)
        return self.joints.copy()

    def read_pose(self, timeout_sec=None):
        return Pose(tuple(self.joints[:3]), (0.0, 0.0, 0.0, 1.0), "world")

    def read_controller_tracking(self, timeout_sec=None):
        return ControllerTracking(
            self.reference.copy(),
            self.joints.copy(),
            self.reference - self.joints,
        )

    def send_effort(self, effort):
        self.events.append("effort")
        self.efforts.append(np.asarray(effort, dtype=float).copy())

    def stream_positions(self, point):
        self.events.append("position")
        self.reference = np.asarray(point, dtype=float).copy()
        self.streamed.append(self.reference.copy())

    def send_trajectory(self, points, period_sec):
        self.trajectories.append(
            [np.asarray(point, dtype=float).copy() for point in points]
        )
        self.joints = np.asarray(points[-1], dtype=float).copy()
        self.reference = self.joints.copy()
        self.period_sec = period_sec


@pytest.fixture
def arm(monkeypatch):
    from robot_control import cli, ros_adapter

    stub = ParkableArm(np.zeros(ARM_JOINTS))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    monkeypatch.setattr(
        cli,
        "_gravity_chain",
        lambda *a, **k: type(
            "FakeGravityChain",
            (),
            {"gravity_torque": lambda self, q: np.asarray(q, dtype=float) + 0.2},
        )(),
    )
    monkeypatch.setattr(cli, "_ready_sleep", lambda _seconds: None)
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
    points = np.vstack([np.zeros(ARM_JOINTS), arm.streamed])
    np.testing.assert_allclose(points[-1], READY_TARGET_RAD)
    first_non_elbow = np.flatnonzero(
        np.any(np.abs(points[:, [0, 1, 2, 4, 5, 6]]) > 1e-12, axis=1)
    )[0]
    assert points[first_non_elbow - 1, ELBOW_INDEX] == pytest.approx(
        READY_TARGET_RAD[ELBOW_INDEX]
    )
    period = 0.01
    velocity = np.diff(points, axis=0) / period
    acceleration = np.diff(velocity, axis=0) / period
    assert np.max(np.abs(velocity)) <= READY_SPEED_RAD_S + 1e-4
    assert np.max(np.abs(acceleration)) <= READY_ACCELERATION_RAD_S2 + 1e-3
    assert arm.events.index("controllers_checked") < arm.events.index("position")
    assert any(np.any(np.abs(effort) > 0) for effort in arm.efforts[:-3])
    assert all(np.allclose(effort, 0.0) for effort in arm.efforts[-3:])


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
    arm.reference = arm.joints.copy()
    assert main(["pose", "rest", "--group", "openarm_right_arm", "--execute"]) == 0
    settle, lower = arm.trajectories
    assert all(point[ELBOW_INDEX] == pytest.approx(READY_ELBOW_RAD) for point in settle)
    expected = np.zeros(ARM_JOINTS)
    expected[WRIST_INDEX] = READY_WRIST_RAD
    np.testing.assert_allclose(lower[-1], expected)


def test_ready_accepts_small_seed_droop_but_refuses_gross_limit_error(arm, capsys):
    arm.joints[ELBOW_INDEX] = -0.003
    arm.reference = arm.joints.copy()
    assert main(["pose", "ready", "--group", "openarm_right_arm", "--execute"]) == 0
    arm.trajectories.clear()
    arm.joints[ELBOW_INDEX] = -(SEED_SLACK_RAD * 4)
    arm.reference = arm.joints.copy()
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

def test_ready_refuses_inactive_effort_controller_before_position_publish(
    arm, tmp_path, capsys
):
    arm.controller_error = AdapterUnavailable(
        "required controller(s) are not active: right_forward_effort_controller"
    )
    after = tmp_path / "inactive.json"
    code = main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--after-output", str(after), "--execute",
    ])
    assert code == 2
    assert arm.streamed == []
    assert after.exists()
    payload = json.loads(after.read_text())
    assert payload["ready_result"]["is_partial"]
    assert payload["ready_result"]["safe_hold"]["attempted"] is False
    assert "not active" in capsys.readouterr().out


def test_ready_delayed_gravity_droop_plant_converges_with_dynamic_effort(
    arm, monkeypatch, tmp_path
):
    arm.follow_gain = 0.25
    arm.droop[[1, 3]] = [0.12, 0.14]
    monkeypatch.setattr(
        "robot_control.cli._minimum_jerk_trajectory",
        lambda start, target, rate: (
            [
                np.asarray(start) + (np.asarray(target) - np.asarray(start)) * phase
                for phase in np.linspace(1.0 / 40.0, 1.0, 40)
            ],
            0.04,
        ),
    )
    after = tmp_path / "gravity.json"
    assert main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--after-output", str(after), "--execute",
    ]) == 0
    payload = json.loads(after.read_text())["ready_result"]
    assert payload["gravity_enabled"]
    assert payload["gravity_scale"] == pytest.approx(1.0)
    assert payload["gravity_samples"] > 0
    assert payload["passed"]
    assert len({tuple(np.round(effort, 6)) for effort in arm.efforts[:-3]}) > 1


def test_ready_settle_timeout_holds_measured_pose_and_writes_partial_json(
    arm, monkeypatch, tmp_path, capsys
):
    arm.follow_gain = 0.0
    monkeypatch.setattr("robot_control.cli.READY_SETTLE_TIMEOUT_SEC", 0.001)
    monkeypatch.setattr(
        "robot_control.cli._minimum_jerk_trajectory",
        lambda start, target, rate: (
            [
                np.asarray(start) + (np.asarray(target) - np.asarray(start)) * phase
                for phase in np.linspace(1.0 / 40.0, 1.0, 40)
            ],
            0.4,
        ),
    )
    after = tmp_path / "partial.json"
    code = main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--after-output", str(after), "--execute",
    ])
    assert code == 3
    payload = json.loads(after.read_text())["ready_result"]
    assert payload["termination"] == "settle_timeout"
    assert payload["is_partial"]
    assert payload["reference_rad"] == pytest.approx(READY_TARGET_RAD.tolist())
    assert payload["safe_hold"]["applied"]
    assert payload["safe_hold"]["reference_rad"] == pytest.approx(
        payload["feedback_rad"]
    )
    assert payload["gravity_cleanup"]["zero_published"]
    assert all(np.allclose(effort, 0.0) for effort in arm.efforts[-3:])
    assert "did not settle" in capsys.readouterr().out


def test_ready_aprime_v2_is_default_standard_and_d_v1_is_explicit_legacy(arm, capsys):
    assert main(["pose", "ready", "--group", "openarm_right_arm"]) == 0
    assert "standard deterministic A-prime posture" in capsys.readouterr().out
    assert main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--posture", READY_D_LEGACY_NAME,
    ]) == 0
    output = capsys.readouterr().out
    assert "legacy D comparison posture" in output
    assert str(READY_D_TARGET_RAD[1]) in output

def test_ready_joint_state_exception_cleans_effort_and_writes_partial(
    arm, monkeypatch, tmp_path, capsys
):
    arm.read_fail_after = 4
    monkeypatch.setattr(
        "robot_control.cli._minimum_jerk_trajectory",
        lambda start, target, rate: (
            [
                np.asarray(start) + (np.asarray(target) - np.asarray(start)) * phase
                for phase in np.linspace(1.0 / 40.0, 1.0, 40)
            ],
            0.4,
        ),
    )
    after = tmp_path / "joint-state-partial.json"
    code = main([
        "pose", "ready", "--group", "openarm_right_arm",
        "--after-output", str(after), "--execute",
    ])
    assert code == 2
    payload = json.loads(after.read_text())["ready_result"]
    assert payload["termination"] == "exception"
    assert payload["is_partial"]
    assert payload["safe_hold"]["attempted"]
    assert payload["gravity_cleanup"]["zero_published"]
    assert all(np.allclose(effort, 0.0) for effort in arm.efforts[-3:])
    assert "joint-state loss" in capsys.readouterr().out
