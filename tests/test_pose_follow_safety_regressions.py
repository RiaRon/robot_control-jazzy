"""Safety regressions from the aborted 2026-08-20 real follow run."""

from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.ready import (
    FOLLOW_READY_SAFE_NEIGHBORHOOD_RAD,
    READY_POSTURE_NAME,
    READY_TARGET_RAD,
    READY_TOLERANCE_RAD,
    check_ready,
)


RIGHT_ARM = ["--group", "openarm_right_arm"]


# Read from /home/cbj4/Downloads/right-pose-before.json on 2026-08-20.
# The raw incident JSON is intentionally not a repository fixture.
INCIDENT_INITIAL_JOINTS_RAD = np.array(
    [
        -0.002861066605630569,
        0.005149919890135024,
        0.008964675364309116,
        -0.005531395437552433,
        -0.02613107499809253,
        -0.02155336842908362,
        -0.0005722133211261138,
    ]
)

# Exact accepted-target delta printed by the 0673903 real translation refusal.
RETEST_BRANCH_JUMP_RAD = np.array(
    [3.1559, 3.1269, 1.5527, 0.0, 1.5889, 0.0, 0.0]
)

# Exact seven arm joints from right-pose-before-retest.json in the recovered
# 2026-08-20 archive. The raw experiment JSON remains outside Git.
RETEST_INITIAL_JOINTS_RAD = np.array(
    [
        -0.0165941863126573,
        0.004768444342717615,
        -0.011253528648813571,
        0.02155336842908362,
        -0.02536812390325771,
        -0.018883039597161755,
        -0.0005722133211261138,
    ]
)


class CartesianReplayChain:
    """Small FK model that makes replayed joint changes observable as a pose."""

    links = ()

    def pose(self, joints):
        from robot_control.kinematics import _rotation

        joints = np.asarray(joints, dtype=float)
        pose = np.eye(4)
        pose[:3, 3] = joints[:3]
        pose[:3, :3] = (
            _rotation(np.array([1.0, 0.0, 0.0]), joints[3])
            @ _rotation(np.array([0.0, 1.0, 0.0]), joints[4])
            @ _rotation(np.array([0.0, 0.0, 1.0]), joints[5])
        )
        return pose

    def gravity_torque(self, _joints):
        return np.zeros(7)


def pose_from_joints(chain, joints):
    from robot_control.cli import _quaternion_from_rotation
    from robot_control.ros_adapter import Pose

    matrix = chain.pose(joints)
    return Pose(
        tuple(matrix[:3, 3]),
        _quaternion_from_rotation(matrix[:3, :3]),
        "world",
    )


class ReplayArm:
    """Read an initial pose and return a controlled IK branch offset."""

    def __init__(self, initial_joints, ik_offset=None):
        self.efforts = []
        self.joints = np.asarray(initial_joints, dtype=float).copy()
        self.ik_offset = np.zeros(7) if ik_offset is None else np.asarray(
            ik_offset, dtype=float
        )
        self.chain = CartesianReplayChain()
        self.target = pose_from_joints(self.chain, self.joints)
        self.streamed = []
        self.ik_requests = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def watch_marker(self):
        return None

    def require_position_effort_controllers_active(self, timeout_sec=None):
        return (
            SimpleNamespace(name="right_joint_trajectory_controller"),
            SimpleNamespace(name="right_forward_effort_controller"),
        )

    def send_effort(self, effort):
        self.efforts.append(np.asarray(effort, dtype=float).copy())

    def read_marker_pose(self, timeout_sec=None):
        return self.target

    def latest_marker_target(self):
        return self.target

    def pump(self, timeout_sec=0.0):
        return None

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def solve_ik(self, pose, seed, timeout_sec=None):
        self.ik_requests.append((pose, np.asarray(seed, dtype=float).copy()))
        return np.asarray(seed, dtype=float) + self.ik_offset

    def stream_positions(self, positions):
        command = np.asarray(positions, dtype=float).copy()
        self.streamed.append(command)
        self.joints = command


class SequenceSixBranchReplayArm(ReplayArm):
    """Return five continuous targets, then only the reported bad branch."""

    def __init__(self):
        super().__init__(READY_TARGET_RAD)
        self.solve_calls = 0

    def solve_ik(self, pose, seed, timeout_sec=None):
        seed = np.asarray(seed, dtype=float).copy()
        self.ik_requests.append((pose, seed.copy()))
        self.solve_calls += 1
        if self.solve_calls <= 20:
            return seed
        return seed + RETEST_BRANCH_JUMP_RAD


def install_replay(monkeypatch, arm):
    from robot_control import ros_adapter

    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *args, **kwargs: arm)
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain", lambda *args, **kwargs: arm.chain
    )


def diagnostic_args(*extra):
    return [
        "pose",
        "follow",
        *RIGHT_ARM,
        "--diagnostic-profile",
        "translation",
        "--diagnostic-distance",
        "0.001",
        "--diagnostic-linear-speed",
        "0.02",
        "--diagnostic-hold-sec",
        "0.01",
        "--startup-settle-sec",
        "0",
        "--seconds",
        "0.5",
        *extra,
    ]


def test_pose_follow_dry_run_never_opens_a_ros_adapter(monkeypatch, capsys):
    from robot_control import ros_adapter

    def forbid_adapter(*_args, **_kwargs):
        raise AssertionError("dry-run opened ROS and could reach a publisher")

    monkeypatch.setattr(ros_adapter, "RosAdapter", forbid_adapter)
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("dry-run read the live robot description")
        ),
    )

    assert main(diagnostic_args()) == 0
    output = capsys.readouterr().out
    assert "no ROS connection is opened" in output
    assert "nothing is published, including startup alignment" in output


def test_unwritable_output_is_refused_before_adapter_creation(
    monkeypatch, tmp_path, capsys
):
    from robot_control import ros_adapter

    parent_is_a_file = tmp_path / "not-a-directory"
    parent_is_a_file.write_text("do not replace\n")

    def forbid_adapter(*_args, **_kwargs):
        raise AssertionError("control adapter opened before output preflight")

    monkeypatch.setattr(ros_adapter, "RosAdapter", forbid_adapter)
    code = main(
        diagnostic_args(
            "--output", str(parent_is_a_file / "run.json"), "--execute"
        )
    )

    assert code == 2
    assert parent_is_a_file.read_text() == "do not replace\n"
    assert "output is not writable before control starts" in capsys.readouterr().out


def test_output_permission_failure_is_refused_before_adapter_creation(
    monkeypatch, tmp_path, capsys
):
    from robot_control import ros_adapter

    def forbid_adapter(*_args, **_kwargs):
        raise AssertionError("control adapter opened before output preflight")

    def deny_temporary_file(*_args, **_kwargs):
        raise PermissionError("simulated read-only directory")

    monkeypatch.setattr(ros_adapter, "RosAdapter", forbid_adapter)
    monkeypatch.setattr(
        "robot_control.cli.tempfile.NamedTemporaryFile", deny_temporary_file
    )
    code = main(
        diagnostic_args(
            "--output", str(tmp_path / "run.json"), "--execute"
        )
    )

    assert code == 2
    output = capsys.readouterr().out
    assert "output is not writable before control starts" in output
    assert "simulated read-only directory" in output


def test_initial_j3_j5_branch_jump_is_refused_before_first_publish(
    monkeypatch, capsys
):
    # The terminal summary retained magnitudes only. These representative
    # signs make the replay deterministic; fake MoveIt independently produced
    # an approximately -0.75/+0.75 rad J3/J5 branch on this exact initial pose.
    branch_offset = np.zeros(7)
    branch_offset[2] = 0.7646
    branch_offset[4] = -0.7480
    arm = ReplayArm(READY_TARGET_RAD, branch_offset)
    install_replay(monkeypatch, arm)

    assert main(diagnostic_args("--execute")) == 3
    assert arm.ik_requests
    np.testing.assert_array_equal(
        arm.ik_requests[0][1], READY_TARGET_RAD
    )
    assert arm.streamed == []
    output = capsys.readouterr().out
    assert "IK target jump refused before publish" in output
    assert "r_aj_3=+0.7646 rad" in output
    assert "r_aj_5=-0.7480 rad" in output


def test_recovered_incident_pose_is_reacquired_before_ik(
    monkeypatch, tmp_path
):
    arm = ReplayArm(INCIDENT_INITIAL_JOINTS_RAD)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "incident-reacquired.json"

    assert (
        main(
            diagnostic_args(
                "--output",
                str(output),
                "--execute",
            )
        )
        == 0
    )
    assert arm.streamed
    assert arm.ik_requests
    payload = json.loads(output.read_text())
    result = payload["result"]
    assert result["ready_reacquisition"]["required"]
    assert result["ready_reacquisition"]["completed"]
    assert result["handoff_sync"]["completed"]
    np.testing.assert_allclose(
        arm.ik_requests[0][1], result["handoff_sync"]["measured_joints_rad"]
    )
    assert result["convergence_gate"]["completed"]
    assert result["diagnostic_execution"]["position_publish_count"] > 0


def test_ik_target_jump_at_exact_hard_boundary_is_refused(monkeypatch, capsys):
    branch_offset = np.zeros(7)
    branch_offset[0] = 0.30
    arm = ReplayArm(READY_TARGET_RAD, branch_offset)
    install_replay(monkeypatch, arm)

    assert main(diagnostic_args("--execute")) == 3
    assert arm.ik_requests
    assert arm.streamed == []
    output = capsys.readouterr().out
    assert "IK target jump refused before publish" in output
    assert "r_aj_1=+0.3000 rad" in output


def test_deterministic_alignment_message_never_invites_marker_drag(
    monkeypatch, capsys
):
    arm = ReplayArm(READY_TARGET_RAD)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)

    assert main(diagnostic_args("--execute")) == 0
    output = capsys.readouterr().out
    assert "deterministic profile started" in output
    assert "keep the RViz marker at Current" in output
    assert "drag the marker" not in output


def test_sequence_six_branch_jump_writes_partial_json_before_refusal(
    monkeypatch, tmp_path, capsys
):
    arm = SequenceSixBranchReplayArm()
    install_replay(monkeypatch, arm)
    output_path = tmp_path / "partial-refusal.json"

    code = main(
        diagnostic_args(
            "--diagnostic-distance",
            "0.01",
            "--diagnostic-linear-speed",
            "0.005",
            "--tolerance",
            "0.0001",
            "--seconds",
            "1.0",
            "--output",
            str(output_path),
            "--execute",
        )
    )

    assert code == 3
    assert output_path.is_file()
    payload = json.loads(output_path.read_text())
    result = payload["result"]
    refusal = result["refusal"]

    assert result["termination"] == "safety_refused"
    assert result["is_partial"]
    assert result["samples"] == len(payload["trace"])
    assert result["samples"] > 0
    np.testing.assert_allclose(
        payload["trace"][0]["joint_positions_rad"]["measured"],
        READY_TARGET_RAD,
        atol=1e-12,
    )
    assert result["ik"]["submitted"] == 6
    assert result["ik"]["succeeded"] == 5
    assert result["ik"]["continuity_rejected"] == 4
    assert result["ik"]["continuity_retries"] == 3
    assert result["ik"]["continuity_exhausted"] == 1
    assert refusal["reason"] == "ik_continuity_exhausted"
    assert refusal["refused_sequence"] == 6
    assert refusal["profile_phase"] == "translation_ramp_out"
    assert refusal["attempts"] == 4
    assert refusal["triggered_joints"] == [
        "r_aj_1",
        "r_aj_2",
        "r_aj_3",
        "r_aj_5",
    ]
    np.testing.assert_allclose(
        refusal["joint_delta_rad"], RETEST_BRANCH_JUMP_RAD
    )
    assert all(sample["ik_sequence"] != 6 for sample in payload["trace"])
    assert arm.streamed
    for command in arm.streamed:
        np.testing.assert_allclose(command, arm.streamed[0])

    terminal = capsys.readouterr().out
    assert "wrote partial pose follow diagnostics" in terminal
    assert "IK target jump refused before publish after 4" in terminal


def _install_fast_reacquisition(monkeypatch):
    monkeypatch.setattr("robot_control.cli._ready_sleep", lambda _seconds: None)
    monkeypatch.setattr(
        "robot_control.cli.FOLLOW_READY_STATIONARY_DWELL_SEC", 0.0
    )
    monkeypatch.setattr("robot_control.cli.READY_SETTLE_WINDOW_SEC", 0.0)
    monkeypatch.setattr("robot_control.cli.READY_SETTLE_TIMEOUT_SEC", 0.03)
    monkeypatch.setattr(
        "robot_control.follow_observability.HANDOFF_STABLE_WINDOW_SEC", 0.01
    )
    monkeypatch.setattr(
        "robot_control.follow_observability.HANDOFF_TIMEOUT_SEC", 0.08
    )


class HandoffReplayArm(ReplayArm):
    def __init__(self, initial_joints, *, track_positions=True):
        super().__init__(initial_joints)
        self.track_positions = track_positions
        self.events = []

    def require_position_effort_controllers_active(self, timeout_sec=None):
        self.events.append("controllers_checked")
        return super().require_position_effort_controllers_active(timeout_sec)

    def send_effort(self, effort):
        self.events.append("effort")
        super().send_effort(effort)

    def stream_positions(self, positions):
        self.events.append("position")
        command = np.asarray(positions, dtype=float).copy()
        self.streamed.append(command)
        if self.track_positions:
            self.joints = command

    def read_marker_pose(self, timeout_sec=None):
        return pose_from_joints(self.chain, self.joints)

    def latest_marker_target(self):
        return pose_from_joints(self.chain, self.joints)


def test_sagged_start_keeps_gravity_on_through_reacquisition_alignment_and_profile(
    monkeypatch, tmp_path
):
    sagged = np.array(
        [-0.0887, 0.1055, 0.0013, 0.4801, -0.0261, -0.0380, -0.0345]
    )
    arm = HandoffReplayArm(sagged)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "handoff.json"

    assert (
        main(
            diagnostic_args(
                "--output",
                str(output),
                "--execute",
            )
        )
        == 0
    )

    payload = json.loads(output.read_text())
    result = payload["result"]
    gravity = result["gravity_compensation"]
    reacquisition = result["ready_reacquisition"]
    alignment = result["startup_alignment"]
    diagnostic = result["diagnostic_execution"]

    assert gravity["activated"]
    assert gravity["scale"] == [1.0] * 7
    assert gravity["activation_elapsed_sec"] <= reacquisition["started_elapsed_sec"]
    assert reacquisition["required"]
    assert reacquisition["completed"]
    assert reacquisition["decision_passed"]
    assert (
        reacquisition["max_abs_final_error_rad"] <= FOLLOW_READY_SAFE_NEIGHBORHOOD_RAD
    )
    assert reacquisition["motion_samples"] >= 1
    assert alignment["completed"]
    assert diagnostic["started"]
    assert diagnostic["position_publish_count"] > 0
    assert (
        reacquisition["completed_elapsed_sec"]
        <= alignment["started_elapsed_sec"]
        <= alignment["completed_elapsed_sec"]
        <= diagnostic["started_elapsed_sec"]
    )
    cleanup = gravity["cleanup"]
    assert cleanup["zero_published"]
    assert cleanup["started_elapsed_sec"] >= diagnostic["started_elapsed_sec"]
    assert arm.events.index("effort") < arm.events.index("position")
    assert len(arm.efforts) > 3
    for zero in arm.efforts[-3:]:
        np.testing.assert_allclose(zero, np.zeros(7))


def test_reacquisition_failure_holds_safely_writes_partial_and_never_starts_profile(
    monkeypatch, tmp_path
):
    arm = HandoffReplayArm(
        [-0.0887, 0.1055, 0.0013, 0.4801, -0.0261, -0.0380, -0.0345],
        track_positions=False,
    )
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "reacquisition-failed.json"

    assert (
        main(
            diagnostic_args(
                "--output",
                str(output),
                "--execute",
            )
        )
        == 3
    )

    payload = json.loads(output.read_text())
    result = payload["result"]
    reacquisition = result["ready_reacquisition"]
    assert result["termination"] == "ready_reacquisition_failed"
    assert result["is_partial"]
    assert result["ik"]["submitted"] == 0
    assert result["diagnostic_execution"]["position_publish_count"] == 0
    assert not result["startup_alignment"]["started"]
    assert reacquisition["safe_hold"]["attempted"]
    assert reacquisition["safe_hold"]["applied"]
    assert result["gravity_compensation"]["cleanup"]["zero_published"]
    assert arm.ik_requests == []


def test_stale_rviz_marker_is_reanchored_to_measured_tcp(
    monkeypatch, tmp_path
):
    arm = HandoffReplayArm(READY_TARGET_RAD)
    arm.target = pose_from_joints(arm.chain, READY_TARGET_RAD)
    arm.target = type(arm.target)(
        (arm.target.position[0] + 1.0, *arm.target.position[1:]),
        arm.target.orientation,
        arm.target.frame_id,
    )
    arm.read_marker_pose = lambda timeout_sec=None: arm.target
    arm.latest_marker_target = lambda: arm.target
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "alignment-failed.json"

    assert (
        main(
            diagnostic_args(
                "--output",
                str(output),
                "--execute",
            )
        )
        == 0
    )

    payload = json.loads(output.read_text())
    result = payload["result"]
    assert result["handoff_sync"]["marker_reanchored"]
    assert result["startup_alignment"]["completed"]
    assert result["convergence_gate"]["completed"]
    assert result["diagnostic_execution"]["position_publish_count"] > 0
    first = payload["trace"][0]
    np.testing.assert_allclose(
        first["tcp_positions_m"]["live_marker"],
        result["handoff_sync"]["measured_tcp"]["xyz_m"],
    )

def test_deterministic_follow_rejects_unvalidated_gravity_scale_before_ros(
    monkeypatch, capsys
):
    from robot_control import ros_adapter

    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("invalid scale opened ROS")
        ),
    )

    assert main(diagnostic_args("--gravity", "0.9", "--execute")) == 2
    assert "requires the validated gravity scale 1.0" in capsys.readouterr().out


def test_controller_activation_failure_writes_zero_profile_partial(
    monkeypatch, tmp_path
):
    arm = HandoffReplayArm(READY_TARGET_RAD)
    arm.require_position_effort_controllers_active = lambda timeout_sec=None: (
        (_ for _ in ()).throw(RuntimeError("effort controller unavailable"))
    )
    install_replay(monkeypatch, arm)
    output = tmp_path / "controller-failed.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3
    result = json.loads(output.read_text())["result"]
    assert result["termination"] == "gravity_activation_failed"
    assert result["diagnostic_execution"]["position_publish_count"] == 0
    assert not result["gravity_compensation"]["activated"]
    assert not result["gravity_compensation"]["cleanup"]["attempted"]
    assert arm.streamed == []


def test_reacquisition_exception_safe_holds_before_cleanup(
    monkeypatch, tmp_path
):
    arm = HandoffReplayArm(
        [-0.0887, 0.1055, 0.0013, 0.4801, -0.0261, -0.0380, -0.0345]
    )
    install_replay(monkeypatch, arm)
    monkeypatch.setattr(
        "robot_control.cli._minimum_jerk_trajectory",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("trajectory generation failed")
        ),
    )
    output = tmp_path / "reacquisition-exception.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3
    result = json.loads(output.read_text())["result"]
    reacquisition = result["ready_reacquisition"]
    assert result["termination"] == "ready_reacquisition_failed"
    assert result["diagnostic_execution"]["position_publish_count"] == 0
    assert reacquisition["safe_hold"]["attempted"]
    assert reacquisition["safe_hold"]["applied"]
    assert result["gravity_compensation"]["cleanup"]["zero_published"]
    assert arm.ik_requests == []


def test_deterministic_handoff_does_not_depend_on_marker_service(
    monkeypatch, tmp_path
):
    arm = HandoffReplayArm(READY_TARGET_RAD)
    arm.read_marker_pose = lambda timeout_sec=None: (
        (_ for _ in ()).throw(RuntimeError("marker service unavailable"))
    )
    install_replay(monkeypatch, arm)
    output = tmp_path / "marker-failed.json"

    _install_fast_reacquisition(monkeypatch)
    assert main(diagnostic_args("--output", str(output), "--execute")) == 0
    result = json.loads(output.read_text())["result"]
    assert result["handoff_sync"]["marker_reanchored"]
    assert result["diagnostic_execution"]["position_publish_count"] > 0
    assert result["gravity_compensation"]["cleanup"]["zero_published"]


def test_convergence_timeout_safe_holds_and_writes_partial_json(
    monkeypatch, tmp_path
):
    offset = np.zeros(7)
    offset[0] = 0.20
    arm = HandoffReplayArm(READY_TARGET_RAD, track_positions=False)
    arm.ik_offset = offset
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "convergence-timeout.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3
    payload = json.loads(output.read_text())
    result = payload["result"]
    assert result["termination"] == "safety_refused"
    assert result["refusal"]["reason"] == "handoff_convergence_timeout"
    assert result["convergence_gate"]["safe_hold"]["attempted"]
    assert result["convergence_gate"]["safe_hold"]["applied"]
    assert result["diagnostic_execution"]["position_publish_count"] == 0
    assert not any(
        str(sample["stage"]).startswith("profile_")
        for sample in payload["trace"]
    )
    assert arm.ik_requests


class TimeoutRecordingArm(HandoffReplayArm):
    def __init__(self, initial_joints, *, track_positions=True):
        super().__init__(initial_joints, track_positions=track_positions)
        self.state_timeouts = []

    def read_state(self, timeout_sec=None):
        self.state_timeouts.append(timeout_sec)
        return super().read_state(timeout_sec)


class MovingResidualArm(HandoffReplayArm):
    def __init__(self):
        residual = READY_TARGET_RAD.copy()
        residual[3] -= 0.0532
        super().__init__(residual, track_positions=False)
        self._state_reads = 0

    def read_state(self, timeout_sec=None):
        self._state_reads += 1
        measured = self.joints.copy()
        measured[3] += 0.0021 if self._state_reads % 2 else -0.0021
        return measured


class MissingInitialStateArm(HandoffReplayArm):
    def __init__(self):
        super().__init__(READY_TARGET_RAD)
        self.state_timeouts = []

    def read_state(self, timeout_sec=None):
        from robot_control.ros_adapter import AdapterUnavailable

        self.state_timeouts.append(timeout_sec)
        raise AdapterUnavailable(
            f"no /joint_states within {timeout_sec} s (simulated)"
        )


class FailHandoffSyncArm(HandoffReplayArm):
    def __init__(self):
        super().__init__(READY_TARGET_RAD)
        self._state_reads = 0

    def read_state(self, timeout_sec=None):
        from robot_control.ros_adapter import AdapterUnavailable

        self._state_reads += 1
        if self._state_reads == 3:
            raise AdapterUnavailable(
                "lost /joint_states during measured-state handoff (simulated)"
            )
        return super().read_state(timeout_sec)


def test_stationary_j4_residual_above_old_tolerance_is_adopted(
    monkeypatch, tmp_path
):
    residual = READY_TARGET_RAD.copy()
    residual[3] -= 0.0532
    arm = TimeoutRecordingArm(residual, track_positions=False)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "stationary-residual.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 0

    result = json.loads(output.read_text())["result"]
    ready = result["ready_reacquisition"]
    assert ready["required"]
    assert ready["completed"]
    assert ready["decision"] == "accepted_safe_stationary_measured_state"
    assert ready["target_accuracy_required"] is False
    assert ready["max_abs_final_error_rad"] == pytest.approx(0.0532)
    assert ready["max_abs_final_error_rad"] > 0.050
    assert ready["stationary"]["threshold_rad"] == pytest.approx(0.002)
    assert ready["stationary"]["decision_window_max_delta_rad"] <= 0.002
    assert result["ready_posture"]["passed"]
    handoff = result["handoff_sync"]
    assert handoff["measured_state_resynchronized"]
    assert handoff["marker_reanchored"]
    assert handoff["command_reanchored"]
    assert handoff["ik_seed_reanchored"]
    np.testing.assert_allclose(handoff["measured_joints_rad"], residual)
    assert result["convergence_gate"]["completed"]
    assert result["diagnostic_execution"]["position_publish_count"] > 0


def test_same_ready_residual_is_rejected_while_feedback_keeps_moving(
    monkeypatch, tmp_path
):
    arm = MovingResidualArm()
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    monkeypatch.setattr(
        "robot_control.cli.FOLLOW_READY_STATIONARY_DWELL_SEC", 0.005
    )
    monkeypatch.setattr("robot_control.cli.READY_SETTLE_TIMEOUT_SEC", 0.02)
    output = tmp_path / "moving-residual.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3

    result = json.loads(output.read_text())["result"]
    ready = result["ready_reacquisition"]
    assert result["termination"] == "ready_reacquisition_failed"
    assert ready["decision"] == "rejected_not_stationary"
    assert not ready["decision_passed"]
    assert ready["stationary"]["observed_worst_delta_rad"] > 0.002
    assert "remained in motion" in ready["termination_reason"]
    assert result["diagnostic_execution"]["position_publish_count"] == 0


@pytest.mark.parametrize(
    ("measured", "message"),
    [
        (
            np.array([0.0, 0.2, 0.0, -0.001, 0.0, 0.0, 0.0]),
            "joint limit violation",
        ),
        (
            np.array([0.061, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0]),
            "outside the safe A-prime neighbourhood",
        ),
    ],
)
def test_follow_ready_rejects_joint_limit_or_safe_neighbourhood_violation(
    monkeypatch, tmp_path, measured, message
):
    arm = HandoffReplayArm(measured, track_positions=False)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "unsafe-ready.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3

    result = json.loads(output.read_text())["result"]
    assert result["termination"] == "ready_reacquisition_failed"
    assert message in result["termination_reason"]
    assert result["diagnostic_execution"]["position_publish_count"] == 0


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_non_finite_initial_joint_feedback_is_refused_with_partial_json(
    monkeypatch, tmp_path, bad
):
    measured = READY_TARGET_RAD.copy()
    measured[3] = bad
    arm = HandoffReplayArm(READY_TARGET_RAD)
    arm.joints = measured
    install_replay(monkeypatch, arm)
    output = tmp_path / "non-finite.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3

    result = json.loads(output.read_text())["result"]
    acquisition = result["initial_joint_state_acquisition"]
    assert result["termination"] == "invalid_initial_joint_state_feedback"
    assert acquisition["received"]
    assert not acquisition["valid"]
    assert "non-finite" in acquisition["failure_reason"]
    assert result["diagnostic_execution"]["position_publish_count"] == 0


def test_initial_joint_state_uses_three_second_acquisition_then_one_second_watchdog(
    monkeypatch, tmp_path
):
    arm = TimeoutRecordingArm(READY_TARGET_RAD)
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "timeouts.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 0

    payload = json.loads(output.read_text())
    acquisition = payload["result"]["initial_joint_state_acquisition"]
    assert arm.state_timeouts[0] == pytest.approx(3.0)
    assert all(
        timeout == pytest.approx(1.0) for timeout in arm.state_timeouts[1:]
    )
    assert acquisition["received"]
    assert acquisition["valid"]
    assert acquisition["timeout_sec"] == pytest.approx(3.0)
    assert acquisition["wait_sec"] >= 0.0
    assert payload["settings"]["feedback_watchdog_sec"] == pytest.approx(1.0)


def test_initial_joint_state_timeout_is_clear_and_records_three_seconds(
    monkeypatch, tmp_path, capsys
):
    arm = MissingInitialStateArm()
    install_replay(monkeypatch, arm)
    output = tmp_path / "initial-timeout.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 2

    result = json.loads(output.read_text())["result"]
    acquisition = result["initial_joint_state_acquisition"]
    assert arm.state_timeouts == [3.0]
    assert result["termination"] == "initial_joint_state_acquisition_failed"
    assert acquisition["timeout_sec"] == pytest.approx(3.0)
    assert "no /joint_states within 3.0 s" in acquisition["failure_reason"]
    assert "no /joint_states within 3.0 s" in capsys.readouterr().out


def test_measured_state_handoff_sync_failure_refuses_profile(
    monkeypatch, tmp_path
):
    arm = FailHandoffSyncArm()
    install_replay(monkeypatch, arm)
    _install_fast_reacquisition(monkeypatch)
    output = tmp_path / "handoff-sync-failed.json"

    assert main(diagnostic_args("--output", str(output), "--execute")) == 3

    result = json.loads(output.read_text())["result"]
    assert result["termination"] == "handoff_sync_failed"
    assert result["handoff_sync"] is None
    assert "measured-state handoff" in result["termination_reason"]
    assert result["diagnostic_execution"]["position_publish_count"] == 0


def test_standalone_ready_accuracy_contract_remains_020_rad():
    assert READY_TOLERANCE_RAD == pytest.approx(0.020)
    just_outside = READY_TARGET_RAD.copy()
    just_outside[3] += 0.0201
    assert not check_ready(just_outside).passed
