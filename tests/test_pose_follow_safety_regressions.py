"""Safety regressions from the aborted 2026-08-20 real follow run."""

from __future__ import annotations

import json

import numpy as np

from robot_control.cli import main


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

    def read_marker_pose(self, timeout_sec=None):
        return self.target

    def latest_marker_target(self):
        return self.target

    def pump(self, timeout_sec=0.0):
        return None

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def solve_ik(self, pose, seed):
        self.ik_requests.append((pose, np.asarray(seed, dtype=float).copy()))
        return np.asarray(seed, dtype=float) + self.ik_offset

    def stream_positions(self, positions):
        command = np.asarray(positions, dtype=float).copy()
        self.streamed.append(command)
        self.joints = command


class SequenceSixBranchReplayArm(ReplayArm):
    """Return five continuous targets, then only the reported bad branch."""

    def __init__(self):
        initial = np.zeros(7)
        initial[3] = 0.021553368
        super().__init__(initial)
        self.solve_calls = 0

    def solve_ik(self, pose, seed):
        seed = np.asarray(seed, dtype=float).copy()
        self.ik_requests.append((pose, seed.copy()))
        self.solve_calls += 1
        if self.solve_calls <= 5:
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
    arm = ReplayArm(INCIDENT_INITIAL_JOINTS_RAD, branch_offset)
    install_replay(monkeypatch, arm)

    assert main(diagnostic_args("--execute")) == 3
    assert arm.ik_requests
    np.testing.assert_array_equal(
        arm.ik_requests[0][1], INCIDENT_INITIAL_JOINTS_RAD
    )
    assert arm.streamed == []
    output = capsys.readouterr().out
    assert "IK target jump refused before publish" in output
    assert "r_aj_3=+0.7646 rad" in output
    assert "r_aj_5=-0.7480 rad" in output


def test_deterministic_position_clamp_is_refused_before_publish(
    monkeypatch, capsys
):
    # J4 is already 0.005531 rad below the URDF and profile lower limit.
    # No artificial IK offset is needed to reproduce the incident clamp.
    arm = ReplayArm(INCIDENT_INITIAL_JOINTS_RAD)
    assert INCIDENT_INITIAL_JOINTS_RAD[3] < 0.0
    install_replay(monkeypatch, arm)

    assert main(diagnostic_args("--execute")) == 3
    assert arm.ik_requests
    assert arm.streamed == []
    np.testing.assert_array_equal(
        arm.ik_requests[0][1], INCIDENT_INITIAL_JOINTS_RAD
    )
    output = capsys.readouterr().out
    assert "deterministic profile position clamp refused before publish" in output
    assert "lower: r_aj_4" in output
    assert "upper:" not in output


def test_ik_target_jump_at_exact_hard_boundary_is_refused(monkeypatch, capsys):
    branch_offset = np.zeros(7)
    branch_offset[0] = 0.30
    arm = ReplayArm(np.zeros(7), branch_offset)
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
    arm = ReplayArm(np.zeros(7))
    install_replay(monkeypatch, arm)

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
