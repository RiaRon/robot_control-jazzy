import sys

import numpy as np
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


def test_collect_needs_to_be_told_which_group(capsys):
    """Group-scoped like everything else that reaches a controller.

    It used to build an excitation over all 43 profile joints — both arms plus
    the whole hand — which no single controller could have accepted.
    """
    assert main(["r2s", "collect", "--profile", "openarm_tesollo"]) == 2
    assert "--group" in capsys.readouterr().out


def test_collect_dry_run_still_needs_the_robot(no_ros, capsys):
    """The excitation is built around the arm's current pose, so a dry run has
    to read it. Building one around the midpoint of the range instead would
    review a track that --execute would not publish."""
    code = main(
        ["r2s", "collect", "--profile", "openarm_tesollo",
         "--group", "openarm_right_arm"]
    )

    assert code == 2
    assert "rclpy" in capsys.readouterr().out


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


class DroopingArm:
    """A stub adapter whose joints stop short of every command, as the real
    ones do: the DM motors hold position by *having* a position error, so a
    command repeated unchanged reproduces the same shortfall."""

    #: Radians of droop at zero load angle. The real value is the holding
    #: torque divided by kp, so it follows the pose rather than the step size.
    DROOP = 0.05

    def __init__(self, *_args, **_kwargs):
        from robot_control.ros_adapter import Pose

        self._Pose = Pose
        self.joints = np.zeros(7)
        self.target = np.full(7, 0.2)
        self.sent = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    # Position along a single axis stands in for the tool centre point, so the
    # residual the CLI prints is a readable multiple of the joint error.
    def read_pose(self):
        return self._Pose((float(self.joints.mean()), 0.0, 0.0), (0, 0, 0, 1), "world")

    def read_marker_pose(self):
        return self._Pose((float(self.target.mean()), 0.0, 0.0), (0, 0, 0, 1), "world")

    def read_state(self):
        return self.joints.copy()

    def solve_ik(self, _pose, seed):
        return self.target.copy()

    def send_trajectory(self, points, period_sec):
        command = np.asarray(points[-1], dtype=float)
        self.sent.append(command)
        # The joint settles where kp times the error balances the load, so it
        # stops short by an amount the pose sets, not the size of the step.
        self.joints = command - self.DROOP * np.cos(command)


@pytest.fixture
def drooping(monkeypatch):
    from robot_control import ros_adapter

    arm = DroopingArm()
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: arm)
    return arm


def test_pose_ee_reports_how_far_short_the_arm_stopped(drooping, capsys):
    """A silent miss is the failure mode; the residual is always printed."""
    assert main(["pose", "ee", *RIGHT_ARM, "--from-marker", "--execute"]) == 0

    output = capsys.readouterr().out
    assert "residual" in output
    assert len(drooping.sent) == 1


def test_pose_ee_settle_drives_the_residual_below_the_tolerance(drooping, capsys):
    assert (
        main(
            [
                "pose",
                "ee",
                *RIGHT_ARM,
                "--from-marker",
                "--execute",
                "--settle",
                "--tolerance",
                "0.002",
            ]
        )
        == 0
    )

    assert len(drooping.sent) > 1, "settle sent no correction"
    np.testing.assert_allclose(drooping.joints, drooping.target, atol=0.002)
    assert "settled" in capsys.readouterr().out


def test_pose_ee_settle_corrects_by_adding_the_shortfall(drooping):
    """Re-sending the IK solution unchanged would reproduce the same droop, so
    each pass has to command past the target by what the last pass missed."""
    main(["pose", "ee", *RIGHT_ARM, "--from-marker", "--execute", "--settle"])

    first, second = drooping.sent[0], drooping.sent[1]
    np.testing.assert_allclose(first, drooping.target)
    assert (second > first).all(), "the correction did not command past the target"


def test_pose_ee_settle_requires_execute(no_ros, capsys):
    """A dry run sends nothing, so there is no shortfall to correct."""
    assert main(["pose", "ee", *RIGHT_ARM, "--from-marker", "--settle"]) == 2
    assert "--execute" in capsys.readouterr().out


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


class StiffArm:
    """A stub adapter whose tracking error shrinks as gravity torque is added.

    Modelled the way the real arm behaves: a joint holds position by sitting
    short of its command by (unmet load / kp), so feedforward torque matching
    the load removes the error and torque past it overshoots.
    """

    #: What the hardware's DEFAULT_KP applies on the wrist joints upward.
    KP = 20.0

    def __init__(self):
        self.published = []
        self.applied = np.zeros(7)
        self.load = np.zeros(7)

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_robot_description(self):
        return "<robot name='stub'/>"  # unused: the chain is injected

    def read_state(self):
        return np.zeros(7)

    def send_effort(self, effort):
        self.applied = np.asarray(effort, dtype=float).copy()
        self.published.append(self.applied)

    def read_tracking_error(self):
        return (self.load - self.applied) / self.KP


def _stub_chain(masses):
    """A seven-joint chain about +y, each link's mass a metre out along +x."""
    from robot_control import kinematics

    joints = tuple(
        kinematics.Revolute(
            f"j{index}",
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, 0.0, 0.0]) if index == 0 else np.array([1e-6, 0.0, 0.0]),
            np.eye(3),
            f"l{index}",
        )
        for index in range(7)
    )
    links = tuple(
        kinematics.Link(f"l{index}", mass, np.array([1.0, 0.0, 0.0]))
        for index, mass in enumerate(masses)
    )
    return kinematics.Chain(joints, links)


@pytest.fixture
def stiff(monkeypatch):
    from robot_control import ros_adapter

    arm = StiffArm()
    # One light link a metre out on the last joint only, so the modelled torque
    # is unambiguous and stays inside the 20 N.m the profile allows the wrist
    # joints even at the largest scale a sweep can ask for.
    chain = _stub_chain([0.0] * 6 + [0.5])
    arm.load = chain.gravity_torque(np.zeros(7))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: arm)
    monkeypatch.setattr("robot_control.cli._gravity_chain", lambda *a: chain)
    return arm


def test_pose_gravity_dry_run_publishes_nothing(stiff, capsys):
    assert main(["pose", "gravity", *RIGHT_ARM, "--scale", "1.0"]) == 0

    assert stiff.published == []
    assert "DRY RUN" in capsys.readouterr().out


def test_pose_gravity_releases_the_torque_when_it_finishes(stiff, capsys):
    assert (
        main(
            ["pose", "gravity", *RIGHT_ARM, "--scale", "1.0", "--execute",
             "--hold-sec", "0.02"]
        )
        == 0
    )

    # Whatever was held during the run, the last thing published is zero: torque
    # left applied after the process exits would keep pushing.
    np.testing.assert_allclose(stiff.published[-1], np.zeros(7))
    assert "torque released" in capsys.readouterr().out


def test_pose_gravity_sweep_names_the_scale_that_measured_best(stiff, capsys):
    assert (
        main(
            ["pose", "gravity", *RIGHT_ARM, "--execute", "--hold-sec", "0.02",
             "--sweep", "0,0.5,1.0"]
        )
        == 0
    )

    output = capsys.readouterr().out
    # The stub's error vanishes at full compensation, so 1 must win.
    assert "best measured scale: 1" in output
    for scale in ("0.00", "0.50", "1.00"):
        assert scale in output


def test_pose_gravity_refuses_a_scale_that_would_overcompensate(stiff, capsys):
    assert main(["pose", "gravity", *RIGHT_ARM, "--scale", "3.0", "--execute"]) == 2

    assert "outside 0 to 1.5" in capsys.readouterr().out
    assert stiff.published == []


def test_pose_gravity_refuses_a_group_with_no_effort_controller(no_ros, capsys):
    code = main(["pose", "gravity", "--group", "tesollo_curl", "--scale", "1.0"])

    assert code == 2
    assert "effort_controller" in capsys.readouterr().out


def test_pose_gravity_refuses_torque_over_the_profile_limit(stiff, capsys, monkeypatch):
    """A scale inside range can still ask for more torque than a joint allows."""
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain", lambda *a: _stub_chain([500.0] + [0.0] * 6)
    )

    code = main(["pose", "gravity", *RIGHT_ARM, "--scale", "1.0", "--execute"])

    assert code == 3
    assert "effort limit exceeded" in capsys.readouterr().out


class DraggableArm(StiffArm):
    """A stub that reports a marker being dragged, and follows what it is sent."""

    def __init__(self, target=None, target_after_anchor=None):
        super().__init__()
        self.joints = np.zeros(7)
        self.streamed = []
        self._target = target
        self._target_after_anchor = target_after_anchor
        self.pumped = 0
        self.ik_requests = []

    def watch_marker(self):
        self.watching = True

    def read_marker_pose(self, timeout_sec=None):
        """RViz가 현재 보관 중인 시작 마커 위치를 반환한다."""
        return self._target

    def latest_marker_target(self):
        return self._target

    def pump(self, timeout_sec=0.0):
        self.pumped += 1
        if self.pumped == 2 and self._target_after_anchor is not None:
            self._target = self._target_after_anchor

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def solve_ik(self, pose, seed):
        """테스트용 IK가 위치와 Roll·Pitch·Yaw를 모두 관절값으로 바꾼다."""
        from robot_control.cli import _rotation_from_quaternion

        self.ik_requests.append(
            (
                pose,
                np.asarray(seed, dtype=float).copy(),
            )
        )

        solution = np.asarray(seed, dtype=float).copy()

        # 테스트 체인의 J1~J3은 x, y, z 위치를 직접 나타낸다.
        solution[:3] = np.asarray(
            pose.position,
            dtype=float,
        )

        # 목표 quaternion을 3x3 회전행렬로 바꾼다.
        rotation = _rotation_from_quaternion(
            pose.orientation
        )

        # 테스트 체인은 Rx(roll) @ Ry(pitch) @ Rz(yaw) 순서로 회전한다.
        # 같은 순서의 Roll·Pitch·Yaw를 회전행렬에서 다시 추출한다.
        pitch = float(
            np.arcsin(
                np.clip(rotation[0, 2], -1.0, 1.0)
            )
        )
        roll = float(
            np.arctan2(
                -rotation[1, 2],
                rotation[2, 2],
            )
        )
        yaw = float(
            np.arctan2(
                -rotation[0, 1],
                rotation[0, 0],
            )
        )

        # 테스트 체인의 J4~J6이 Roll·Pitch·Yaw 역할을 한다.
        solution[3:6] = [
            roll,
            pitch,
            yaw,
        ]

        return solution

    def stream_positions(self, positions):
        self.joints = np.asarray(positions, dtype=float).copy()
        self.streamed.append(self.joints)


def _reachable_target(chain, q):
    """A pose the stub chain can actually hold, so following can converge."""
    from robot_control.ros_adapter import Pose

    pose = chain.pose(q)
    trace = np.trace(pose[:3, :3])
    w = np.sqrt(max(0.0, 1.0 + trace)) / 2.0
    if w < 1e-8:
        return Pose(tuple(pose[:3, 3]), (0.0, 0.0, 0.0, 1.0), "world")
    return Pose(
        tuple(pose[:3, 3]),
        (
            (pose[2, 1] - pose[1, 2]) / (4 * w),
            (pose[0, 2] - pose[2, 0]) / (4 * w),
            (pose[1, 0] - pose[0, 1]) / (4 * w),
            w,
        ),
        "world",
    )


class _CartesianTestChain:
    """Identity Cartesian axes for testing the CLI control-loop boundary."""

    links = ()

    def pose(self, q):
        from robot_control.kinematics import _rotation

        q = np.asarray(q, dtype=float)
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        pose[:3, :3] = (
            _rotation(np.array([1.0, 0.0, 0.0]), q[3])
            @ _rotation(np.array([0.0, 1.0, 0.0]), q[4])
            @ _rotation(np.array([0.0, 0.0, 1.0]), q[5])
        )
        return pose

    def delta_q(self, _q, twist):
        return np.concatenate((np.asarray(twist, dtype=float), [0.0]))

    def gravity_torque(self, _q):
        return np.zeros(7)


def _servo_chain():
    return _CartesianTestChain()


@pytest.fixture
def draggable(monkeypatch):
    from robot_control import ros_adapter

    chain = _servo_chain()
    arm = DraggableArm(
        target=_reachable_target(chain, np.zeros(7)),
        target_after_anchor=_reachable_target(chain, np.full(7, 0.05)),
    )
    arm.load = chain.gravity_torque(np.zeros(7))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: arm)
    monkeypatch.setattr("robot_control.cli._gravity_chain", lambda *a: chain)
    return arm


def test_quaternion_angular_distance_treats_opposite_sign_as_same():
    """q와 -q가 같은 방향으로 처리되는지 확인한다."""
    from robot_control.cli import _quaternion_angular_distance

    quaternion = (0.0, 0.0, 0.0, 1.0)
    opposite_sign = (0.0, 0.0, 0.0, -1.0)

    # Quaternion의 부호는 반대지만 실제 공간 방향은 완전히 같다.
    assert _quaternion_angular_distance(
        quaternion,
        opposite_sign,
    ) == pytest.approx(0.0)


@pytest.mark.parametrize(
    "goal",
    [
        # Roll: 손끝 x축을 기준으로 90도 회전한다.
        pytest.param(
            (np.sin(np.pi / 4.0), 0.0, 0.0, np.cos(np.pi / 4.0)),
            id="roll",
        ),

        # Pitch: 손끝 y축을 기준으로 90도 회전한다.
        pytest.param(
            (0.0, np.sin(np.pi / 4.0), 0.0, np.cos(np.pi / 4.0)),
            id="pitch",
        ),

        # Yaw: 손끝 z축을 기준으로 90도 회전한다.
        pytest.param(
            (0.0, 0.0, np.sin(np.pi / 4.0), np.cos(np.pi / 4.0)),
            id="yaw",
        ),
    ],
)
def test_quaternion_step_limits_all_three_rotation_axes(goal):
    """Roll, Pitch, Yaw 모두 한 번에 최대 0.10 rad만 이동해야 한다."""
    from robot_control.cli import (
        _quaternion_angular_distance,
        _step_quaternion_towards,
    )

    current = (0.0, 0.0, 0.0, 1.0)

    stepped, original_distance = _step_quaternion_towards(
        current,
        goal,
        max_step_rad=0.10,
    )

    # 원래 목표는 현재 방향에서 90도, 즉 pi/2 rad 떨어져 있다.
    assert original_distance == pytest.approx(np.pi / 2.0)

    # 생성된 중간목표는 현재 방향에서 정확히 0.10 rad 앞이어야 한다.
    assert _quaternion_angular_distance(
        current,
        stepped,
    ) == pytest.approx(0.10)

    # 중간목표에서 최종 목표까지 남은 각도도 올바르게 감소해야 한다.
    assert _quaternion_angular_distance(
        stepped,
        goal,
    ) == pytest.approx(np.pi / 2.0 - 0.10)


def test_quaternion_step_uses_goal_when_it_is_inside_the_limit():
    """목표가 0.10 rad 이내라면 중간값이 아니라 최종 목표를 사용한다."""
    from robot_control.cli import (
        _quaternion_angular_distance,
        _step_quaternion_towards,
    )

    current = (0.0, 0.0, 0.0, 1.0)

    # x축으로 0.05 rad 회전한 quaternion이다.
    goal = (
        np.sin(0.05 / 2.0),
        0.0,
        0.0,
        np.cos(0.05 / 2.0),
    )

    stepped, original_distance = _step_quaternion_towards(
        current,
        goal,
        max_step_rad=0.10,
    )

    assert original_distance == pytest.approx(0.05)

    # 제한 안쪽이므로 반환된 방향과 최종 목표의 차이는 0이어야 한다.
    assert _quaternion_angular_distance(
        stepped,
        goal,
    ) == pytest.approx(0.0)

@pytest.mark.parametrize(
    "rotation_joint",
    [
        pytest.param(3, id="roll"),
        pytest.param(4, id="pitch"),
        pytest.param(5, id="yaw"),
    ],
)
def test_pose_follow_tracks_all_rotation_axes_with_speed_limit(
    monkeypatch,
    rotation_joint,
    capsys,
):
    """Roll·Pitch·Yaw를 추종하며 최대 회전속도를 지켜야 한다."""
    from robot_control import ros_adapter
    from robot_control.cli import (
        _quaternion_angular_distance,
        _quaternion_from_rotation,
    )

    chain = _servo_chain()

    # 처음에는 실제 팔과 RViz 마커가 같은 영점자세에 있다.
    start_joints = np.zeros(7)
    start_target = _reachable_target(
        chain,
        start_joints,
    )

    # 드래그 후에는 선택된 한 회전축만 0.20 rad 회전한다.
    goal_joints = np.zeros(7)
    goal_joints[rotation_joint] = 0.20
    goal_target = _reachable_target(
        chain,
        goal_joints,
    )

    arm = DraggableArm(
        target=start_target,
        target_after_anchor=goal_target,
    )
    arm.load = chain.gravity_torque(start_joints)

    # 실제 ROS와 MoveIt 대신 테스트용 가짜 로봇과 체인을 연결한다.
    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    start_pose = chain.pose(arm.joints)
    goal_pose = chain.pose(goal_joints)

    start_orientation = _quaternion_from_rotation(
        start_pose[:3, :3]
    )
    goal_orientation = _quaternion_from_rotation(
        goal_pose[:3, :3]
    )
    error_before = _quaternion_angular_distance(
        start_orientation,
        goal_orientation,
    )

    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.6",
            ]
        )
        == 0
    )

    # 실행 후 실제 가짜 로봇 방향이 목표 방향에 가까워졌는지 확인한다.
    finish_pose = chain.pose(arm.joints)
    finish_orientation = _quaternion_from_rotation(
        finish_pose[:3, :3]
    )
    error_after = _quaternion_angular_distance(
        finish_orientation,
        goal_orientation,
    )

    assert error_after < error_before
    assert arm.streamed

    # 연속된 모든 명령 자세의 회전속도를 계산한다.
    streamed_orientations = [
        _quaternion_from_rotation(
            chain.pose(command)[:3, :3]
        )
        for command in arm.streamed
    ]
    angular_speeds = [
        _quaternion_angular_distance(first, second)
        / 0.01
        for first, second in zip(
            streamed_orientations,
            streamed_orientations[1:],
        )
    ]

    assert angular_speeds

    # 기본 제한인 0.20 rad/s를 넘어서는 명령이 없어야 한다.
    assert max(angular_speeds) <= 0.2000001
    # 실행 결과에 방향 추종 오차와 회전속도 제한 횟수가 출력되어야 한다.
    output = capsys.readouterr().out
    assert "TCP orientation trailed the marker" in output
    assert "last TCP orientation error" in output
    assert "Cartesian angular speed limit" in output
def test_pose_follow_dry_run_streams_nothing(draggable, capsys):
    assert main(["pose", "follow", *RIGHT_ARM, "--seconds", "0.1"]) == 0

    assert draggable.streamed == []
    assert "DRY RUN" in capsys.readouterr().out


def test_pose_follow_streams_towards_the_dragged_marker(draggable, capsys):
    # 가짜 로봇에서 pose follow를 0.4초 동안 실행한다.
    assert (
        main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.4"]) == 0
    )

    # 실제 관절 명령이 한 번 이상 전송되었는지 확인한다.
    assert draggable.streamed, "nothing was streamed"

    # 관절이 마커 방향으로 실제로 이동했는지 확인한다.
    assert np.abs(draggable.joints).sum() > 0.0

    # 터미널에 출력된 pose follow 실험 결과를 한 번만 가져온다.
    output = capsys.readouterr().out

    # 기존 실행 결과가 그대로 출력되는지 확인한다.
    assert "followed" in output

    # 관절별 진단표가 출력되는지 확인한다.
    assert "per-joint tracking diagnostics:" in output

    # 위치 제한의 전체·아래쪽·위쪽 열이 모두 출력되는지 확인한다.
    assert "position  lower  upper" in output

def test_pose_follow_solves_bounded_subgoals_from_measured_seed(draggable):
    from robot_control.cli import (
        _quaternion_angular_distance,
        _quaternion_from_rotation,
    )
    # 테스트에서 사용하는 가짜 7자유도 로봇의 순기구학 모델을 만든다.
    chain = _servo_chain()
    """IK 목표는 실제 TCP에서 최대 2cm 떨어진 중간 목표여야 한다."""

    assert (
        main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.2"])
        == 0
    )

    assert draggable.ik_requests

    submitted_distances = []

    for pose, seed in draggable.ik_requests:
        # 테스트용 로봇에서는 seed의 처음 세 값이 실제 TCP 위치다.
        measured_tcp = np.asarray(seed[:3], dtype=float)
        ik_position = np.asarray(pose.position, dtype=float)

        # 실제 TCP에서 IK 중간 목표까지의 거리를 계산한다.
        submitted_distance = float(
            np.linalg.norm(ik_position - measured_tcp)
        )
        submitted_distances.append(submitted_distance)

        # 어떤 IK 요청도 기본 최대 중간 거리 2cm를 넘으면 안 된다.
        assert submitted_distance <= 0.020000001

        # IK 계산에 사용한 실제 관절 seed로 현재 TCP 방향을 계산한다.
        measured_orientation = _quaternion_from_rotation(
            chain.pose(seed)[:3, :3]
        )

        # 현재 실제 방향과 IK 회전 중간목표 사이의 회전각을 계산한다.
        submitted_angle = _quaternion_angular_distance(
            measured_orientation,
            pose.orientation,
        )

        # 어떤 IK 요청도 기본 최대 회전 중간간격인 0.10 rad를
        # 넘어서는 안 된다.
        assert submitted_angle <= 0.100000001

    # 먼 마커 목표가 실제로 2cm 중간 목표로 제한됐는지 확인한다.
    assert any(
        np.isclose(distance, 0.02, atol=1e-6)
        for distance in submitted_distances
    )


def test_pose_follow_aligns_actual_tcp_to_initial_marker(monkeypatch):
    """실제 TCP가 처음부터 떨어진 파란 마커를 향해 이동해야 한다."""

    from robot_control import ros_adapter

    # 위치와 관절축이 일치하는 시험용 가짜 로봇 모델을 만든다.
    chain = _servo_chain()

    # 실제 로봇은 원점에 있지만 파란 마커는 x축으로 5cm 앞에 둔다.
    marker_joints = np.zeros(7)
    marker_joints[0] = 0.05
    marker = _reachable_target(chain, marker_joints)

    # 마커가 움직이지 않는 가짜 로봇 어댑터를 만든다.
    class StartupMarkerOnlyArm(DraggableArm):
        """시작 마커는 서비스로 읽히지만 드래그 피드백은 아직 없다."""

        def latest_marker_target(self):
            # 실제 RViz처럼 사용자가 드래그하기 전에는 새 피드백이 없다.
            return None

    arm = StartupMarkerOnlyArm(target=marker)
    arm.load = chain.gravity_torque(np.zeros(7))

    # 실제 ROS 대신 위에서 만든 가짜 로봇과 가짜 운동학을 사용한다.
    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    # 시험에서는 안정화 대기시간을 0초로 설정해 바로 정렬을 확인한다.
    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.2",
                "--startup-settle-sec",
                "0",
            ]
        )
        == 0
    )

    # 파란 마커를 향한 IK 계산이 실제로 요청되었는지 확인한다.
    assert arm.ik_requests

    # 첫 번째 IK 목표는 5cm 전체가 아니라 최대 2cm 중간 목표여야 한다.
    first_pose, _first_seed = arm.ik_requests[0]
    np.testing.assert_allclose(
        first_pose.position,
        [0.02, 0.0, 0.0],
        atol=1e-6,
    )

    # 실제 가짜 로봇도 원점에서 파란 마커 방향으로 움직였는지 확인한다.
    assert arm.joints[0] > 0.0
    assert arm.joints[0] < 0.05


def test_pose_follow_refuses_a_distant_start_marker(monkeypatch, capsys):
    """시작 마커가 허용거리보다 멀면 실물을 움직이지 않아야 한다."""

    from robot_control import ros_adapter

    # 위치와 관절축이 일치하는 시험용 가짜 로봇 모델을 만든다.
    chain = _servo_chain()

    # 실제 로봇은 원점이지만 파란 마커를 x축 20cm 앞에 둔다.
    # 기본 시작 허용거리 10cm를 초과하도록 만든 값이다.
    marker_joints = np.zeros(7)
    marker_joints[0] = 0.20
    marker = _reachable_target(chain, marker_joints)

    arm = DraggableArm(target=marker)
    arm.load = chain.gravity_torque(np.zeros(7))

    # 실제 ROS 대신 가짜 로봇을 사용한다.
    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    # 시작 거리가 너무 크므로 안전 오류 코드 2로 종료되어야 한다.
    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.2",
                "--startup-settle-sec",
                "0",
            ]
        )
        == 2
    )

    # 너무 멀다는 원인이 터미널에 표시되는지 확인한다.
    assert "exceeds --max-start-distance" in capsys.readouterr().out

    # 안전 거부가 발생했으므로 관절 명령은 전혀 보내지 않아야 한다.
    assert arm.streamed == []


def test_pose_follow_rejects_invalid_cartesian_servo_settings(no_ros, capsys):
    for option, value in (
        ("--kp", "0"),
        ("--ki", "-1"),
        ("--tolerance", "0"),
        ("--max-tcp-speed", "nan"),
        # 중간 IK 거리는 유한한 값이어야 한다.
        ("--max-ik-step", "nan"),
        # 기본 tolerance 0.002m보다 작은 값도 허용하지 않는다.
        ("--max-ik-step", "0.001"),
                # 안정화 시간은 음수나 숫자가 아닌 값을 허용하지 않는다.
        ("--startup-settle-sec", "-1"),
        ("--startup-settle-sec", "nan"),

        # 시작 허용거리는 유한하고 기본 TCP 허용오차 이상이어야 한다.
        ("--max-start-distance", "nan"),
        ("--max-start-distance", "0.001"),
                # 방향 허용오차는 유한한 양수여야 한다.
        ("--orientation-tolerance", "0"),
        ("--orientation-tolerance", "nan"),

        # 최대 회전속도는 유한한 양수여야 한다.
        ("--max-tcp-angular-speed", "0"),
        ("--max-tcp-angular-speed", "nan"),

        # 회전 중간목표는 방향 허용오차 이상이어야 한다.
        ("--max-ik-angular-step", "nan"),
        ("--max-ik-angular-step", "0.01"),

        # 시작 허용각은 방향 허용오차 이상이고 pi 이하여야 한다.
        ("--max-start-angle", "nan"),
        ("--max-start-angle", "0.01"),
        ("--max-start-angle", "4.0"),
    ):
        assert main(["pose", "follow", *RIGHT_ARM, option, value]) == 2


def test_pose_follow_refuses_large_startup_orientation_difference(
    draggable,
    capsys,
):
    """시작 방향 차이가 너무 크면 실물 명령을 거부해야 한다."""
    from robot_control.ros_adapter import Pose

    target = draggable._target

    # 현재 방향에서 Roll 방향으로 90도 회전된 마커를 만든다.
    # 기본 시작 허용각 0.35 rad보다 훨씬 큰 1.57 rad 차이다.
    draggable._target = Pose(
        target.position,
        (
            np.sin(np.pi / 4.0),
            0.0,
            0.0,
            np.cos(np.pi / 4.0),
        ),
        target.frame_id,
    )

    code = main(
        [
            "pose",
            "follow",
            *RIGHT_ARM,
            "--execute",
            "--seconds",
            "0.2",
        ]
    )

    # 위험한 시작 방향 차이이므로 명령을 실행하지 않고 오류로 종료한다.
    assert code == 2
    assert "--max-start-angle" in capsys.readouterr().out
    assert draggable.streamed == []


def test_pose_follow_recovers_marker_orientation_after_a_disturbance(
    monkeypatch,
):
    """외란을 받은 뒤에도 시작 방향이 아니라 RViz 목표 방향으로 복귀한다."""
    from robot_control import ros_adapter
    from robot_control.cli import (
        _quaternion_angular_distance,
        _quaternion_from_rotation,
    )

    chain = _servo_chain()

    # RViz 마커의 목표 위치와 방향을 만드는 관절값이다.
    goal_joints = np.full(7, 0.01)
    goal_target = _reachable_target(
        chain,
        goal_joints,
    )

    class DisturbedArm(DraggableArm):
        def __init__(self):
            super().__init__(target=goal_target)
            self.disturbed = False
            self.joints_after_disturbance = None

        def read_state(self, timeout_sec=None):
            # 첫 명령이 전달된 뒤 Roll 역할을 하는 테스트 관절에
            # 0.02 rad의 외란을 한 번 추가한다.
            if self.streamed and not self.disturbed:
                self.joints[3] += 0.02
                self.disturbed = True
                self.joints_after_disturbance = (
                    self.joints.copy()
                )

            return self.joints.copy()

    arm = DisturbedArm()

    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.3",
            ]
        )
        == 0
    )

    assert arm.disturbed
    assert arm.joints_after_disturbance is not None

    # RViz 마커의 목표 방향을 계산한다.
    goal_orientation = _quaternion_from_rotation(
        chain.pose(goal_joints)[:3, :3]
    )

    # 외란 직후의 방향 오차를 계산한다.
    disturbed_orientation = _quaternion_from_rotation(
        chain.pose(arm.joints_after_disturbance)[:3, :3]
    )
    disturbed_error = _quaternion_angular_distance(
        disturbed_orientation,
        goal_orientation,
    )

    # 제어가 끝났을 때의 방향 오차를 계산한다.
    finish_orientation = _quaternion_from_rotation(
        chain.pose(arm.joints)[:3, :3]
    )
    finish_error = _quaternion_angular_distance(
        finish_orientation,
        goal_orientation,
    )

    # 제어 후 방향 오차가 외란 직후보다 작아져야 한다.
    assert finish_error < disturbed_error


def test_pose_follow_limits_streamed_tcp_speed_to_default(draggable):
    chain = _servo_chain()

    assert (
        main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.3"])
        == 0
    )

    positions = np.array([chain.pose(q)[:3, 3] for q in draggable.streamed])
    speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1) / 0.01
    assert np.max(speeds) <= 0.05 + 1e-9


def test_pose_follow_refuses_a_stale_marker_far_from_current_tcp(
    monkeypatch,
):
    """실제 TCP에서 너무 먼 오래된 마커는 시작 목표로 사용하지 않는다."""

    from robot_control import ros_adapter
    from robot_control.ros_adapter import Pose

    chain = _servo_chain()

    # 파란 마커는 실제 TCP에서 약 56cm 떨어진 오래된 위치에 있다.
    arm = DraggableArm(
        target=Pose(
            (0.5, -0.4, 0.3),
            (0.0, 0.0, 0.0, 1.0),
            "world",
        )
    )
    arm.joints[:3] = np.array([0.1, -0.1, 0.05])
    start = arm.joints.copy()

    # 실제 ROS 대신 가짜 로봇과 가짜 운동학을 사용한다.
    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    # 기본 허용거리 10cm를 초과하므로 안전하게 거부되어야 한다.
    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.1",
            ]
        )
        == 2
    )

    # 안전 거부 후 실제 관절 자세가 전혀 바뀌지 않아야 한다.
    np.testing.assert_allclose(arm.joints, start)
    assert arm.streamed == []


def test_pose_follow_tracks_marker_after_startup_alignment(monkeypatch):
    """시작 정렬이 끝난 뒤 마커 이동을 실제 TCP가 따라가야 한다."""

    from robot_control import ros_adapter
    from robot_control.ros_adapter import Pose

    chain = _servo_chain()

    class MovingMarkerArm(DraggableArm):
        """두 번째 제어주기에 파란 마커를 x축으로 1cm 이동시킨다."""

        def pump(self, timeout_sec=0.0):
            super().pump(timeout_sec)

            # 첫 제어주기에는 실제 TCP와 파란 마커가 같은 위치이므로
            # 시작 정렬이 완료된다. 그다음 주기에 마커를 1cm 움직인다.
            if self.pumped == 2:
                x, y, z = self._target.position
                self._target = Pose(
                    (x + 0.01, y, z),
                    self._target.orientation,
                    "world",
                )

    # 실제 TCP와 파란 마커를 처음부터 같은 위치에 둔다.
    start_position = np.array([0.1, -0.1, 0.05])
    arm = MovingMarkerArm(
        target=Pose(
            tuple(start_position),
            (0.0, 0.0, 0.0, 1.0),
            "world",
        )
    )
    arm.joints[:3] = start_position
    start = arm.joints.copy()

    # 실제 ROS 대신 가짜 로봇과 가짜 운동학을 사용한다.
    monkeypatch.setattr(
        ros_adapter,
        "RosAdapter",
        lambda *args, **kwargs: arm,
    )
    monkeypatch.setattr(
        "robot_control.cli._gravity_chain",
        lambda *args: chain,
    )

    # 시험에서는 안정화 시간을 0초로 설정해 첫 주기에 정렬을 완료한다.
    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                "0.5",
                "--startup-settle-sec",
                "0",
            ]
        )
        == 0
    )

    # 정렬 후 파란 마커가 움직인 x축 방향으로 실제 TCP도 이동해야 한다.
    assert arm.joints[0] > start[0]

    # 움직이지 않은 y축과 z축은 시작 위치를 유지해야 한다.
    np.testing.assert_allclose(
        arm.joints[1:3],
        start[1:3],
        atol=1e-6,
    )

def test_pose_follow_holds_still_when_no_marker_has_been_dragged(capsys, monkeypatch):
    """No target is not a reason to command zero; it is a reason to command
    nothing, so the controller keeps holding where the arm already is."""
    from robot_control import ros_adapter

    chain = _servo_chain()
    arm = DraggableArm(target=None)
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: arm)
    monkeypatch.setattr("robot_control.cli._gravity_chain", lambda *a: chain)

    assert main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.2"]) == 0

    assert arm.streamed == []


def test_pose_follow_releases_torque_it_was_holding(draggable, capsys):
    assert (
        main(
            ["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.2",
             "--gravity", "1.0"]
        )
        == 0
    )

    np.testing.assert_allclose(draggable.published[-1], np.zeros(7))


def test_pose_follow_refuses_a_group_with_no_planning_group(no_ros, capsys):
    code = main(["pose", "follow", "--group", "tesollo_curl", "--seconds", "1"])

    assert code == 2
    assert "no planning group" in capsys.readouterr().out


def test_pose_follow_refuses_an_out_of_range_gravity_scale(no_ros, capsys):
    assert main(["pose", "follow", *RIGHT_ARM, "--gravity", "9"]) == 2
    assert "outside 0 to 1.5" in capsys.readouterr().out


class DroopingDraggableArm(DraggableArm):
    """A dragging stub that also droops, which fake hardware never does.

    This is the case that mattered: with droop larger than one period's velocity
    budget, a command rate-limited from the measured pose can never advance, and
    the arm sits still while the loop reports thousands of samples sent.
    """

    #: Radians the joint sits behind its command in order to hold position.
    DROOP = 0.03

    def stream_positions(self, positions):
        command = np.asarray(positions, dtype=float)
        self.streamed.append(command.copy())
        # Move towards the command, stopping the droop short of it.
        gap = command - self.joints
        self.joints = self.joints + np.sign(gap) * np.maximum(
            np.abs(gap) - self.DROOP, 0.0
        )


def test_pose_follow_advances_a_drooping_arm(monkeypatch, capsys):
    """The regression that a perfect-tracking stub cannot catch."""
    from robot_control import ros_adapter

    chain = _servo_chain()
    arm = DroopingDraggableArm(
        target=_reachable_target(chain, np.zeros(7)),
        target_after_anchor=_reachable_target(chain, np.full(7, 0.01)),
    )
    arm.DROOP = 0.003
    arm.load = chain.gravity_torque(np.zeros(7))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: arm)
    monkeypatch.setattr("robot_control.cli._gravity_chain", lambda *a: chain)

    assert (
        main(
            [
                "pose",
                "follow",
                *RIGHT_ARM,
                "--execute",
                "--seconds",
                    "1.5",
                ]
        )
        == 0
    )

    assert arm.streamed, "nothing was streamed"
    # The arm has to have actually moved, not merely been commanded at.
    assert np.abs(arm.joints).max() > arm.DROOP, (
        f"a drooping arm did not advance: {arm.joints}"
    )
    actual_position = chain.pose(arm.joints)[:3, 3]
    target_position = np.asarray(arm._target.position)
    assert np.linalg.norm(target_position - actual_position) <= 0.002


def test_pose_follow_reports_how_far_it_trailed_the_marker(draggable, capsys):
    """Clamp counts say the command moved; only the lag says the arm did."""
    assert main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.3"]) == 0

    output = capsys.readouterr().out
    assert "trailed the marker by" in output
    assert "mm on average" in output
    assert "last TCP position error" in output
    assert "within 2.0 mm" in output
    assert "actual control rate" in output
    assert "IK requests" in output


def test_pose_gravity_accepts_one_scale_per_joint(stiff, capsys):
    """The measured optima differ per joint, so one number is a compromise."""
    assert (
        main(
            ["pose", "gravity", *RIGHT_ARM, "--execute", "--hold-sec", "0.02",
             "--scale", "1.23,1.14,1.31,1.1,1.0,1.0,1.0"]
        )
        == 0
    )

    output = capsys.readouterr().out
    assert "1.23" in output and "1.14" in output


def test_pose_gravity_rejects_a_scale_count_that_is_not_one_or_per_joint(stiff, capsys):
    assert main(["pose", "gravity", *RIGHT_ARM, "--scale", "1.0,1.1"]) == 2
    assert "one value or one per joint" in capsys.readouterr().out


def test_pose_gravity_sweeps_a_single_joint_holding_the_others(stiff, capsys):
    """Refining one joint at a time is what per-joint tuning actually is."""
    assert (
        main(
            ["pose", "gravity", *RIGHT_ARM, "--execute", "--hold-sec", "0.02",
             "--scale", "1.1", "--sweep", "0.9,1.0,1.1", "--sweep-joint", "r_aj_7"]
        )
        == 0
    )

    output = capsys.readouterr().out
    assert "best measured scale for r_aj_7" in output
    # The suggested next command carries every joint's scale, not just this one.
    assert "--scale 1.1,1.1,1.1,1.1,1.1,1.1," in output


def test_pose_gravity_rejects_sweeping_a_joint_outside_the_group(stiff, capsys):
    code = main(
        ["pose", "gravity", *RIGHT_ARM, "--sweep", "1.0", "--sweep-joint", "l_aj_1"]
    )

    assert code == 2
    assert "not a joint of" in capsys.readouterr().out


def test_pose_gravity_requires_a_scale_or_a_sweep(stiff, capsys):
    assert main(["pose", "gravity", *RIGHT_ARM]) == 2
    assert "--scale, --sweep, or both" in capsys.readouterr().out


def test_pose_gravity_writes_what_it_measured(stiff, tmp_path, capsys):
    from robot_control.artifacts import read_sweep
    from robot_control.profile import load_builtin_profile

    path = tmp_path / "sweep.json"
    code = main(
        ["pose", "gravity", *RIGHT_ARM, "--execute", "--hold-sec", "0.02",
         "--sweep", "0,0.5,1.0", "--sweep-joint", "r_aj_7", "--output", str(path)]
    )

    assert code == 0
    assert str(path) in capsys.readouterr().out
    sweep = read_sweep(path, load_builtin_profile("openarm_tesollo"))
    assert sweep.rounds == 3
    assert sweep.group == "openarm_right_arm"
    assert sweep.sweep_joint == "r_aj_7"
    # The scale that varied is the one the sweep was told to vary, and the
    # errors are the ones the run printed, not a recomputation.
    np.testing.assert_allclose(sweep.scales[:, 6], [0.0, 0.5, 1.0])
    np.testing.assert_allclose(sweep.errors[-1], np.zeros(7), atol=1e-9)


def test_pose_gravity_writes_a_single_scale_too(stiff, tmp_path):
    """One pose at one scale is a round; several files make the fit."""
    from robot_control.artifacts import read_sweep
    from robot_control.profile import load_builtin_profile

    path = tmp_path / "one.json"
    code = main(
        ["pose", "gravity", *RIGHT_ARM, "--execute", "--hold-sec", "0.02",
         "--scale", "1.0", "--output", str(path)]
    )

    assert code == 0
    assert read_sweep(path, load_builtin_profile("openarm_tesollo")).rounds == 1


def test_pose_gravity_output_needs_execute(stiff, capsys):
    """A dry run measures nothing, so there is nothing honest to write."""
    code = main(["pose", "gravity", *RIGHT_ARM, "--scale", "1.0", "--output", "x.json"])

    assert code == 2
    assert "--execute" in capsys.readouterr().out


def test_pose_follow_accepts_one_gravity_scale_per_joint(draggable, capsys):
    assert (
        main(
            ["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.2",
             "--gravity", "1.23,1.14,1.31,1.1,1.0,1.0,1.0"]
        )
        == 0
    )

    np.testing.assert_allclose(draggable.published[-1], np.zeros(7))


def test_pose_follow_without_gravity_publishes_no_torque(draggable, capsys):
    assert main(["pose", "follow", *RIGHT_ARM, "--execute", "--seconds", "0.2"]) == 0

    assert draggable.published == [], "torque was published with --gravity absent"
    assert "gravity off" in capsys.readouterr().out
