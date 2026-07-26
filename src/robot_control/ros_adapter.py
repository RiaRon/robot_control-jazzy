"""Optional ROS execution edge for canonical commands.

``import robot_control`` must not require ``rclpy``, so every ROS import lives
inside :class:`_RclpyBackend` and surfaces as :class:`AdapterUnavailable`
rather than as an import error. The adapter itself is pure Python and takes an
injected backend, which is how it is tested without a running ROS graph.

Nothing here decides whether a motion is safe. The adapter converts between the
canonical boundary and ROS names, and puts on the wire exactly what it is
given; :class:`~robot_control.safety.CommandGate` is what authorizes it.

Reading state, forward kinematics, and inverse kinematics only observe the
robot, so they need no ``execute``. Sending a goal does.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
import time
from typing import Any

import numpy as np

from .interface import CanonicalInterface, InterfaceError
from .profile import FOLLOW_JOINT_TRAJECTORY, PARALLEL_GRIPPER_COMMAND, RobotProfile
from .safety import SafetyError

# moveit_msgs/MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1
DEFAULT_TIMEOUT_SEC = 10.0


class AdapterUnavailable(RuntimeError):
    """ROS is not installed, not running, or not answering."""


class IkFailed(RuntimeError):
    """The IK service answered, and reported that the pose is not reachable."""


@dataclass(frozen=True)
class Pose:
    """An end-effector pose. Orientation is a quaternion in x, y, z, w order."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    frame_id: str = "world"

    def translated(self, offset: Sequence[float]) -> Pose:
        """Return this pose shifted by *offset*, keeping its orientation."""
        if len(offset) != 3:
            raise ValueError("a translation offset needs exactly three values")
        moved = tuple(float(a + b) for a, b in zip(self.position, offset))
        return Pose(moved, self.orientation, self.frame_id)

    def rotated_to(self, orientation: Sequence[float]) -> Pose:
        if len(orientation) != 4:
            raise ValueError("an orientation needs exactly four values")
        return Pose(self.position, tuple(float(v) for v in orientation), self.frame_id)

    @property
    def rpy(self) -> tuple[float, float, float]:
        return rpy_from_quaternion(self.orientation)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert fixed-axis roll/pitch/yaw to a unit quaternion in x, y, z, w order."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(
    orientation: Sequence[float],
) -> tuple[float, float, float]:
    """Convert a quaternion in x, y, z, w order to fixed-axis roll/pitch/yaw."""
    x, y, z, w = (float(v) for v in orientation)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # asin saturates rather than raising when the quaternion is slightly denormal.
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (roll, pitch, yaw)


def trajectory_failure(result: Any) -> str | None:
    """Describe why a FollowJointTrajectory result failed, or return None."""
    if result.error_code == 0:
        return None
    return f"error code {result.error_code}: {result.error_string}"


def gripper_failure(result: Any) -> str | None:
    """Describe why a ParallelGripperCommand result failed, or return None.

    The result carries no error code at all, only state, stalled, and
    reached_goal. Stalling is how a parallel gripper reports that it closed on
    something, so it counts as success.
    """
    if result.reached_goal or result.stalled:
        return None
    return "the gripper neither reached its commanded position nor stalled"


def make_backend(node_name: str = "robot_control_pose") -> Any:
    """Build one ROS backend, shareable by several adapters.

    Reading every group costs one node and one rclpy context rather than one
    per group, and each adapter created with it must not close it.
    """
    return _RclpyBackend(node_name)


class RosAdapter:
    """Convert canonical commands to controller traffic for one actuator group."""

    def __init__(
        self,
        profile: RobotProfile,
        group_name: str,
        execute: bool = False,
        backend: Any | None = None,
        node_name: str = "robot_control_pose",
    ):
        if group_name not in profile.groups:
            raise ValueError(
                f"unknown group {group_name!r}; known groups are "
                f"{sorted(profile.groups)}"
            )
        group = profile.groups[group_name]
        if group.controller is None:
            raise ValueError(
                f"group {group_name!r} declares no controller, so it cannot be "
                "commanded; add one to the profile"
            )
        self.profile = profile
        self.group = group
        # Reading state, FK, and IK observe the robot and are always allowed;
        # only putting a goal on the wire needs --execute.
        self.execute = execute
        self.interface = CanonicalInterface(profile)
        self._backend = backend if backend is not None else _RclpyBackend(node_name)

    def __enter__(self) -> RosAdapter:
        return self

    def __exit__(self, *_exception: object) -> None:
        self.close()

    def close(self) -> None:
        self._backend.close()

    def read_source_state(
        self, timeout_sec: float = DEFAULT_TIMEOUT_SEC
    ) -> dict[str, float]:
        """Return the latest ``/joint_states``, keyed by source joint name."""
        return self._backend.joint_states(timeout_sec)

    def read_state(self, timeout_sec: float = DEFAULT_TIMEOUT_SEC) -> np.ndarray:
        """Return this group's canonical joint values."""
        source = self.read_source_state(timeout_sec)
        try:
            return self.interface.group_state_to_canonical(self.group.name, source)
        except InterfaceError as error:
            # The graph is up but not publishing this group; that is an
            # environment problem, not a bad request.
            raise AdapterUnavailable(
                f"/joint_states does not cover group {self.group.name!r}: {error}"
            ) from error

    def read_pose(self, timeout_sec: float = DEFAULT_TIMEOUT_SEC) -> Pose:
        """Return the group's end-effector pose from ``/compute_fk``."""
        tip = self._require_planning_group()
        seed = self.read_source_state(timeout_sec)
        code, pose = self._backend.compute_fk(tip, seed, "world", timeout_sec)
        if code != MOVEIT_SUCCESS or pose is None:
            raise IkFailed(
                f"forward kinematics for {tip} failed with MoveIt error code {code}"
            )
        return pose

    def solve_ik(
        self,
        pose: Pose,
        seed: Sequence[float],
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    ) -> np.ndarray:
        """Solve IK for *pose*, seeded from canonical *seed*, in canonical order."""
        tip = self._require_planning_group()
        source_seed = self.interface.group_command_to_source(self.group.name, seed)
        code, solution = self._backend.compute_ik(
            self.group.moveit_group, tip, pose, source_seed, timeout_sec
        )
        if code != MOVEIT_SUCCESS:
            raise IkFailed(
                f"no IK solution for {tip} in group {self.group.moveit_group!r} "
                f"(MoveIt error code {code})"
            )
        try:
            # MoveIt answers with the whole robot state, so the group is
            # extracted rather than assumed to be the entire answer.
            return self.interface.group_state_to_canonical(self.group.name, solution)
        except InterfaceError as error:
            raise IkFailed(f"IK solution does not cover the group: {error}") from error

    def send_trajectory(
        self, points: Sequence[Sequence[float]], period_sec: float
    ) -> None:
        """Send an authorized canonical trajectory to the group's controller."""
        self._require_execute()
        self._require_action(FOLLOW_JOINT_TRAJECTORY)
        if not points:
            raise ValueError("a trajectory needs at least one waypoint")
        source = [
            list(
                self.interface.group_command_to_source(self.group.name, point).values()
            )
            for point in points
        ]
        self._backend.follow_joint_trajectory(
            self.group.controller,
            self.interface.group_source_names(self.group.name),
            source,
            period_sec,
        )

    def send_gripper(self, position: float) -> None:
        """Send an authorized canonical position to the group's gripper action."""
        self._require_execute()
        self._require_action(PARALLEL_GRIPPER_COMMAND)
        source = self.interface.group_command_to_source(self.group.name, [position])
        joint, value = next(iter(source.items()))
        self._backend.gripper_command(self.group.controller, joint, value)

    def _require_execute(self) -> None:
        if not self.execute:
            raise SafetyError("publishing requires explicit --execute")

    def _require_planning_group(self) -> str:
        if self.group.moveit_group is None or self.group.tip_link is None:
            raise ValueError(
                f"group {self.group.name!r} has no planning group, so it has no "
                "end-effector pose; set it with direct joint values instead"
            )
        return self.group.tip_link

    def _require_action(self, action: str) -> None:
        if self.group.action != action:
            raise ValueError(
                f"group {self.group.name!r} is driven by {self.group.action}, "
                f"not {action}"
            )


class _RclpyBackend:
    """The real ROS backend. Every rclpy import is confined to this class."""

    def __init__(self, node_name: str):
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.qos import qos_profile_sensor_data
            from builtin_interfaces.msg import Duration
            from control_msgs.action import FollowJointTrajectory
            from control_msgs.action import ParallelGripperCommand
            from geometry_msgs.msg import Pose as PoseMsg
            from geometry_msgs.msg import PoseStamped, Quaternion, Point
            from moveit_msgs.srv import GetPositionFK, GetPositionIK
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Header
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        except ImportError as error:
            raise AdapterUnavailable(
                "the ROS adapter needs rclpy and the MoveIt message packages; "
                "source a ROS 2 Jazzy workspace "
                "(source ros_ws/install/setup.bash) and try again"
            ) from error

        self._rclpy = rclpy
        self._ActionClient = ActionClient
        self._sensor_qos = qos_profile_sensor_data
        self._Duration = Duration
        self._FollowJointTrajectory = FollowJointTrajectory
        self._ParallelGripperCommand = ParallelGripperCommand
        self._PoseMsg, self._PoseStamped = PoseMsg, PoseStamped
        self._Quaternion, self._Point = Quaternion, Point
        self._GetPositionFK, self._GetPositionIK = GetPositionFK, GetPositionIK
        self._JointState, self._Header = JointState, Header
        self._JointTrajectory = JointTrajectory
        self._JointTrajectoryPoint = JointTrajectoryPoint

        # Only shut down the context this backend started, so an adapter used
        # from inside an existing rclpy application does not tear it down.
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init()
        self._node = rclpy.create_node(node_name)
        self._joint_subscription = None
        self._latest: dict[str, float] | None = None
        self._fk_client = None
        self._ik_client = None

    def close(self) -> None:
        self._node.destroy_node()
        if self._owns_context:
            self._rclpy.shutdown()

    def joint_states(self, timeout_sec: float) -> dict[str, float]:
        if self._joint_subscription is None:
            self._joint_subscription = self._node.create_subscription(
                self._JointState, "/joint_states", self._record, self._sensor_qos
            )
        # Discard anything received earlier, so a caller never reads a pose
        # from before the motion it just commanded.
        self._latest = None
        deadline = time.monotonic() + timeout_sec
        while self._latest is None and time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
        if self._latest is None:
            raise AdapterUnavailable(
                f"no /joint_states within {timeout_sec} s; is the robot bringup "
                "running? (ros_ws/pose_bringup.sh)"
            )
        return dict(self._latest)

    def _record(self, message: Any) -> None:
        self._latest = dict(zip(message.name, message.position))

    def compute_fk(
        self,
        link: str,
        seed: Mapping[str, float],
        frame_id: str,
        timeout_sec: float,
    ) -> tuple[int, Pose | None]:
        if self._fk_client is None:
            self._fk_client = self._node.create_client(
                self._GetPositionFK, "/compute_fk"
            )
        request = self._GetPositionFK.Request()
        request.header.frame_id = frame_id
        request.fk_link_names = [link]
        request.robot_state.joint_state = self._joint_state(seed, frame_id)
        request.robot_state.is_diff = True
        response = self._call(self._fk_client, request, timeout_sec, "/compute_fk")
        if not response.pose_stamped:
            return (response.error_code.val, None)
        stamped = response.pose_stamped[0]
        position, orientation = stamped.pose.position, stamped.pose.orientation
        return (
            response.error_code.val,
            Pose(
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                stamped.header.frame_id or frame_id,
            ),
        )

    def compute_ik(
        self,
        group: str,
        link: str,
        pose: Pose,
        seed: Mapping[str, float],
        timeout_sec: float,
    ) -> tuple[int, dict[str, float]]:
        if self._ik_client is None:
            self._ik_client = self._node.create_client(
                self._GetPositionIK, "/compute_ik"
            )
        request = self._GetPositionIK.Request()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = link
        # The seed covers only the planning group, so it is a diff over the
        # planning scene's current state rather than a complete robot state.
        request.ik_request.robot_state.joint_state = self._joint_state(
            seed, pose.frame_id
        )
        request.ik_request.robot_state.is_diff = True
        request.ik_request.pose_stamped = self._PoseStamped(
            header=self._Header(frame_id=pose.frame_id),
            pose=self._PoseMsg(
                position=self._Point(
                    x=pose.position[0], y=pose.position[1], z=pose.position[2]
                ),
                orientation=self._Quaternion(
                    x=pose.orientation[0],
                    y=pose.orientation[1],
                    z=pose.orientation[2],
                    w=pose.orientation[3],
                ),
            ),
        )
        request.ik_request.timeout = self._duration(min(timeout_sec, 5.0))
        request.ik_request.avoid_collisions = True
        response = self._call(self._ik_client, request, timeout_sec, "/compute_ik")
        solution = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        return (response.error_code.val, solution)

    def follow_joint_trajectory(
        self,
        controller: str,
        joint_names: Sequence[str],
        points: Sequence[Sequence[float]],
        period_sec: float,
    ) -> None:
        goal = self._FollowJointTrajectory.Goal()
        goal.trajectory = self._JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)
        for index, point in enumerate(points):
            waypoint = self._JointTrajectoryPoint()
            waypoint.positions = [float(value) for value in point]
            # The first waypoint sits one period ahead: a point at t=0 asks the
            # controller to already be there.
            waypoint.time_from_start = self._duration((index + 1) * period_sec)
            goal.trajectory.points.append(waypoint)
        self._send_goal(
            self._FollowJointTrajectory,
            f"/{controller}/follow_joint_trajectory",
            goal,
            result_timeout_sec=(len(points) + 1) * period_sec + DEFAULT_TIMEOUT_SEC,
            failure=trajectory_failure,
        )

    def gripper_command(self, controller: str, joint: str, position: float) -> None:
        # ParallelGripperCommand carries a JointState, not a bare position, so
        # the goal must name the joint it is driving.
        goal = self._ParallelGripperCommand.Goal()
        goal.command = self._joint_state({joint: position}, "")
        self._send_goal(
            self._ParallelGripperCommand,
            f"/{controller}/gripper_cmd",
            goal,
            result_timeout_sec=DEFAULT_TIMEOUT_SEC,
            failure=gripper_failure,
        )

    def _send_goal(
        self,
        action_type: Any,
        action_name: str,
        goal: Any,
        result_timeout_sec: float,
        failure: Any,
    ) -> None:
        client = self._ActionClient(self._node, action_type, action_name)
        try:
            if not client.wait_for_server(timeout_sec=DEFAULT_TIMEOUT_SEC):
                raise AdapterUnavailable(
                    f"no action server at {action_name}; is the controller "
                    "spawned and active?"
                )
            handle = self._resolve(
                client.send_goal_async(goal), DEFAULT_TIMEOUT_SEC, action_name
            )
            if not handle.accepted:
                raise AdapterUnavailable(f"{action_name} rejected the goal")
            result = self._resolve(
                handle.get_result_async(), result_timeout_sec, action_name
            )
            reason = failure(result.result)
            if reason is not None:
                raise AdapterUnavailable(f"{action_name} failed: {reason}")
        finally:
            client.destroy()

    def _call(
        self, client: Any, request: Any, timeout_sec: float, name: str
    ) -> Any:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise AdapterUnavailable(
                f"{name} is not available; is move_group running?"
            )
        return self._resolve(client.call_async(request), timeout_sec, name)

    def _resolve(self, future: Any, timeout_sec: float, name: str) -> Any:
        # A synchronous call() from a node nobody spins deadlocks, because the
        # response can only arrive while the executor is running.
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout_sec
        )
        if not future.done():
            raise AdapterUnavailable(f"{name} did not answer within {timeout_sec} s")
        return future.result()

    def _joint_state(self, values: Mapping[str, float], frame_id: str) -> Any:
        return self._JointState(
            header=self._Header(frame_id=frame_id),
            name=list(values),
            position=[float(value) for value in values.values()],
        )

    def _duration(self, seconds: float) -> Any:
        whole = int(seconds)
        return self._Duration(
            sec=whole, nanosec=int(round((seconds - whole) * 1_000_000_000))
        )
