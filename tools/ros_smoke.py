"""Bounded ROS smoke-test helpers.

The validation helpers in this module are deliberately independent of ROS.
Runtime ROS imports are confined to the command-line entry point below.
"""

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
import time


def wait_for(
    predicate: Callable[[], bool],
    timeout_s: float,
    interval_s: float = 0.1,
) -> None:
    """Wait until *predicate* succeeds or raise ``TimeoutError``."""
    deadline = time.monotonic() + timeout_s
    while not predicate():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"condition not met within {timeout_s} seconds")
        time.sleep(min(interval_s, remaining))


def validate_joint_state(
    expected_targets: Mapping[str, float],
    names: Sequence[str],
    positions: Sequence[float],
    tolerance: float,
) -> None:
    """Validate exact joint coverage and proximity to expected positions.

    ``expected_targets`` maps every required joint name to its commanded target
    in radians. ``tolerance`` is the maximum absolute error, also in radians.
    """
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("tolerance must be finite and non-negative")
    if len(names) != len(positions):
        raise ValueError("joint name and position length mismatch")
    if len(set(names)) != len(names):
        raise ValueError("duplicate joint name")

    actual_names = set(names)
    expected_names = set(expected_targets)
    missing = expected_names - actual_names
    extra = actual_names - expected_names
    if missing or extra:
        details = []
        if missing:
            details.append(f"missing joints: {sorted(missing)}")
        if extra:
            details.append(f"extra joints: {sorted(extra)}")
        raise ValueError("; ".join(details))

    for name, target in expected_targets.items():
        if not math.isfinite(target):
            raise ValueError(f"non-finite target for joint {name}")

    actual_by_name = dict(zip(names, positions, strict=True))
    for name, position in actual_by_name.items():
        if not math.isfinite(position):
            raise ValueError(f"non-finite position for joint {name}")
        error = abs(position - expected_targets[name])
        if error > tolerance:
            raise ValueError(
                f"position error for joint {name} is {error}, "
                f"above tolerance {tolerance}"
            )


@dataclass(frozen=True)
class SmokePlan:
    """ROS names and bounded joint targets for one smoke-test mode."""

    command_targets: Mapping[str, float]
    state_targets: Mapping[str, float]
    controller_name: str = "joint_trajectory_controller"
    state_broadcaster_name: str = "joint_state_broadcaster"


@dataclass
class JointStateCapture:
    """Keep only state received since the most recent command publication."""

    latest: tuple[Sequence[str], Sequence[float]] | None = None

    def record(
        self,
        names: Sequence[str],
        positions: Sequence[float],
    ) -> None:
        self.latest = (tuple(names), tuple(positions))

    def publish_requiring_fresh_state(
        self,
        publish: Callable[[object], None],
        message: object,
    ) -> None:
        self.latest = None
        publish(message)


_DG5F_JOINTS = tuple(
    f"rj_dg_{finger}_{joint}"
    for finger in range(1, 6)
    for joint in range(1, 5)
)
_OPENARM_ARM_JOINTS = tuple(f"openarm_joint{joint}" for joint in range(1, 8))
_OPENARM_FINGER_JOINT = "openarm_finger_joint1"
# rj_dg_1_2 is limited to [-pi, 0.0] by the DG5F description, so the shared
# +0.05 target is unreachable there. Simulated joints clamp at their limits
# while fake hardware echoes any command back, so the sign must be flipped to
# keep one plan valid for both.
_DG5F_NEGATIVE_ONLY_JOINTS = ("rj_dg_1_2",)


def smoke_plan(robot: str) -> SmokePlan:
    """Return the fixed, neutral-relative targets for a supported smoke test."""
    if robot == "dg5f":
        targets = {
            joint: -0.05 if joint in _DG5F_NEGATIVE_ONLY_JOINTS else 0.05
            for joint in _DG5F_JOINTS
        }
        return SmokePlan(command_targets=targets, state_targets=targets)
    if robot == "openarm":
        command_targets = dict.fromkeys(_OPENARM_ARM_JOINTS, 0.05)
        state_targets = {**command_targets, _OPENARM_FINGER_JOINT: 0.0}
        return SmokePlan(
            command_targets=command_targets,
            state_targets=state_targets,
        )
    raise ValueError(f"unsupported smoke-test robot: {robot}")


def _run_ros_smoke(
    plan: SmokePlan,
    controller_timeout_s: float,
    state_timeout_s: float,
    tolerance: float,
) -> None:
    # ROS imports remain below the pure helpers so importing this module never
    # initializes ROS or requires a sourced ROS installation.
    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = rclpy.create_node("robot_control_smoke_validator")
    joint_states = JointStateCapture()

    def record_state(message: JointState) -> None:
        joint_states.record(message.name, message.position)

    node.create_subscription(
        JointState,
        "/joint_states",
        record_state,
        qos_profile_sensor_data,
    )
    publisher = node.create_publisher(
        JointTrajectory,
        f"/{plan.controller_name}/joint_trajectory",
        10,
    )
    controller_client = node.create_client(
        ListControllers,
        "/controller_manager/list_controllers",
    )
    controller_future = None

    def controllers_are_active() -> bool:
        nonlocal controller_future
        rclpy.spin_once(node, timeout_sec=0.05)
        if controller_future is None:
            if controller_client.service_is_ready():
                controller_future = controller_client.call_async(
                    ListControllers.Request()
                )
            return False
        if not controller_future.done():
            return False
        response = controller_future.result()
        controller_future = None
        if response is None:
            return False
        states = {
            controller.name: controller.state for controller in response.controller
        }
        return (
            states.get(plan.controller_name) == "active"
            and states.get(plan.state_broadcaster_name) == "active"
        )

    try:
        wait_for(
            controllers_are_active,
            timeout_s=controller_timeout_s,
            interval_s=0,
        )

        def trajectory_subscriber_ready() -> bool:
            rclpy.spin_once(node, timeout_sec=0.05)
            return publisher.get_subscription_count() > 0

        wait_for(
            trajectory_subscriber_ready,
            timeout_s=state_timeout_s,
            interval_s=0,
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(plan.command_targets)
        point = JointTrajectoryPoint()
        point.positions = list(plan.command_targets.values())
        point.time_from_start = Duration(sec=2)
        trajectory.points = [point]
        joint_states.publish_requiring_fresh_state(
            publisher.publish,
            trajectory,
        )

        last_validation_error: ValueError | None = None

        def target_state_reached() -> bool:
            nonlocal last_validation_error
            rclpy.spin_once(node, timeout_sec=0.05)
            latest_state = joint_states.latest
            if latest_state is None:
                return False
            try:
                validate_joint_state(
                    plan.state_targets,
                    latest_state[0],
                    latest_state[1],
                    tolerance,
                )
            except ValueError as error:
                last_validation_error = error
                return False
            return True

        try:
            wait_for(
                target_state_reached,
                timeout_s=state_timeout_s,
                interval_s=0,
            )
        except TimeoutError:
            if last_validation_error is not None:
                raise last_validation_error
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    """Run a bounded ROS state/trajectory smoke check."""
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=("openarm", "dg5f"), required=True)
    parser.add_argument("--controller-timeout", type=float, default=30.0)
    parser.add_argument("--state-timeout", type=float, default=15.0)
    parser.add_argument("--tolerance", type=float, default=0.02)
    arguments = parser.parse_args(argv)

    try:
        _run_ros_smoke(
            smoke_plan(arguments.robot),
            controller_timeout_s=arguments.controller_timeout,
            state_timeout_s=arguments.state_timeout,
            tolerance=arguments.tolerance,
        )
    except (RuntimeError, TimeoutError, ValueError) as error:
        parser.exit(1, f"smoke validation failed: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
