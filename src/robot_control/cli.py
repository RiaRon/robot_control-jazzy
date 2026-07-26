from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from .artifacts import read_hdf5, track_sha256, write_hdf5
from .calibration import load_bundle
from .identification import build_excitation, fit_second_order, validate_holdout
from .interface import CanonicalInterface
from .profile import PARALLEL_GRIPPER_COMMAND, load_builtin_profile
from .safety import CommandGate, SafetyError
from .srdf import named_state, repository_root
from .track import normalize_track


DEFAULT_DURATION_SEC = 3.0

# The arms hold position through the DM motors' impedance control, which needs
# a standing position error to produce holding torque, so a command lands short
# by roughly (gravity torque / kp). --settle closes that gap by re-commanding.
DEFAULT_TOLERANCE_M = 0.005
# Long enough for the arm to stop moving after a torque step before its
# tracking error is read, at the 100 Hz the controller manager runs.
DEFAULT_HOLD_SEC = 2.0
# Refuse a scale beyond this. The model is only as good as the URDF's masses,
# and over-compensating does not mispose the arm, it drives it away from where
# it was holding.
MAX_GRAVITY_SCALE = 1.5
# Following ends on its own rather than running until interrupted: a servo loop
# left running is a robot that moves when someone touches the marker hours later.
DEFAULT_FOLLOW_SEC = 60.0
# How long a streamed command may be ahead of the arm, expressed as travel time
# at the joint's velocity limit. It has to exceed the standing droop or the arm
# cannot advance at all, and stay small enough that a blocked joint does not wind
# up torque: 0.1 s is 0.2 rad at 2 rad/s, an order over the droop measured with
# compensation on, and about 4 N.m at the stiffness the hardware applies.
LEAD_SEC = 0.1
SETTLE_PASSES = 4
# Below this fraction of improvement the loop has stopped converging: the arm
# is against a hard stop, or holding something, and more passes would only wind
# the command further past a target it cannot reach.
SETTLE_PROGRESS = 0.1

# Exit codes, shared with the r2s stages: 2 means the request or the
# environment cannot support the command, 3 means it was understood and
# refused.
UNUSABLE = 2
REFUSED = 3


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotctl")
    commands = parser.add_subparsers(dest="command", required=True)
    r2s = commands.add_parser("r2s")
    stages = r2s.add_subparsers(dest="stage", required=True)
    for stage in ("preflight", "collect", "normalize", "fit", "validate", "export"):
        item = stages.add_parser(stage)
        item.add_argument("--profile", default="openarm_tesollo")
        if stage == "collect":
            mode = item.add_mutually_exclusive_group()
            mode.add_argument("--dry-run", action="store_true")
            mode.add_argument("--execute", action="store_true")
            item.add_argument("--amplitude-scale", type=float, default=0.3)
        if stage == "fit":
            item.add_argument("--population", type=int, default=128)
            item.add_argument("--track", type=Path)
            item.add_argument("--output", type=Path)
        if stage == "normalize":
            item.add_argument("--input", type=Path)
            item.add_argument("--output", type=Path)
        if stage in {"validate", "export"}:
            item.add_argument("--bundle", type=Path)
        if stage == "validate":
            item.add_argument("--metrics", type=Path)
            item.add_argument("--output", type=Path)
        if stage == "export":
            item.add_argument("--validation", type=Path)
            item.add_argument("--output", type=Path)
    _add_pose(commands)
    return parser


def _add_pose(commands: argparse._SubParsersAction) -> None:
    pose = commands.add_parser("pose", help="read and set robot poses")
    stages = pose.add_subparsers(dest="stage", required=True)

    show = stages.add_parser("show", help="report the current pose")
    show.add_argument("--profile", default="openarm_tesollo")
    show.add_argument("--group")

    joints = stages.add_parser("joints", help="set a group by joint values")
    joints.add_argument("--profile", default="openarm_tesollo")
    joints.add_argument("--group", required=True)
    target = joints.add_mutually_exclusive_group(required=True)
    target.add_argument("--values", help="comma-separated canonical values")
    target.add_argument("--named", help="an SRDF group state, such as home")
    joints.add_argument("--duration", type=float, default=DEFAULT_DURATION_SEC)
    joints.add_argument("--execute", action="store_true")

    end_effector = stages.add_parser("ee", help="set a group by end-effector pose")
    end_effector.add_argument("--profile", default="openarm_tesollo")
    end_effector.add_argument("--group", required=True)
    where = end_effector.add_mutually_exclusive_group(required=True)
    where.add_argument("--xyz", help="x,y,z in metres")
    where.add_argument(
        "--from-marker",
        action="store_true",
        help="take the target from the RViz end-effector marker you dragged",
    )
    end_effector.add_argument("--rpy", help="roll,pitch,yaw in radians")
    end_effector.add_argument(
        "--relative",
        action="store_true",
        help="treat --xyz and --rpy as an offset from the current pose",
    )
    end_effector.add_argument("--duration", type=float, default=DEFAULT_DURATION_SEC)
    end_effector.add_argument("--execute", action="store_true")
    end_effector.add_argument(
        "--settle",
        action="store_true",
        help="re-command until the residual falls below --tolerance",
    )
    end_effector.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE_M,
        help="metres of residual --settle aims for",
    )

    gravity = stages.add_parser(
        "gravity", help="publish gravity feedforward torque, and tune its scale"
    )
    gravity.add_argument("--profile", default="openarm_tesollo")
    gravity.add_argument("--group", required=True)
    how = gravity.add_mutually_exclusive_group(required=True)
    how.add_argument(
        "--scale",
        type=float,
        help="fraction of the modelled gravity torque to publish",
    )
    how.add_argument(
        "--sweep",
        help="comma-separated scales to measure in turn, for tuning",
    )
    gravity.add_argument(
        "--hold-sec",
        type=float,
        default=DEFAULT_HOLD_SEC,
        help="seconds to publish at each scale before measuring",
    )
    gravity.add_argument("--execute", action="store_true")

    follow = stages.add_parser(
        "follow", help="follow the RViz end-effector marker continuously"
    )
    follow.add_argument("--profile", default="openarm_tesollo")
    follow.add_argument("--group", required=True)
    follow.add_argument(
        "--gravity",
        type=float,
        default=0.0,
        help="gravity feedforward scale to hold while following",
    )
    follow.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_FOLLOW_SEC,
        help="how long to follow before stopping on its own",
    )
    follow.add_argument("--execute", action="store_true")

    rviz = stages.add_parser("rviz", help="launch the MoveIt stack with RViz")
    rviz.add_argument("--profile", default="openarm_tesollo")
    rviz.add_argument(
        "--real",
        action="store_true",
        help="drive real hardware over CAN instead of fake hardware",
    )
    rviz.add_argument("--right-can", help="CAN interface for the right arm")
    rviz.add_argument("--left-can", help="CAN interface for the left arm")


def _pose(args: argparse.Namespace) -> int:
    """Dispatch a pose stage, mapping every failure onto the exit convention."""
    # Imported here so `robotctl r2s` never pays for it. The module itself is
    # rclpy-free; only using an adapter pulls ROS in.
    from .ros_adapter import AdapterUnavailable, IkFailed

    try:
        if args.stage == "rviz":
            return _pose_rviz(args)
        profile = load_builtin_profile(args.profile)
        if args.stage == "show":
            return _pose_show(args, profile)
        if args.stage == "joints":
            return _pose_joints(args, profile)
        if args.stage == "gravity":
            return _pose_gravity(args, profile)
        if args.stage == "follow":
            return _pose_follow(args, profile)
        return _pose_ee(args, profile)
    except (SafetyError, IkFailed) as error:
        print(f"refused: {error}")
        return REFUSED
    except AdapterUnavailable as error:
        print(f"unavailable: {error}")
        return UNUSABLE
    except (ValueError, OSError) as error:
        # ProfileError, InterfaceError, and SrdfError are all ValueError.
        print(f"error: {error}")
        return UNUSABLE


def _group(profile, name: str):
    if name not in profile.groups:
        raise ValueError(
            f"unknown group {name!r}; known groups are {sorted(profile.groups)}"
        )
    group = profile.groups[name]
    if group.controller is None:
        raise ValueError(f"group {name!r} declares no controller, so it cannot be set")
    return group


def _gate(profile, group, seed: np.ndarray | None) -> CommandGate:
    """Build a gate over one group's profile limits, optionally seeded."""
    joints = {joint.canonical: joint for joint in profile.joints}
    limits = [joints[canonical] for canonical in group.joints]
    gate = CommandGate(
        execute=True,
        lower=np.array([joint.lower for joint in limits]),
        upper=np.array([joint.upper for joint in limits]),
        velocity=np.array([joint.velocity for joint in limits]),
        command_period_sec=1.0 / profile.ros["jazzy"].command_rate_hz,
        effort=np.array([joint.effort for joint in limits]),
        max_lead=np.array([joint.velocity * LEAD_SEC for joint in limits]),
    )
    if seed is not None:
        # Seeding makes the velocity limit apply to the move itself, not just
        # between waypoints of a multi-point plan.
        gate.authorize(seed, now_sec=0.0)
    return gate


def _parse_floats(text: str, label: str) -> list[float]:
    values = []
    for item in text.split(","):
        try:
            values.append(float(item))
        except ValueError:
            raise ValueError(f"{label} is not a number: {item!r}") from None
    return values


def _target_from_args(args, profile, group, interface) -> np.ndarray:
    """Resolve --values or --named into a canonical target for the group."""
    if args.values is not None:
        values = _parse_floats(args.values, "--values")
        if len(values) != len(group.joints):
            raise ValueError(
                f"group {group.name!r} has {len(group.joints)} joints, "
                f"but --values gave {len(values)}"
            )
        return np.asarray(values, dtype=float)

    if group.moveit_group is None:
        raise ValueError(
            f"group {group.name!r} has no planning group, so it has no SRDF "
            "named states; use --values"
        )
    state = named_state(group.moveit_group, args.named)
    return interface.group_state_to_canonical(group.name, state)


def _describe(group, interface, target: np.ndarray) -> None:
    """Print exactly what would go on the wire, in the robot's own names."""
    names = interface.group_source_names(group.name)
    source = interface.group_command_to_source(group.name, target)
    print(f"group: {group.name} -> controller {group.controller} ({group.action})")
    print(f"  {'canonical':<16} {'source joint':<28} {'commanded (rad)':>15}")
    for canonical, name in zip(group.joints, names):
        print(f"  {canonical:<16} {name:<28} {source[name]:>+15.4f}")


def _pose_show(args, profile) -> int:
    from .ros_adapter import AdapterUnavailable, RosAdapter, make_backend

    interface = CanonicalInterface(profile)
    groups = (
        {args.group: _group(profile, args.group)}
        if args.group
        else profile.executable_groups()
    )
    # The static contract is worth printing even with no robot to read.
    for name, group in groups.items():
        planning = group.moveit_group or "-"
        print(f"{name}: controller={group.controller} planning_group={planning}")

    try:
        backend = make_backend()
    except AdapterUnavailable as error:
        print(f"unavailable: {error}")
        return UNUSABLE
    try:
        for name, group in groups.items():
            adapter = RosAdapter(profile, name, execute=False, backend=backend)
            state = adapter.read_state()
            values = " ".join(f"{value:+.4f}" for value in state)
            print(f"{name}: {values}")
            if group.moveit_group is not None and group.tip_link is not None:
                pose = adapter.read_pose()
                xyz = " ".join(f"{value:+.4f}" for value in pose.position)
                rpy = " ".join(f"{value:+.4f}" for value in pose.rpy)
                print(f"{name}: {group.tip_link} xyz [{xyz}] rpy [{rpy}]")
    finally:
        backend.close()
    return 0


def _pose_joints(args, profile) -> int:
    from .ros_adapter import RosAdapter

    group = _group(profile, args.group)
    interface = CanonicalInterface(profile)
    target = _target_from_args(args, profile, group, interface)

    if not args.execute:
        # A dry run stays entirely offline, so it can never reach the robot.
        _gate(profile, group, seed=None).authorize_trajectory(
            [target], start_time_sec=0.0, period_sec=args.duration
        )
        print(f"DRY RUN: would send over {args.duration:g} s; pass --execute to send")
        _describe(group, interface, target)
        return 0

    with RosAdapter(profile, args.group, execute=True) as adapter:
        gate = _gate(profile, group, seed=adapter.read_state())
        points = gate.authorize_trajectory(
            [target], start_time_sec=0.0, period_sec=args.duration
        )
        _describe(group, interface, target)
        if group.action == PARALLEL_GRIPPER_COMMAND:
            adapter.send_gripper(float(points[-1][0]))
        else:
            adapter.send_trajectory(points, period_sec=args.duration)
        print(f"EXECUTED: {group.name} over {args.duration:g} s")
    return 0


def _pose_ee(args, profile) -> int:
    from .ros_adapter import Pose, RosAdapter, quaternion_from_rpy

    group = _group(profile, args.group)
    if group.moveit_group is None or group.tip_link is None:
        raise ValueError(
            f"group {group.name!r} has no planning group, so it has no "
            "end-effector pose; set it with pose joints --values instead"
        )
    interface = CanonicalInterface(profile)
    if args.settle and not args.execute:
        raise ValueError(
            "--settle corrects what a move actually reached, so it needs "
            "--execute; a dry run sends nothing to fall short of"
        )
    xyz, rpy = None, None
    if args.from_marker:
        # The marker carries a full pose already, so anything that modifies a
        # typed one would move the arm somewhere the operator never saw.
        for name, given in (("--relative", args.relative), ("--rpy", args.rpy)):
            if given:
                raise ValueError(f"{name} cannot be combined with --from-marker")
    else:
        xyz = _parse_floats(args.xyz, "--xyz")
        if len(xyz) != 3:
            raise ValueError(f"--xyz needs exactly three values, got {len(xyz)}")
        if args.rpy is not None:
            rpy = _parse_floats(args.rpy, "--rpy")
            if len(rpy) != 3:
                raise ValueError(f"--rpy needs exactly three values, got {len(rpy)}")

    # Even a dry run needs move_group: IK is a service, with no offline form.
    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        current = adapter.read_pose()
        seed = adapter.read_state()
        if args.from_marker:
            target = adapter.read_marker_pose()
        elif args.relative:
            target = current.translated(xyz)
            if rpy is not None:
                roll, pitch, yaw = (a + b for a, b in zip(current.rpy, rpy))
                target = target.rotated_to(quaternion_from_rpy(roll, pitch, yaw))
        else:
            orientation = (
                current.orientation if rpy is None else quaternion_from_rpy(*rpy)
            )
            target = Pose(tuple(xyz), orientation, current.frame_id)

        solution = adapter.solve_ik(target, seed=seed)
        gate = _gate(profile, group, seed=seed)
        points = gate.authorize_trajectory(
            [solution], start_time_sec=0.0, period_sec=args.duration
        )

        start = " ".join(f"{value:+.4f}" for value in current.position)
        goal = " ".join(f"{value:+.4f}" for value in target.position)
        print(f"{group.tip_link}: [{start}] -> [{goal}] in {target.frame_id}")
        _describe(group, interface, solution)
        if not args.execute:
            print("DRY RUN: solved but not sent; pass --execute to send")
            return 0
        adapter.send_trajectory(points, period_sec=args.duration)
        print(f"EXECUTED: {group.name} over {args.duration:g} s")
        _report_residual(adapter, target, solution, profile, group, args)
    return 0


def _residual(adapter, target) -> float:
    """Metres between where the tool centre point is and where it was sent."""
    landed = adapter.read_pose()
    return float(
        np.linalg.norm(np.asarray(landed.position) - np.asarray(target.position))
    )


def _report_residual(adapter, target, solution, profile, group, args) -> None:
    """Print how far short the move stopped, and with --settle, close the gap.

    The arms hold position through impedance control with no gravity feedforward,
    so a joint only produces holding torque while it sits short of its command.
    Re-sending the same solution therefore reproduces the same shortfall exactly;
    each pass has to command past the target by what the last one missed.
    """
    residual = _residual(adapter, target)
    if not args.settle:
        print(f"residual: {residual * 1000:.1f} mm from the commanded pose")
        return

    command = np.asarray(solution, dtype=float)
    for attempt in range(1, SETTLE_PASSES + 1):
        if residual <= args.tolerance:
            print(f"settled: {residual * 1000:.1f} mm after {attempt - 1} corrections")
            return
        actual = adapter.read_state()
        command = command + (solution - actual)
        # A fresh gate each pass, so the wound-up command is checked against the
        # profile limits rather than trusted for having been safe once.
        gate = _gate(profile, group, seed=actual)
        points = gate.authorize_trajectory(
            [command], start_time_sec=0.0, period_sec=args.duration
        )
        adapter.send_trajectory(points, period_sec=args.duration)
        corrected = _residual(adapter, target)
        print(f"settle {attempt}: {residual * 1000:.1f} -> {corrected * 1000:.1f} mm")
        if corrected > residual * (1.0 - SETTLE_PROGRESS):
            print(
                f"settle: stopped converging at {corrected * 1000:.1f} mm; the arm "
                "is against a limit, holding a load, or the target is unreachable"
            )
            return
        residual = corrected

    print(f"settle: {residual * 1000:.1f} mm after {SETTLE_PASSES} corrections")


def _gravity_chain(adapter, profile, group):
    """Build the kinematic chain for *group* from the running stack's URDF."""
    from .kinematics import chain_from_urdf

    source_by_canonical = {joint.canonical: joint.source for joint in profile.joints}
    return chain_from_urdf(
        adapter.read_robot_description(),
        [source_by_canonical[canonical] for canonical in group.joints],
        group.tip_link,
    )


def _pose_gravity(args, profile) -> int:
    """Publish gravity feedforward, at one scale or measured across several.

    The model is only as good as the URDF's masses, and the gains it works
    against are hard-coded in the vendor hardware rather than configured, so the
    right scale is a measured quantity rather than a derived one. --sweep exists
    to measure it: hold at each scale, read the controller's own tracking error,
    and print what actually happened.
    """
    from .ros_adapter import RosAdapter

    group = _group(profile, args.group)
    if not group.compensable:
        raise ValueError(
            f"group {group.name!r} declares no effort_controller, so torque "
            "cannot be published for it"
        )
    if group.tip_link is None:
        raise ValueError(
            f"group {group.name!r} has no tip_link, so its chain cannot be built"
        )
    scales = (
        [args.scale] if args.sweep is None else _parse_floats(args.sweep, "--sweep")
    )
    for scale in scales:
        if not 0.0 <= scale <= MAX_GRAVITY_SCALE:
            raise ValueError(
                f"gravity scale {scale:g} is outside 0 to {MAX_GRAVITY_SCALE:g}; "
                "over-compensating drives the arm away from where it was holding"
            )
    if args.hold_sec <= 0:
        raise ValueError("--hold-sec must be positive")

    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group)
        state = adapter.read_state()
        modelled = chain.gravity_torque(state)
        gate = _gate(profile, group, seed=None)

        print(f"{group.name}: {len(chain)} joints, "
              f"{sum(link.mass for link in chain.links):.3f} kg modelled")
        _describe_torque(group, modelled)
        if not args.execute:
            print("DRY RUN: torque computed but not published; pass --execute")
            return 0

        try:
            rows = []
            for scale in scales:
                # Recomputed each round: compensation moves the arm, and the
                # torque that holds it depends on where it now is.
                state = adapter.read_state()
                effort = gate.authorize_effort(chain.gravity_torque(state) * scale)
                _publish_for(adapter, effort, args.hold_sec)
                error = adapter.read_tracking_error()
                rows.append((scale, error))
                print(
                    f"scale {scale:4.2f}: worst joint error "
                    f"{np.max(np.abs(error)):+.4f} rad, "
                    f"mean {np.mean(np.abs(error)):.4f} rad"
                )
            if len(rows) > 1:
                _report_sweep(group, rows)
        finally:
            # Torque left applied after this process exits would keep pushing.
            adapter.send_effort(np.zeros(len(group.joints)))
            print("torque released")
    return 0


def _publish_for(adapter, effort, seconds: float) -> None:
    """Republish *effort* at the controller rate for *seconds*.

    ForwardCommandController holds its last command, so one message would do,
    but republishing means a dropped message cannot silently leave the arm on a
    stale torque.
    """
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        adapter.send_effort(effort)
        time.sleep(0.01)


def _describe_torque(group, torque) -> None:
    print("  joint            modelled gravity torque (N.m)")
    for canonical, value in zip(group.joints, torque):
        print(f"  {canonical:<16} {value:+8.2f}")


def _report_sweep(group, rows) -> None:
    """Print the sweep as a table, and name the scale that measured best."""
    print()
    header = "  scale  " + "".join(f"{canonical:>10}" for canonical in group.joints)
    print(header)
    for scale, error in rows:
        print(f"  {scale:5.2f}  " + "".join(f"{value:+10.4f}" for value in error))
    best = min(rows, key=lambda row: float(np.max(np.abs(row[1]))))
    print()
    print(
        f"best measured scale: {best[0]:g} "
        f"(worst joint {np.max(np.abs(best[1])):+.4f} rad)"
    )
    print("re-run with --scale to hold there, and record it in the profile notes")


def _pose_follow(args, profile) -> int:
    """Track the dragged marker continuously, at the controller rate.

    Differential inverse kinematics rather than /compute_ik: a service round trip
    per sample cannot keep up, and the Jacobian gives a step that is smooth and
    local, so the arm sweeps to a nearby solution instead of jumping between
    branches the way a fresh IK solve can.

    Every sample is clamped by the gate rather than refused, since dragging
    faster than the arm can move is normal operation, not an error.
    """
    from .ros_adapter import RosAdapter

    group = _group(profile, args.group)
    if group.moveit_group is None or group.tip_link is None:
        raise ValueError(
            f"group {group.name!r} has no planning group, so it has no "
            "end-effector marker to follow"
        )
    if not 0.0 <= args.gravity <= MAX_GRAVITY_SCALE:
        raise ValueError(
            f"gravity scale {args.gravity:g} is outside 0 to {MAX_GRAVITY_SCALE:g}"
        )
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")

    period = 1.0 / profile.ros["jazzy"].command_rate_hz
    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group)
        gate = _gate(profile, group, seed=None)
        adapter.watch_marker()
        state = adapter.read_state()
        print(
            f"following {group.tip_link} at {1.0 / period:g} Hz for "
            f"{args.seconds:g} s, gravity scale {args.gravity:g}"
        )
        print("drag the marker in RViz; the arm tracks it until the time runs out")
        if not args.execute:
            print("DRY RUN: nothing is published; pass --execute to follow")
            return 0
        _follow_loop(adapter, chain, gate, group, state, period, args)
    return 0


def _follow_loop(adapter, chain, gate, group, state, period, args) -> None:
    from .kinematics import twist_between

    samples = 0
    notes: dict[str, int] = {}
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            cycle = time.monotonic()
            adapter.pump(timeout_sec=0.0)
            target = adapter.latest_marker_target()
            state = adapter.read_state(timeout_sec=1.0)
            if args.gravity > 0.0:
                adapter.send_effort(
                    gate.authorize_effort(chain.gravity_torque(state) * args.gravity)
                )
            if target is not None:
                goal = np.eye(4)
                goal[:3, 3] = target.position
                goal[:3, :3] = _rotation_from_quaternion(target.orientation)
                step = chain.delta_q(state, twist_between(chain.pose(state), goal))
                command, limited = gate.follow(state + step, state, period)
                if limited is not None:
                    notes[limited] = notes.get(limited, 0) + 1
                adapter.stream_positions(command, period_sec=period)
                samples += 1
            time.sleep(max(0.0, period - (time.monotonic() - cycle)))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Stop commanding, and stop pushing. The trajectory controller holds its
        # last position, which is where the arm already is, so it stays put.
        if args.gravity > 0.0:
            adapter.send_effort(np.zeros(len(group.joints)))
        print(f"followed {samples} samples; the arm holds its last commanded pose")
        for note, count in sorted(notes.items()):
            print(f"  {note} clamped on {count} of {samples} samples")


def _rotation_from_quaternion(orientation) -> np.ndarray:
    """Return the 3x3 rotation of a quaternion given in x, y, z, w order."""
    x, y, z, w = (float(value) for value in orientation)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _pose_rviz(args) -> int:
    script = repository_root() / "ros_ws/pose_bringup.sh"
    if not script.is_file():
        raise ValueError(f"bringup wrapper not found: {script}")
    command = [str(script)]
    if args.real:
        command.append("--real")
        for flag, value in (("--right-can", args.right_can), ("--left-can", args.left_can)):
            if value:
                command += [flag, value]
    print(f"launching: {' '.join(command)}")
    return subprocess.call(command)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "pose":
        return _pose(args)
    profile = load_builtin_profile(args.profile)
    if args.stage == "preflight":
        print(f"profile: {profile.name}")
        print(f"asset: {profile.asset_id}")
        print(f"joints: {len(profile.joints)}")
        print("publish_enabled: false")
    elif args.stage == "collect":
        if args.amplitude_scale <= 0 or args.amplitude_scale > 1:
            raise SystemExit("--amplitude-scale must be in (0, 1]")
        mode = "EXECUTE" if args.execute else "DRY RUN"
        neutral = np.array([(joint.lower + joint.upper) / 2 for joint in profile.joints])
        amplitude = np.array(
            [(joint.upper - joint.lower) * 0.05 * args.amplitude_scale for joint in profile.joints]
        )
        time, command, phases = build_excitation(neutral, amplitude, profile.ros["jazzy"].command_rate_hz)
        print(
            f"{mode}: profile={profile.name} amplitude_scale={args.amplitude_scale} "
            f"samples={len(time)} phases={','.join(dict.fromkeys(phases))}"
        )
        if args.execute:
            print("ROS publisher backend is required; no command was published")
            return 2
    elif args.stage == "normalize":
        if not args.input or not args.output:
            raise SystemExit("--input and --output are required")
        raw = np.load(args.input, allow_pickle=False)
        track = normalize_track(
            raw["command_time_ns"],
            raw["command"],
            raw["measured_time_ns"],
            raw["measured"],
            list(raw["joint_names"]),
            profile.ros["jazzy"].command_rate_hz,
        )
        write_hdf5(args.output, track)
        print(f"normalize: {args.output} sha256={track_sha256(track)}")
    elif args.stage == "fit":
        if not args.track or not args.output:
            raise SystemExit("--track and --output are required")
        if args.population <= 0:
            raise SystemExit("--population must be positive")
        track = read_hdf5(args.track)
        estimate = fit_second_order(track.timestamps_ns * 1e-9, track.command, track.measured)
        args.output.write_text(
            json.dumps(
                {
                    "population": args.population,
                    "joint_names": track.joint_names,
                    "stiffness": estimate.stiffness.tolist(),
                    "damping": estimate.damping.tolist(),
                    "friction": estimate.friction.tolist(),
                    "residual_rmse": estimate.residual_rmse.tolist(),
                    "track_sha256": track_sha256(track),
                },
                indent=2,
            )
            + "\n"
        )
        print(f"fit: {args.output}")
    elif args.stage == "validate":
        if not args.bundle:
            raise SystemExit("--bundle is required")
        bundle = load_bundle(args.bundle, profile)
        if not args.metrics or not args.output:
            raise SystemExit("--metrics and --output are required")
        metrics = json.loads(args.metrics.read_text())
        result = validate_holdout(**metrics)
        args.output.write_text(
            json.dumps({"status": result.status, "failures": result.failures}, indent=2) + "\n"
        )
        print(f"validate: schema v{bundle.schema_version}, status={result.status}")
        return 0 if result.status == "validated" else 3
    elif args.stage == "export":
        if not args.bundle or not args.validation or not args.output:
            raise SystemExit("--bundle, --validation, and --output are required")
        load_bundle(args.bundle, profile)
        validation = json.loads(args.validation.read_text())
        if validation.get("status") != "validated":
            print("export blocked: model_inadequate")
            return 3
        args.output.write_bytes(args.bundle.read_bytes())
        print(f"export: {args.output}")
    else:
        print(f"{args.stage}: profile={profile.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
