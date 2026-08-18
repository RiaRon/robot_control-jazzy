from __future__ import annotations

import argparse
from collections.abc import Sequence
import dataclasses
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

from .artifacts import (
    ArtifactError,
    nan_to_null,
    read_hdf5,
    read_static_estimate,
    read_sweep,
    sweep_sha256,
    track_sha256,
    write_hdf5,
    write_static_estimate,
    write_sweep,
)
from .calibration import (
    CalibrationError,
    identified_block,
    load_bundle,
    write_bundle,
)
from .diagnostic_profile import (
    AXES as DIAGNOSTIC_AXES,
    DEFAULT_ANGLE_RAD as DEFAULT_DIAGNOSTIC_ANGLE_RAD,
    DEFAULT_ANGULAR_SPEED_RAD_S as DEFAULT_DIAGNOSTIC_ANGULAR_SPEED_RAD_S,
    DEFAULT_DISTANCE_M as DEFAULT_DIAGNOSTIC_DISTANCE_M,
    DEFAULT_HOLD_SEC as DEFAULT_DIAGNOSTIC_HOLD_SEC,
    DEFAULT_LINEAR_SPEED_M_S as DEFAULT_DIAGNOSTIC_LINEAR_SPEED_M_S,
    DEFAULT_REPETITIONS as DEFAULT_DIAGNOSTIC_REPETITIONS,
    PROFILE_KINDS as DIAGNOSTIC_PROFILE_KINDS,
    DiagnosticProfile,
)
from .identification import (
    DEFAULT_NOISE_RAD,
    MAX_CONDITION,
    MAX_INERTIA_DISAGREEMENT,
    FitError,
    GravitySweep,
    CombinedEstimate,
    SecondOrderEstimate,
    build_excitation,
    combine,
    design_pose_set,
    fit_second_order,
    fit_second_order_runs,
    fit_staircase,
    fit_static_gravity,
    score_holdout,
    split_repetitions,
    validate_holdout,
)
from .excitation import (
    MIN_STAIRCASE_STEPS,
    MOTION_NOISE_MARGIN,
    measure_staircase,
)
from .hdgp_export import (
    DEFAULT_MAX_SPREAD,
    HdgpExportError,
    write_hdgp_calibration,
)
from .interface import CanonicalInterface
from .ik_follow import LatestIkWorker
from .profile import PARALLEL_GRIPPER_COMMAND, load_builtin_profile
from .safety import CommandGate, SafetyError
from .servo import CartesianPI
from .srdf import named_state, repository_root
from .track import DEFAULT_MAX_GAP_PERIODS, TrackError, normalize_track


# Seconds a commanded repositioning takes by default. Ten rather than three
# because the arm outran its operator at three: a pose change of about 1.5 rad
# — an ordinary step between identification poses — went by at 0.5 rad/s, which
# is faster than anyone reaches an estop. This is the bound that fits, and the
# joint velocity limit is not: the excitation is a small fast dither whose peak
# slew a cap low enough to slow a large move would refuse outright.
DEFAULT_DURATION_SEC = 10.0

# How far outside the profile a *measured* pose may sit and still be adopted,
# clamped into bounds, as a move's start. The measured pose is evidence, not a
# command: an arm at rest sits ON its lower stop, and impedance droop reads a
# hair past it, so refusing to move because the seed is 3 mrad outside a bound
# the arm is resting against would make the rest pose unrecoverable. Beyond
# this slack the pose is not droop, it is a wrong profile or a wrong arm, and
# the gate refuses it by name.
SEED_SLACK_RAD = 0.05

# The initial working state `pose ready` moves each arm to: elbow raised here,
# wrist pitched READY_WRIST_RAD away from the table, every other joint at zero.
# It goes in two phases because a tool resting on the table blocks the wrist's
# sweep entirely — commanding the wrist while the arm lies down is a stall, not
# a move, and it burned left j7's rotor. Phase 1 raises only the elbow with
# every other joint holding where it measured; phase 2, airborne, settles the
# rest. `pose rest` reverses the ritual and leaves the wrist at its lifted
# angle, so the tool meets the table yielding in the direction the table
# pushes rather than commanded straight against it.
READY_ELBOW_RAD = 0.8
READY_WRIST_RAD = 0.524  # 30 degrees
# joints[3] is *_aj_4, the elbow, and joints[6] is *_aj_7, the wrist pitch, in
# each arm group's serial order.
ELBOW_INDEX = 3
WRIST_INDEX = 6
# Slow enough to walk to the E-stop mid-move: the full raise takes 8 s.
PARK_SPEED_RAD_PER_SEC = 0.1

# The arms hold position through the DM motors' impedance control, which needs
# a standing position error to produce holding torque, so a command lands short
# by roughly (gravity torque / kp). --settle closes that gap by re-commanding.
DEFAULT_TOLERANCE_M = 0.005
# Long enough for the arm to stop moving after a torque step before its
# tracking error is read, at the 100 Hz the controller manager runs.
DEFAULT_HOLD_SEC = 2.0
# Deflection each joint is pushed to at the ends of its torque staircase. The
# torque that produces it is probed rather than fixed, since the joint's
# stiffness — what this stage measures — is the unknown that sets it.
DEFAULT_DEFLECTION_RAD = 0.05
# Refuse a scale beyond this. The model is only as good as the URDF's masses,
# and over-compensating does not mispose the arm, it drives it away from where
# it was holding.
MAX_GRAVITY_SCALE = 1.5
# Following ends on its own rather than running until interrupted: a servo loop
# left running is a robot that moves when someone touches the marker hours later.
DEFAULT_FOLLOW_SEC = 60.0
DEFAULT_FOLLOW_KP = 2.0
DEFAULT_FOLLOW_KI = 1.0
# 손끝 위치가 목표에서 2mm 이내이면 위치가 도착한 것으로 판단한다.
DEFAULT_FOLLOW_TOLERANCE_M = 0.002

# 손끝 방향이 목표에서 약 2도 이내이면 방향이 도착한 것으로 판단한다.
DEFAULT_FOLLOW_ORIENTATION_TOLERANCE_RAD = 0.035

# 손끝의 최대 직선 이동속도이다.
DEFAULT_MAX_TCP_SPEED_M_S = 0.05

# 손끝의 최대 회전속도이다.
# 0.20 rad/s는 약 11.5 deg/s이다.
DEFAULT_MAX_TCP_ANGULAR_SPEED_RAD_S = 0.20

# IK가 한 번에 계산할 Cartesian 위치 중간목표의 최대 거리이다.
# 먼 목표도 실제 TCP에서 최대 2cm 앞까지만 IK에 전달한다.
DEFAULT_MAX_IK_STEP_M = 0.02

# IK가 한 번에 계산할 방향 중간목표의 최대 회전각이다.
# 0.10 rad는 약 5.7도이다.
DEFAULT_MAX_IK_ANGULAR_STEP_RAD = 0.10

# 중력보상을 적용한 뒤 초기 정렬 완료를 판단하기 전까지 기다린다.
DEFAULT_STARTUP_SETTLE_SEC = 2.0

# 시작할 때 실제 TCP와 RViz 마커 사이에 허용할 최대 거리이다.
DEFAULT_MAX_START_DISTANCE_M = 0.10

# 시작할 때 실제 TCP와 RViz 마커 사이에 허용할 최대 방향 차이이다.
# 0.35 rad는 약 20도이다.
DEFAULT_MAX_START_ANGLE_RAD = 0.35

# Diagnostics only: this threshold records an event and never changes a target.
DEFAULT_IK_TARGET_JUMP_THRESHOLD_RAD = 0.10

# How long a streamed command may be ahead of the arm, expressed as travel time
# at the joint's velocity limit. It has to exceed the standing droop or the arm
# cannot advance at all, and stay small enough that a blocked joint does not wind
# up torque: 0.1 s is 0.2 rad at 2 rad/s, an order over the droop measured with
# compensation on, and about 4 N.m at the stiffness the hardware applies.
LEAD_SEC = 0.1
SETTLE_PASSES = 4
# Enough to condition the fit with a pose to spare: one pose is one equation in
# three unknowns, the second separates them, and the rest buy redundancy against
# measurement noise.
DEFAULT_POSES = 4
# Spanning zero so every pose sees the joint both uncompensated and over-
# compensated, which is what puts a slope through the samples.
DEFAULT_COLLECT_SCALES = "0,0.5,1.0"
# Half of each joint's range, about its middle. A pose against a hard stop cannot
# droop, and a joint that cannot droop looks exactly like one held by stiction.
DEFAULT_REACH = 0.5
# Two to fit and one held out, which is what split_repetitions has always
# required. Two runs would leave one of each, and a model fitted on one run has
# nothing to be validated against.
REPETITIONS = 3
# Below this fraction of improvement the loop has stopped converging: the arm
# is against a hard stop, or holding something, and more passes would only wind
# the command further past a target it cannot reach.
SETTLE_PROGRESS = 0.1

# Exit codes, shared with the r2s stages: 2 means the request or the
# environment cannot support the command, 3 means it was understood and
# refused.
UNUSABLE = 2
REFUSED = 3


class Refused(RuntimeError):
    """Understood, measured, and declined — the exit-3 half of the convention.

    Distinct from a ValueError, which means the request itself could not be
    carried out. An under-conditioned pose set is a well-formed request whose
    answer is no.
    """


# Shared by every command that builds a gravity chain against the live stack.
# The bringup description carries the vendor's masses, so a mounted hand needs
# the generated asset URDF handed in instead; either naming convention reads.
_URDF_OVERRIDE_HELP = (
    "URDF file for the gravity model, instead of the running stack's "
    "description — the generated asset URDF (canonical names, hand masses "
    "included) is the usual override"
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotctl")
    commands = parser.add_subparsers(dest="command", required=True)
    r2s = commands.add_parser("r2s")
    stages = r2s.add_subparsers(dest="stage", required=True)
    for stage in (
        "preflight",
        "collect",
        "normalize",
        "fit",
        "identify",
        "bundle",
        "validate",
        "export",
    ):
        item = stages.add_parser(stage)
        item.add_argument("--profile", default="openarm_tesollo")
        if stage == "identify":
            item.add_argument(
                "--sweep",
                type=Path,
                action="append",
                help="a file written by pose gravity --output; pass it once per "
                "pose, since one pose cannot separate stiffness from the "
                "torque model",
            )
            item.add_argument("--output", type=Path)
            item.add_argument(
                "--noise",
                type=float,
                default=DEFAULT_NOISE_RAD,
                help="radians below which a joint counts as not having moved, "
                "so its samples are dropped as frozen by stiction",
            )
            item.add_argument(
                "--collect",
                action="store_true",
                help="design a pose set and sweep at each pose, instead of only "
                "fitting files that already exist",
            )
            item.add_argument("--group", help="group to collect on; needs --collect")
            item.add_argument(
                "--sweep-dir",
                type=Path,
                help="directory to write one sweep file per collected pose",
            )
            item.add_argument(
                "--poses", type=int, default=DEFAULT_POSES, help="poses to design"
            )
            item.add_argument(
                "--scales",
                default=DEFAULT_COLLECT_SCALES,
                help="comma-separated gravity scales to hold at every pose",
            )
            item.add_argument(
                "--reach",
                type=float,
                default=DEFAULT_REACH,
                help="fraction of each joint's range the poses may use, about "
                "its middle; keeps the set off the hard stops",
            )
            item.add_argument(
                "--seed",
                type=int,
                default=0,
                help="pose-set seed; the same seed designs the same poses, which "
                "is what makes a dry run a review of the run",
            )
            item.add_argument(
                "--duration",
                type=float,
                default=DEFAULT_DURATION_SEC,
                help="seconds to take moving between poses",
            )
            item.add_argument(
                "--hold-sec",
                type=float,
                default=DEFAULT_HOLD_SEC,
                help="seconds to publish at each scale before measuring",
            )
            item.add_argument(
                "--urdf", type=Path, help=_URDF_OVERRIDE_HELP
            )
            item.add_argument(
                "--execute",
                action="store_true",
                help="move the arm through the designed poses; without it the "
                "itinerary is only designed and printed",
            )
        if stage == "collect":
            mode = item.add_mutually_exclusive_group()
            mode.add_argument("--dry-run", action="store_true")
            mode.add_argument("--execute", action="store_true")
            item.add_argument("--amplitude-scale", type=float, default=0.3)
            item.add_argument(
                "--group", help="group to excite; needs a trajectory controller"
            )
            item.add_argument(
                "--output", type=Path, help="`.npz` recording to write"
            )
            item.add_argument(
                "--repetitions",
                type=int,
                default=1,
                help="run the same excitation this many times; 3 writes a "
                "manifest naming two to fit and one to hold out",
            )
        if stage == "fit":
            item.add_argument("--population", type=int, default=128)
            item.add_argument("--track", type=Path)
            item.add_argument("--output", type=Path)
            item.add_argument(
                "--static",
                type=Path,
                help="a stiffness set from r2s identify; adds the gravity term "
                "and turns the fit's ratios into physical parameters",
            )
            item.add_argument(
                "--urdf",
                type=Path,
                help="robot description to compute the modelled torque along the "
                "track; required with --static",
            )
            item.add_argument(
                "--manifest",
                type=Path,
                help="run manifest from r2s collect --repetitions 3; fits across "
                "the runs it names, instead of one --track",
            )
            item.add_argument(
                "--accept-inertia-gap",
                action="store_true",
                help="write the fit even when the two routes to the inertia "
                "disagree. Right when the URDF's masses are only approximate: "
                "kp/k rests on the staircase's measured kp and stands, while "
                "1/g inherits the model's error. The gap is recorded either way",
            )
        if stage == "normalize":
            item.add_argument("--input", type=Path)
            item.add_argument("--output", type=Path)
            item.add_argument(
                "--max-gap-periods",
                type=float,
                default=DEFAULT_MAX_GAP_PERIODS,
                help="command periods a stream may skip before the hole counts "
                "as missing data rather than jitter",
            )
        if stage == "bundle":
            item.add_argument(
                "--base", type=Path, help="schema v2 bundle to merge parameters into"
            )
            item.add_argument(
                "--fit",
                type=Path,
                action="append",
                help="output of r2s fit --static; pass it once per group",
            )
            item.add_argument("--output", type=Path)
        if stage in {"validate", "export"}:
            item.add_argument("--bundle", type=Path)
        if stage == "validate":
            item.add_argument("--metrics", type=Path)
            item.add_argument("--output", type=Path)
            item.add_argument(
                "--manifest",
                type=Path,
                help="run manifest from r2s collect --repetitions 3; scores the "
                "held-out run instead of reading --metrics",
            )
            item.add_argument(
                "--fit",
                type=Path,
                help="fit estimate to score against the holdout; needs --manifest",
            )
            item.add_argument(
                "--urdf",
                type=Path,
                help="robot description, required when the fit carries a "
                "gravity term",
            )
        if stage == "export":
            item.add_argument("--validation", type=Path)
            item.add_argument("--output", type=Path)
            item.add_argument(
                "--hdgp",
                type=Path,
                help="also write the schema v1 file hdgp's training env loads, "
                "one scalar per actuator group under the name that env uses",
            )
            item.add_argument(
                "--hdgp-max-spread",
                type=float,
                default=DEFAULT_MAX_SPREAD,
                help="how far a group's joints may disagree, as (max-min)/mean, "
                "before one scalar is refused as describing none of them",
            )
    _add_pose(commands)
    return parser


def _add_pose(commands: argparse._SubParsersAction) -> None:
    pose = commands.add_parser("pose", help="read and set robot poses")
    stages = pose.add_subparsers(dest="stage", required=True)

    show = stages.add_parser("show", help="report the current pose")
    show.add_argument("--profile", default="openarm_tesollo")
    show.add_argument("--group")
    show.add_argument(
        "--output",
        type=Path,
        help="write the measured joints and end-effector pose as JSON",
    )

    joints = stages.add_parser("joints", help="set a group by joint values")
    joints.add_argument("--profile", default="openarm_tesollo")
    joints.add_argument("--group", required=True)
    target = joints.add_mutually_exclusive_group(required=True)
    target.add_argument("--values", help="comma-separated canonical values")
    target.add_argument("--named", help="an SRDF group state, such as home")
    joints.add_argument("--duration", type=float, default=DEFAULT_DURATION_SEC)
    joints.add_argument("--execute", action="store_true")

    # ready/rest are the bringup and shutdown moves: from wherever each arm is
    # to the initial working state (elbow at READY_ELBOW_RAD, the rest at
    # zero), and back down. Paced by PARK_SPEED_RAD_PER_SEC, not --duration, so
    # they are slow no matter how far the arm has to travel.
    for name, blurb in (
        ("ready", "raise each arm to the initial working state, slowly"),
        ("rest", "lower each arm back to all zeros, slowly"),
    ):
        park = stages.add_parser(name, help=blurb)
        park.add_argument("--profile", default="openarm_tesollo")
        park.add_argument(
            "--group",
            action="append",
            help="an arm group to move; repeatable. Default: every *_arm group",
        )
        park.add_argument("--execute", action="store_true")

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
    gravity.add_argument(
        "--scale",
        help="fraction of the modelled torque to publish: one value, or one "
        "per joint",
    )
    gravity.add_argument(
        "--sweep",
        help="comma-separated scales to measure in turn, for tuning",
    )
    gravity.add_argument(
        "--sweep-joint",
        help="canonical joint whose scale --sweep varies, holding the rest at "
        "--scale",
    )
    gravity.add_argument(
        "--output",
        type=Path,
        help="write what the run measured, for r2s identify to fit",
    )
    gravity.add_argument(
        "--hold-sec",
        type=float,
        default=DEFAULT_HOLD_SEC,
        help="seconds to publish at each scale before measuring",
    )
    gravity.add_argument("--urdf", type=Path, help=_URDF_OVERRIDE_HELP)
    gravity.add_argument("--execute", action="store_true")

    torque = stages.add_parser(
        "torque",
        help="push each joint with a torque of our own and measure the response",
    )
    torque.add_argument("--profile", default="openarm_tesollo")
    torque.add_argument("--group", required=True)
    torque.add_argument(
        "--deflection",
        type=float,
        default=DEFAULT_DEFLECTION_RAD,
        help="radians to push each joint at the ends of its staircase; the "
        "torque that produces it is probed, since stiffness is the unknown",
    )
    torque.add_argument("--steps", type=int, default=7)
    torque.add_argument(
        "--joint",
        action="append",
        help="canonical joint to excite; repeatable. Default: every joint",
    )
    torque.add_argument("--hold-sec", type=float, default=DEFAULT_HOLD_SEC)
    torque.add_argument(
        "--noise",
        type=float,
        default=DEFAULT_NOISE_RAD,
        # Not the threshold itself, unlike identify's flag of the same name:
        # the escalation needs the reading to clear the noise by a margin, and
        # saying "below which a joint counts as not having moved" here would
        # understate what the seed has to achieve by that factor.
        help=f"the encoder's own noise, in radians. A joint counts as having "
        f"moved when its reading changes by {MOTION_NOISE_MARGIN:g}x this, and "
        "the seed doubles until it does, so a coarser encoder needs it raised",
    )
    torque.add_argument("--output", type=Path)
    torque.add_argument("--urdf", type=Path, help=_URDF_OVERRIDE_HELP)
    torque.add_argument("--execute", action="store_true")

    follow = stages.add_parser(
        "follow", help="follow the RViz end-effector marker continuously"
    )
    follow.add_argument("--profile", default="openarm_tesollo")
    follow.add_argument("--group", required=True)
    follow.add_argument(
        "--gravity",
        help="gravity feedforward scale to hold while following: one value, or "
        "one per joint",
    )
    follow.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_FOLLOW_SEC,
        help="how long to follow before stopping on its own",
    )
    follow.add_argument("--urdf", type=Path, help=_URDF_OVERRIDE_HELP)
    follow.add_argument(
        "--kp",
        type=float,
        default=DEFAULT_FOLLOW_KP,
        help="Cartesian position proportional gain",
    )
    follow.add_argument(
        "--ki",
        type=float,
        default=DEFAULT_FOLLOW_KI,
        help="Cartesian position integral gain",
    )
    # 손끝 위치가 목표에 도착했다고 판단할 거리 오차를 설정한다.
    follow.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_FOLLOW_TOLERANCE_M,
        help="TCP position deadband in metres",
    )

    # 손끝 방향이 목표에 도착했다고 판단할 각도 오차를 설정한다.
    follow.add_argument(
        "--orientation-tolerance",
        type=float,
        default=DEFAULT_FOLLOW_ORIENTATION_TOLERANCE_RAD,
        help="TCP orientation deadband in radians",
    )

    # 실제 손끝에 보낼 최대 직선 이동속도를 설정한다.
    follow.add_argument(
        "--max-tcp-speed",
        type=float,
        default=DEFAULT_MAX_TCP_SPEED_M_S,
        help="maximum commanded TCP speed in metres per second",
    )

    # 실제 손끝에 보낼 최대 회전속도를 설정한다.
    follow.add_argument(
        "--max-tcp-angular-speed",
        type=float,
        default=DEFAULT_MAX_TCP_ANGULAR_SPEED_RAD_S,
        help="maximum commanded TCP angular speed in radians per second",
    )

    # IK에 전달할 위치 중간목표의 최대 거리를 설정한다.
    follow.add_argument(
        "--max-ik-step",
        type=float,
        default=DEFAULT_MAX_IK_STEP_M,
        help=(
            "maximum Cartesian distance from the measured TCP "
            "to each intermediate IK target in metres"
        ),
    )

    # IK에 전달할 방향 중간목표의 최대 회전각을 설정한다.
    follow.add_argument(
        "--max-ik-angular-step",
        type=float,
        default=DEFAULT_MAX_IK_ANGULAR_STEP_RAD,
        help=(
            "maximum angular distance from the measured TCP "
            "to each intermediate IK target in radians"
        ),
    )

    # 중력보상이 켜진 뒤 실제 자세가 안정될 시간을 설정한다.
    follow.add_argument(
        "--startup-settle-sec",
        type=float,
        default=DEFAULT_STARTUP_SETTLE_SEC,
        help=(
            "seconds to apply gravity compensation before accepting "
            "the initial TCP-to-marker alignment"
        ),
    )

    # 시작할 때 RViz 마커가 너무 멀면 실물이 움직이지 않도록 제한한다.
    follow.add_argument(
        "--max-start-distance",
        type=float,
        default=DEFAULT_MAX_START_DISTANCE_M,
        help=(
            "maximum allowed TCP-to-marker distance during startup "
            "alignment in metres"
        ),
    )

    # 시작할 때 방향 차이가 너무 크면 실물이 회전하지 않도록 제한한다.
    follow.add_argument(
        "--max-start-angle",
        type=float,
        default=DEFAULT_MAX_START_ANGLE_RAD,
        help=(
            "maximum allowed TCP-to-marker orientation difference "
            "during startup alignment in radians"
        ),
    )

    follow.add_argument(
        "--output",
        type=Path,
        help=(
            "write a JSON trace that separates marker, IK, command, and "
            "measured tracking lag"
        ),
    )

    follow.add_argument(
        "--diagnostic-profile",
        choices=DIAGNOSTIC_PROFILE_KINDS,
        help=(
            "replace live marker updates after startup with a deterministic "
            "round-trip translation, rotation, or combined diagnostic target"
        ),
    )
    follow.add_argument(
        "--diagnostic-distance",
        type=float,
        default=DEFAULT_DIAGNOSTIC_DISTANCE_M,
        help="translation profile distance in metres (hard maximum 0.03)",
    )
    follow.add_argument(
        "--diagnostic-angle",
        type=float,
        default=DEFAULT_DIAGNOSTIC_ANGLE_RAD,
        help="rotation profile angle in radians (hard maximum 10 degrees)",
    )
    follow.add_argument(
        "--diagnostic-linear-speed",
        type=float,
        default=DEFAULT_DIAGNOSTIC_LINEAR_SPEED_M_S,
        help="translation target speed in m/s (hard maximum 0.02)",
    )
    follow.add_argument(
        "--diagnostic-angular-speed",
        type=float,
        default=DEFAULT_DIAGNOSTIC_ANGULAR_SPEED_RAD_S,
        help="rotation target speed in rad/s (hard maximum 0.10)",
    )
    follow.add_argument(
        "--diagnostic-hold-sec",
        type=float,
        default=DEFAULT_DIAGNOSTIC_HOLD_SEC,
        help="hold time at the displaced target and origin",
    )
    follow.add_argument(
        "--diagnostic-repetitions",
        type=int,
        default=DEFAULT_DIAGNOSTIC_REPETITIONS,
        help="number of round trips (hard maximum 3)",
    )
    follow.add_argument(
        "--diagnostic-translation-axis",
        choices=DIAGNOSTIC_AXES,
        default="x",
        help="world-frame axis for the translation profile",
    )
    follow.add_argument(
        "--diagnostic-rotation-axis",
        choices=DIAGNOSTIC_AXES,
        default="z",
        help="startup-TCP local axis for the rotation profile",
    )
    follow.add_argument(
        "--ik-jump-threshold",
        type=float,
        default=DEFAULT_IK_TARGET_JUMP_THRESHOLD_RAD,
        help=(
            "record, but do not block, an IK target transition when any "
            "joint changes by at least this many radians"
        ),
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
        if args.stage in ("ready", "rest"):
            return _pose_park(args, profile, raise_elbow=args.stage == "ready")
        if args.stage == "gravity":
            return _pose_gravity(args, profile)
        if args.stage == "torque":
            return _pose_torque(args, profile)
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
    limits = _joint_limits(profile, group)
    gate = CommandGate(
        execute=True,
        lower=np.array([joint.lower for joint in limits]),
        upper=np.array([joint.upper for joint in limits]),
        velocity=np.array([joint.velocity for joint in limits]),
        command_period_sec=1.0 / profile.endpoint().command_rate_hz,
        effort=np.array([joint.effort for joint in limits]),
        max_lead=np.array([joint.velocity * LEAD_SEC for joint in limits]),
        names=list(group.joints),
    )
    if seed is not None:
        # Seeding makes the velocity limit apply to the move itself, not just
        # between waypoints of a multi-point plan.
        gate.authorize(seed, now_sec=0.0)
    return gate


def _joint_limits(profile, group):
    """The group's joints in order, with their profile limit entries."""
    joints = {joint.canonical: joint for joint in profile.joints}
    return [joints[canonical] for canonical in group.joints]


def _start_pose(profile, group, measured) -> np.ndarray:
    """Adopt the measured pose as a move's start, absorbing standing droop.

    Within SEED_SLACK_RAD of a bound the pose is clamped inside it, so the
    ramp's first waypoints — and the gate's seed — are legal. Further out the
    measured pose is returned untouched, and the gate refuses it naming the
    joint, because that is no longer droop against a stop.
    """
    limits = _joint_limits(profile, group)
    lower = np.array([joint.lower for joint in limits])
    upper = np.array([joint.upper for joint in limits])
    measured = np.asarray(measured, dtype=float)
    clamped = np.clip(measured, lower, upper)
    if np.all(np.abs(clamped - measured) <= SEED_SLACK_RAD):
        return clamped
    return measured


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
    snapshots = []
    try:
        for name, group in groups.items():
            adapter = RosAdapter(profile, name, execute=False, backend=backend)
            state = adapter.read_state()
            values = " ".join(f"{value:+.4f}" for value in state)
            print(f"{name}: {values}")
            tcp = None
            if group.moveit_group is not None and group.tip_link is not None:
                pose = adapter.read_pose()
                xyz = " ".join(f"{value:+.4f}" for value in pose.position)
                rpy = " ".join(f"{value:+.4f}" for value in pose.rpy)
                print(f"{name}: {group.tip_link} xyz [{xyz}] rpy [{rpy}]")
                tcp = {
                    "frame_id": pose.frame_id,
                    "tip_link": group.tip_link,
                    "xyz_m": [float(value) for value in pose.position],
                    "quaternion_xyzw": [
                        float(value) for value in pose.orientation
                    ],
                    "rpy_rad": [float(value) for value in pose.rpy],
                }
            snapshots.append(
                {
                    "name": name,
                    "joint_names": list(group.joints),
                    "joint_positions_rad": [float(value) for value in state],
                    "tcp": tcp,
                }
            )
    finally:
        backend.close()
    if args.output is not None:
        _write_json_atomic(
            args.output,
            {
                "schema_version": 1,
                "kind": "pose_snapshot",
                "profile": profile.name,
                "groups": snapshots,
            },
        )
        print(f"wrote pose snapshot: {args.output}")
    return 0


def _write_json_atomic(path: Path, payload: dict) -> None:
    """Write one complete JSON object without exposing a partial destination."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _ramp(
    start: Sequence[float] | np.ndarray,
    target: Sequence[float] | np.ndarray,
    duration_sec: float,
    rate_hz: float,
) -> list[np.ndarray]:
    """Waypoints from *start* to *target*, one per command period.

    The controller runs `interpolation_method: none`, so it holds the state a
    trajectory was installed with until a waypoint comes due and then steps to
    it — see `STREAM_HORIZON_SEC` for the same behaviour on the servo path. One
    waypoint a whole duration away is therefore not a slow move but a wait and
    then a jump: measured on the OpenArm as an abrupt lunge, and at ten seconds
    as an arm that appears not to move at all until it suddenly does.

    Built before the gate rather than inside the adapter, so that what was
    authorized is what goes on the wire. It is also what makes the gate's
    velocity check bite: a single distant waypoint has a whole duration of
    budget and passes whatever it asks for.
    """
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    steps = max(1, int(round(duration_sec * rate_hz)))
    return [start + (target - start) * (step / steps) for step in range(1, steps + 1)]


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
        here = _start_pose(profile, group, adapter.read_state())
        rate = profile.endpoint().command_rate_hz
        gate = _gate(profile, group, seed=here)
        points = gate.authorize_trajectory(
            _ramp(here, target, args.duration, rate),
            start_time_sec=0.0,
            period_sec=1.0 / rate,
        )
        _describe(group, interface, target)
        if group.action == PARALLEL_GRIPPER_COMMAND:
            adapter.send_gripper(float(points[-1][0]))
        else:
            adapter.send_trajectory(points, period_sec=1.0 / rate)
        print(f"EXECUTED: {group.name} over {args.duration:g} s")
    return 0


def _wrist_lift_sign(name: str) -> float:
    """The lift that clears a mounted tool from the table, per arm.

    The arms are mirrored — joint 7's axis carries the xacro's reflect — so
    the same physical pitch away from the table is positive on the right and
    negative on the left, matching the direction the table itself pushes a
    resting tool (left j7 is pinned near its negative limit at rest).
    """
    return -1.0 if "left" in name else 1.0


def _park_targets(name: str, here: np.ndarray, raise_elbow: bool):
    """The two waypoint poses of a park move, in the order they are visited.

    Ready: lift the elbow with everything else holding, then settle. Rest:
    settle with the elbow still up, then lower. Either way the wrist is only
    ever commanded while the elbow is raised, and it ends at the lifted angle
    in both directions — see READY_ELBOW_RAD's comment for why.
    """
    settled = np.zeros_like(here)
    settled[ELBOW_INDEX] = READY_ELBOW_RAD
    settled[WRIST_INDEX] = _wrist_lift_sign(name) * READY_WRIST_RAD
    if raise_elbow:
        lifted = here.copy()
        lifted[ELBOW_INDEX] = READY_ELBOW_RAD
        return [lifted, settled]
    lowered = settled.copy()
    lowered[ELBOW_INDEX] = 0.0
    # From wherever the arm was working: keep its elbow up while the wrist
    # and shoulder settle, then lower.
    settled[ELBOW_INDEX] = max(float(here[ELBOW_INDEX]), READY_ELBOW_RAD)
    return [settled, lowered]


def _pose_park(args, profile, raise_elbow: bool) -> int:
    """Move each arm between the table rest and the initial working state.

    Paced by PARK_SPEED_RAD_PER_SEC rather than a duration, so the move is
    equally slow whether the arm starts at rest or somewhere it was left
    mid-experiment. Arms go one at a time: a single operator watches a single
    arm.
    """
    from .ros_adapter import RosAdapter

    names = args.group or sorted(
        name for name in profile.groups if name.endswith("_arm")
    )
    for name in names:
        group = _group(profile, name)
        if not name.endswith("_arm"):
            raise ValueError(
                f"{name!r} is not an arm group; ready/rest move whole arms"
            )

        if not args.execute:
            final = _park_targets(name, np.zeros(len(group.joints)), raise_elbow)[-1]
            print(
                f"DRY RUN: {name}: would move in two phases to "
                f"[{', '.join(f'{value:g}' for value in final)}] at "
                f"{PARK_SPEED_RAD_PER_SEC:g} rad/s; pass --execute to send"
            )
            continue

        with RosAdapter(profile, name, execute=True) as adapter:
            rate = profile.endpoint().command_rate_hz
            here = _start_pose(profile, group, adapter.read_state())
            for target in _park_targets(name, here, raise_elbow):
                travel = float(np.abs(target - here).max())
                if travel < 1e-4:
                    continue
                duration = max(travel / PARK_SPEED_RAD_PER_SEC, 1.0)
                gate = _gate(profile, group, seed=here)
                points = gate.authorize_trajectory(
                    _ramp(here, target, duration, rate),
                    start_time_sec=0.0,
                    period_sec=1.0 / rate,
                )
                adapter.send_trajectory(points, period_sec=1.0 / rate)
                print(f"{name}: phase over {duration:.1f} s")
                here = target
            print(f"EXECUTED: {name}")
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
        seed = _start_pose(profile, group, adapter.read_state())
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
        rate = profile.endpoint().command_rate_hz
        gate = _gate(profile, group, seed=seed)
        points = gate.authorize_trajectory(
            _ramp(seed, solution, args.duration, rate),
            start_time_sec=0.0,
            period_sec=1.0 / rate,
        )

        start = " ".join(f"{value:+.4f}" for value in current.position)
        goal = " ".join(f"{value:+.4f}" for value in target.position)
        print(f"{group.tip_link}: [{start}] -> [{goal}] in {target.frame_id}")
        _describe(group, interface, solution)
        if not args.execute:
            print("DRY RUN: solved but not sent; pass --execute to send")
            return 0
        adapter.send_trajectory(points, period_sec=1.0 / rate)
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
        actual = _start_pose(profile, group, adapter.read_state())
        command = command + (solution - actual)
        # A fresh gate each pass, so the wound-up command is checked against the
        # profile limits rather than trusted for having been safe once.
        rate = profile.endpoint().command_rate_hz
        gate = _gate(profile, group, seed=actual)
        points = gate.authorize_trajectory(
            _ramp(actual, command, args.duration, rate),
            start_time_sec=0.0,
            period_sec=1.0 / rate,
        )
        adapter.send_trajectory(points, period_sec=1.0 / rate)
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


def _group_chain(urdf: str, profile, group):
    """Build *group*'s chain from *urdf*, in whichever naming it carries.

    The bringup description names joints at the source
    (openarm_right_joint1...); the generated asset URDF names them canonically
    (r_aj_1...) and ends at the group's `asset_tip_link` instead of its
    `tip_link`. Which convention a file uses is read off the file itself, so
    either can serve as the gravity model.
    """
    from xml.etree import ElementTree

    from .kinematics import KinematicsError, chain_from_urdf

    source_by_canonical = {joint.canonical: joint.source for joint in profile.joints}
    sources = [source_by_canonical[canonical] for canonical in group.joints]
    present = {
        element.get("name")
        for element in ElementTree.fromstring(urdf).findall("joint")
    }
    if all(name in present for name in sources):
        return chain_from_urdf(urdf, sources, group.tip_link)
    if all(name in present for name in group.joints):
        if group.asset_tip_link is None:
            raise KinematicsError(
                f"the URDF names joints canonically, but group {group.name!r} "
                "declares no asset_tip_link to end the chain at"
            )
        return chain_from_urdf(urdf, list(group.joints), group.asset_tip_link)
    missing_source = next(name for name in sources if name not in present)
    missing_canonical = next(
        name for name in group.joints if name not in present
    )
    raise KinematicsError(
        f"the URDF matches neither naming for group {group.name!r}: it has "
        f"no joint {missing_source!r} (source) and no joint "
        f"{missing_canonical!r} (canonical)"
    )


def _gravity_chain(adapter, profile, group, urdf_path=None):
    """Build the group's chain from *urdf_path*, or the running stack's URDF.

    An explicit file wins: the bringup description carries the vendor's
    masses, and a hand the vendor never mounted is exactly what an asset URDF
    override is for.
    """
    if urdf_path is not None:
        urdf = Path(urdf_path).read_text()
    else:
        urdf = adapter.read_robot_description()
    return _group_chain(urdf, profile, group)


def _pose_gravity(args, profile) -> int:
    """Publish gravity feedforward, at one scale set or measured across several.

    The model is only as good as the URDF's masses, and the gains it works
    against are hard-coded in the vendor hardware rather than configured, so the
    right scale is a measured quantity. --sweep measures it: hold at each scale,
    read the controller's own tracking error, and print what actually happened.

    Scales are per joint because the measured optima differ per joint — the
    modelled torque's *distribution* is off, not only its magnitude — so one
    global number can only reach a compromise between them.
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
    if args.scale is None and args.sweep is None:
        raise ValueError("pose gravity needs --scale, --sweep, or both")
    if args.sweep_joint is not None and args.sweep is None:
        raise ValueError("--sweep-joint says which joint --sweep varies; add --sweep")
    if args.hold_sec <= 0:
        raise ValueError("--hold-sec must be positive")
    if args.output is not None and not args.execute:
        raise ValueError(
            "--output records what the arm measured, so it needs --execute; a "
            "dry run publishes nothing and there would be nothing to record"
        )

    base = _scale_vector(args.scale, group)
    index = None
    if args.sweep_joint is not None:
        if args.sweep_joint not in group.joints:
            raise ValueError(
                f"{args.sweep_joint!r} is not a joint of {group.name!r}; it has "
                f"{list(group.joints)}"
            )
        index = group.joints.index(args.sweep_joint)

    if args.sweep is None:
        rounds = [base]
    else:
        rounds = []
        for scale in _parse_floats(args.sweep, "--sweep"):
            step = base.copy()
            if index is None:
                step[:] = scale
            else:
                step[index] = scale
            rounds.append(step)
    for step in rounds:
        _check_scales(step, group)

    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group, args.urdf)
        state = adapter.read_state()
        modelled = chain.gravity_torque(state)
        gate = _gate(profile, group, seed=None)

        print(f"{group.name}: {len(chain)} joints, "
              f"{sum(link.mass for link in chain.links):.3f} kg modelled")
        _describe_torque(group, modelled, base)
        if not args.execute:
            print("DRY RUN: torque computed but not published; pass --execute")
            return 0

        sweep = _measure_sweep(
            adapter, chain, gate, group, rounds, args.hold_sec, index, args.sweep_joint
        )
        if sweep.rounds > 1:
            _report_sweep(group, sweep, index)
        if args.output is not None:
            write_sweep(args.output, sweep, profile)
            print(f"wrote {args.output}")
    return 0


def _measure_sweep(
    adapter, chain, gate, group, rounds, hold_sec, index, sweep_joint
) -> GravitySweep:
    """Hold each scale in turn, read what the arm did, and release the torque."""
    poses, torques, applied, published, errors = [], [], [], [], []
    try:
        for scales in rounds:
            # Recomputed each round: compensation moves the arm, and the torque
            # that holds it depends on where it now is.
            state = adapter.read_state()
            torque = chain.gravity_torque(state)
            effort = gate.authorize_effort(torque * scales)
            _publish_for(adapter, effort, hold_sec)
            error = adapter.read_tracking_error()
            poses.append(state)
            torques.append(torque)
            applied.append(scales)
            published.append(effort)
            errors.append(error)
            print(
                f"scale {_scale_label(scales, index):>8}: worst joint error "
                f"{np.max(np.abs(error)):+.4f} rad, "
                f"mean {np.mean(np.abs(error)):.4f} rad"
            )
    finally:
        # Torque left applied after this process exits would keep pushing.
        adapter.send_effort(np.zeros(len(group.joints)))
        print("torque released")
    return GravitySweep(
        group=group.name,
        joint_names=tuple(group.joints),
        poses=np.asarray(poses, dtype=float),
        modelled_torque=np.asarray(torques, dtype=float),
        scales=np.asarray(applied, dtype=float),
        applied_torque=np.asarray(published, dtype=float),
        errors=np.asarray(errors, dtype=float),
        sweep_joint=sweep_joint,
    )


def _pose_torque(args, profile) -> int:
    """Measure stiffness with a torque we chose rather than one gravity gave.

    `pose gravity` can only publish a multiple of the modelled torque, so a
    joint the model says carries nothing — a wrist whose tool sits on its own
    axis — cannot be excited at all, whatever the scale. This stage instead
    probes a torque per joint from a small seed push and drives a staircase
    with it, one joint at a time.
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
    joints = args.joint or list(group.joints)
    for name in joints:
        if name not in group.joints:
            raise ValueError(
                f"{name!r} is not a joint of {group.name!r}; it has "
                f"{list(group.joints)}"
            )
    repeated = sorted({name for name in joints if joints.count(name) > 1})
    if repeated:
        # A joint's rounds are read back out of the file as the block between
        # its first and last round carrying torque, so a joint driven twice
        # claims every joint driven between its two turns as well.
        raise ValueError(
            f"--joint names {', '.join(repeated)} more than once; a joint gets "
            "one turn per file, since its rounds are read back as one "
            "contiguous block"
        )
    if args.hold_sec <= 0:
        raise ValueError("--hold-sec must be positive")
    # Both of these are refused downstream as well, but only after the
    # escalation has run — on a real joint that means publishing all the way to
    # the ceiling or the doubling cap first. A number that was never going to
    # be accepted should cost nothing to reject.
    #
    # `isfinite and > 0` rather than `<= 0`: nan fails every comparison, so it
    # passes a `<= 0` guard, and inf passes it outright. Either one makes the
    # motion threshold something no reading is ever above, and the joint is
    # walked to the cap and then told its dry friction exceeds a quarter of its
    # rating — the false diagnosis the split ceiling refusal exists to prevent.
    if not (np.isfinite(args.deflection) and args.deflection > 0):
        raise ValueError(
            f"--deflection must be a positive number, got {args.deflection:g}; "
            "it is how far each joint is pushed at the ends of its staircase, "
            "and the torque that produces it is what the run probes for"
        )
    if not (np.isfinite(args.noise) and args.noise > 0):
        # `identify --noise 0` only drops fewer rounds; here it commands the arm.
        raise ValueError(
            f"--noise must be a positive number, got {args.noise:g}; it is the "
            "encoder noise the seed has to be seen through, and without it "
            "dither reads as motion and sizes a torque from nothing"
        )
    if args.steps < MIN_STAIRCASE_STEPS:
        raise ValueError(
            f"--steps {args.steps} writes a sweep that cannot be fitted; the "
            f"fewest that can is {MIN_STAIRCASE_STEPS}. Each branch needs two "
            "rounds, and the first torque is not recorded — the joint arrives "
            "at it from the probe travelling the wrong way for that branch — "
            "so a shorter run costs a trip to the robot and produces nothing"
        )
    if args.output is not None and not args.execute:
        raise ValueError(
            "--output records what the arm measured, so it needs --execute; a "
            "dry run publishes nothing and there would be nothing to record"
        )

    limits = _joint_limits(profile, group)
    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group, args.urdf)
        gate = _gate(profile, group, seed=None)
        print(
            # One fewer than the staircase's torques: the first is where the
            # joint arrives from the probe, held but not recorded.
            f"{group.name}: {len(joints)} joints, {2 * args.steps - 2} rounds "
            f"each, {args.deflection:g} rad target deflection"
        )
        if not args.execute:
            print("DRY RUN: nothing published; pass --execute")
            return 0

        poses, applied, errors = measure_staircase(
            adapter, gate, group,
            joints=joints, limits=limits, deflection_rad=args.deflection,
            steps=args.steps, hold_sec=args.hold_sec,
            publish=lambda effort, seconds: _publish_for(adapter, effort, seconds),
            # A joint's seed is only known once it has been probed, and the run
            # is minutes long, so it is reported as it is measured rather than
            # collected up and printed at the end.
            announce=print,
            noise_rad=args.noise,
        )
        # The real gravity torque at each round's measured pose — the same
        # column a gravity sweep's alpha multiplies — so torque and gravity
        # sweeps fit jointly: gravity sweeps vary this column and pin alpha,
        # torque sweeps vary applied_torque instead and pin kp.
        modelled = np.array([chain.gravity_torque(pose) for pose in poses])
        sweep = GravitySweep(
            group=group.name,
            joint_names=tuple(group.joints),
            poses=poses,
            modelled_torque=modelled,
            # No fraction of the model was published — the torque came from
            # probe_torque, not from scaling the gravity model — so there is
            # no scale to record, only zeros.
            scales=np.zeros_like(applied),
            applied_torque=applied,
            errors=errors,
        )
        if args.output is not None:
            write_sweep(args.output, sweep, profile)
            print(f"wrote {args.output}")
    return 0


def _scale_vector(given: str | None, group) -> np.ndarray:
    """Read --scale as one value for every joint, or one value per joint."""
    if given is None:
        return np.ones(len(group.joints))
    values = _parse_floats(given, "--scale")
    if len(values) == 1:
        return np.full(len(group.joints), values[0])
    if len(values) != len(group.joints):
        raise ValueError(
            f"--scale needs one value or one per joint: {group.name!r} has "
            f"{len(group.joints)} joints, got {len(values)}"
        )
    return np.asarray(values, dtype=float)


def _check_scales(scales: np.ndarray, group) -> None:
    for canonical, scale in zip(group.joints, scales):
        if not 0.0 <= scale <= MAX_GRAVITY_SCALE:
            raise ValueError(
                f"gravity scale {scale:g} for {canonical} is outside 0 to "
                f"{MAX_GRAVITY_SCALE:g}; over-compensating drives the arm away "
                "from where it was holding"
            )


def _scale_label(scales: np.ndarray, index: int | None) -> str:
    """Label a round by the number that varied, or by the shared one."""
    if index is not None:
        return f"{scales[index]:.2f}"
    if np.allclose(scales, scales[0]):
        return f"{scales[0]:.2f}"
    return "per-joint"


def _publish_for(adapter, effort, seconds: float) -> None:
    """Republish *effort* at the controller rate for *seconds*, at least once.

    ForwardCommandController holds its last command, so one message would do,
    but republishing means a dropped message cannot silently leave the arm on a
    stale torque. The publish comes before the deadline check rather than after
    it, so *seconds* of 0.0 still puts one message out rather than none.
    """
    deadline = time.monotonic() + seconds
    while True:
        adapter.send_effort(effort)
        if time.monotonic() >= deadline:
            return
        time.sleep(0.01)


def _describe_torque(group, torque, scales) -> None:
    print("  joint            modelled (N.m)   scale   published (N.m)")
    for canonical, value, scale in zip(group.joints, torque, scales):
        print(f"  {canonical:<16} {value:+8.2f}   {scale:9.2f} {value * scale:+12.2f}")


def _report_sweep(group, sweep, index: int | None) -> None:
    """Print the sweep as a table, and name what measured best.

    With one joint varying, "best" is that joint's own error: a global worst
    would be dominated by joints this sweep never touched.
    """
    print()
    print("  scale  " + "".join(f"{canonical:>10}" for canonical in group.joints))
    rows = list(zip(sweep.scales, sweep.errors))
    for scales, error in rows:
        label = _scale_label(scales, index)
        print(f"  {label:>5}  " + "".join(f"{value:+10.4f}" for value in error))

    score = (
        (lambda row: abs(float(row[1][index])))
        if index is not None
        else (lambda row: float(np.max(np.abs(row[1]))))
    )
    best = min(rows, key=score)
    print()
    if index is not None:
        joint = group.joints[index]
        print(
            f"best measured scale for {joint}: {best[0][index]:g} "
            f"(that joint {best[1][index]:+.4f} rad)"
        )
        print(
            "  refine the next joint the same way, then hold them all at once:\n"
            "  --scale " + ",".join(f"{value:g}" for value in best[0])
        )
    else:
        print(
            f"best measured scale: {best[0][0]:g} "
            f"(worst joint {np.max(np.abs(best[1])):+.4f} rad)"
        )
        print(
            "  refine one joint at a time from here with --sweep-joint, since "
            "each joint's optimum differs"
        )


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
    _check_scales(_scale_vector(args.gravity, group), group)
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    if args.output is not None and not args.execute:
        raise ValueError(
            "--output records a pose follow run, so it needs --execute"
        )
    # These legacy flags remain accepted while the MoveIt-target follower uses
    # kp as its measured-joint outer-loop gain. Constructing the old controller
    # here retains the established finite/range validation for all four values.
    CartesianPI(
        kp=args.kp,
        ki=args.ki,
        tolerance=args.tolerance,
        max_speed=args.max_tcp_speed,
    )
        # 방향 허용오차는 0보다 크고 최대 회전각인 pi 이하여야 한다.
    if (
        not np.isfinite(args.orientation_tolerance)
        or args.orientation_tolerance <= 0.0
        or args.orientation_tolerance > np.pi
    ):
        raise ValueError(
            "--orientation-tolerance must be finite, positive, "
            "and no greater than pi"
        )

    # 손끝 최대 회전속도는 유한한 양수여야 한다.
    if (
        not np.isfinite(args.max_tcp_angular_speed)
        or args.max_tcp_angular_speed <= 0.0
    ):
        raise ValueError(
            "--max-tcp-angular-speed must be finite and positive"
        )
    # 중간 IK 목표 거리는 유한한 양수여야 하며,
    # 마커 변화 감지 범위보다 작으면 다음 목표를 제출할 수 없으므로 거부한다.
    if (
        not np.isfinite(args.max_ik_step)
        or args.max_ik_step < args.tolerance
    ):
        raise ValueError(
            "--max-ik-step must be finite and at least --tolerance"
        )
    # 회전 중간목표는 방향 허용오차보다 크고 pi 이하여야 한다.
    if (
        not np.isfinite(args.max_ik_angular_step)
        or args.max_ik_angular_step < args.orientation_tolerance
        or args.max_ik_angular_step > np.pi
    ):
        raise ValueError(
            "--max-ik-angular-step must be finite, no greater than pi, "
            "and at least --orientation-tolerance"
        )
    # 안정화 시간은 유한한 0 이상의 값이어야 한다.
    if (
        not np.isfinite(args.startup_settle_sec)
        or args.startup_settle_sec < 0.0
    ):
        raise ValueError(
            "--startup-settle-sec must be finite and not negative"
        )

    # 시작 정렬 허용거리는 최소한 TCP 오차 허용범위보다 커야 한다.
    if (
        not np.isfinite(args.max_start_distance)
        or args.max_start_distance < args.tolerance
    ):
        raise ValueError(
            "--max-start-distance must be finite and at least --tolerance"
        )
    # 시작 방향 허용각은 방향 허용오차보다 크고 pi 이하여야 한다.
    if (
        not np.isfinite(args.max_start_angle)
        or args.max_start_angle < args.orientation_tolerance
        or args.max_start_angle > np.pi
    ):
        raise ValueError(
            "--max-start-angle must be finite, no greater than pi, "
            "and at least --orientation-tolerance"
        )
    if (
        not np.isfinite(args.ik_jump_threshold)
        or args.ik_jump_threshold <= 0.0
        or args.ik_jump_threshold > np.pi
    ):
        raise ValueError(
            "--ik-jump-threshold must be finite, positive, and no greater "
            "than pi"
        )
    diagnostic_profile = None
    if args.diagnostic_profile is not None:
        diagnostic_profile = DiagnosticProfile(
            kind=args.diagnostic_profile,
            distance_m=args.diagnostic_distance,
            angle_rad=args.diagnostic_angle,
            linear_speed_m_s=args.diagnostic_linear_speed,
            angular_speed_rad_s=args.diagnostic_angular_speed,
            hold_sec=args.diagnostic_hold_sec,
            repetitions=args.diagnostic_repetitions,
            translation_axis=args.diagnostic_translation_axis,
            rotation_axis=args.diagnostic_rotation_axis,
        )
        if diagnostic_profile.linear_speed_m_s > args.max_tcp_speed:
            raise ValueError(
                "--diagnostic-linear-speed must not exceed --max-tcp-speed"
            )
        if (
            diagnostic_profile.angular_speed_rad_s
            > args.max_tcp_angular_speed
        ):
            raise ValueError(
                "--diagnostic-angular-speed must not exceed "
                "--max-tcp-angular-speed"
            )
    period = 1.0 / profile.endpoint().command_rate_hz
    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group, args.urdf)
        gate = _gate(profile, group, seed=None)
        # 드래그 중 들어오는 마커 변경을 구독한다.
        adapter.watch_marker()

        # 사용자가 아직 드래그하지 않았어도 시작 정렬을 할 수 있도록
        # RViz가 현재 보관 중인 파란 마커 위치를 한 번 직접 읽는다.
        startup_marker_target = adapter.read_marker_pose()

        # 시작 시점의 실제 관절각을 읽는다.
        state = adapter.read_state()
        print(
            f"following {group.tip_link} at {1.0 / period:g} Hz for "
            f"{args.seconds:g} s, gravity "
            + (
                "off"
                if args.gravity is None
                else f"scale {_scale_label(_scale_vector(args.gravity, group), None)}"
            )
        )
        print(
            "startup alignment: do not drag until the actual TCP "
            "reaches the RViz marker"
        )
        if diagnostic_profile is not None:
            spec = diagnostic_profile.as_dict()
            print(
                f"diagnostic profile {spec['kind']}: "
                f"{spec['repetitions']} round trip(s), "
                f"{spec['duration_sec']:.1f} s after startup alignment"
            )
            print(
                "  deterministic target only; live marker updates are "
                "ignored after alignment"
            )
        if not args.execute:
            print("DRY RUN: nothing is published; pass --execute to follow")
            return 0
        # MoveIt service calls block, so a second node belongs exclusively to
        # the IK worker while this adapter keeps servicing feedback and commands.
        with RosAdapter(
            profile,
            args.group,
            execute=False,
            node_name="robot_control_pose_follow_ik",
        ) as ik_adapter:
            worker = LatestIkWorker(ik_adapter.solve_ik)
            try:
                diagnostics = _follow_loop(
                    adapter,
                    chain,
                    gate,
                    group,
                    state,
                    period,
                    args,
                    worker,
                    startup_marker_target,
                    diagnostic_profile,
                )
            finally:
                worker.close()
    if args.output is not None:
        _write_json_atomic(
            args.output,
            {
                "schema_version": 1,
                "kind": "pose_follow_diagnostics",
                "profile": profile.name,
                **diagnostics,
            },
        )
        print(f"wrote pose follow diagnostics: {args.output}")
    return 0


def _follow_loop(
    adapter,
    chain,
    gate,
    group,
    state,
    period,
    args,
    ik_worker,
    startup_marker_target,
    diagnostic_profile=None,
) -> dict:
    from .ros_adapter import Pose

    scales = None if args.gravity is None else _scale_vector(args.gravity, group)
    if scales is not None and not np.any(scales):
        scales = None

    command = np.asarray(state, dtype=float).copy()

    # 시작 정렬이 완료되었는지 판단하기 위한 위치 기준점이다.
    marker_origin = None
    tcp_origin = None
    startup_alignment_completed_elapsed_sec = None
    diagnostic_started_at = None
    diagnostic_sample = None
    diagnostic_origin_orientation = None

    # 마지막으로 IK에 제출한 위치와 방향을 각각 저장한다.
    # 마커 변화가 충분히 클 때만 새 IK를 요청하기 위해 사용한다.
    last_submitted_position = None
    last_submitted_orientation = None

    # 각 IK 결과가 어떤 최종 마커 목표를 기준으로 계산됐는지 저장한다.
    requested_positions: list[np.ndarray] = []
    requested_orientations: list[tuple[float, float, float, float]] = []

    # 처음에는 RViz 서비스에서 읽은 파란 마커를 사용한다.
    # 이후 사용자가 드래그하면 최신 피드백으로 교체한다.
    target = startup_marker_target
    samples = 0
    cycles = 0
    notes: dict[str, int] = {}
    lag_total = 0.0
    lag_worst = 0.0
    lag_last = 0.0
    within_tolerance = 0
    live_within_tolerance = 0
    # 실험 중 방향 오차의 합계·최대·마지막 값을 rad 단위로 저장한다.
    orientation_lag_total = 0.0
    orientation_lag_worst = 0.0
    orientation_lag_last = 0.0

    # 방향 오차가 허용범위 안에 있었던 제어주기 수를 저장한다.
    orientation_within_tolerance = 0
    live_orientation_within_tolerance = 0
    speed_limited = 0
    angular_speed_limited = 0
    joint_error_last = 0.0

    # The old headline error is tied to the marker snapshot that produced the
    # currently accepted IK result. Keep it for baseline comparisons, but also
    # record the live marker and every downstream layer so IK latency, servo
    # backlog, and actuator tracking are no longer folded into one number.
    component_names = (
        "live_marker_to_measured",
        "accepted_marker_to_measured",
        "marker_update_staleness",
        "accepted_marker_to_ik_target",
        "ik_target_to_command",
        "command_to_measured",
    )
    position_components = {
        name: {"total": 0.0, "worst": 0.0, "last": 0.0}
        for name in component_names
    }
    projection_names = (
        "live_marker_to_measured",
        "marker_update_staleness",
        "accepted_marker_to_ik_target",
        "ik_target_to_command",
        "command_to_measured",
    )
    signed_position_components = {
        name: {
            "total": 0.0, "minimum": float("inf"),
            "maximum": float("-inf"), "last": 0.0,
        }
        for name in projection_names
    }
    orientation_components = {
        name: {"total": 0.0, "worst": 0.0, "last": 0.0}
        for name in component_names
    }

    # A bounded JSON trace is small at 100 Hz (roughly 3,000 records for the
    # 30-second field test) and preserves transient IK branch changes that a
    # mean/worst/last summary cannot explain.
    trace: list[dict] = []

    # OpenArm의 관절 개수를 가져온다. 오른팔은 J1~J7이므로 7이다.
    joint_count = len(group.joints)

    # 실험 전체에서 관절별 절댓값 오차를 계속 더한다.
    # 실험 종료 후 샘플 수로 나누면 관절별 평균 오차가 된다.
    joint_error_sum_by_joint = np.zeros(joint_count, dtype=float)

    # 실험 중 관절별로 가장 크게 발생한 오차를 저장한다.
    joint_error_worst_by_joint = np.zeros(joint_count, dtype=float)

    # 가장 마지막 제어주기의 관절별 오차를 저장한다.
    joint_error_last_by_joint = np.zeros(joint_count, dtype=float)

    # Split the IK-target-to-measured joint error at the command boundary. The
    # first half diagnoses controller/cap backlog; the second diagnoses how far
    # the physical joint trails the command it was actually sent.
    joint_target_to_command_sum = np.zeros(joint_count, dtype=float)
    joint_target_to_command_worst = np.zeros(joint_count, dtype=float)
    joint_target_to_command_last = np.zeros(joint_count, dtype=float)
    joint_command_to_measured_sum = np.zeros(joint_count, dtype=float)
    joint_command_to_measured_worst = np.zeros(joint_count, dtype=float)
    joint_command_to_measured_last = np.zeros(joint_count, dtype=float)

    # Observe target transitions without changing or rejecting them.
    last_accepted_ik_target = None
    last_accepted_ik_sequence = None
    ik_target_transition_count = 0
    ik_target_jump_worst = np.zeros(joint_count, dtype=float)
    ik_target_jump_counts = np.zeros(joint_count, dtype=int)
    ik_target_jump_events: list[dict] = []

    # r_aj_4와 같은 관절 이름을 배열 번호로 바꾸기 위한 표를 만든다.
    # 예: {"r_aj_1": 0, "r_aj_4": 3}
    joint_index_by_name = {
        name: index for index, name in enumerate(group.joints)
    }

    # 각 관절이 안전 제한에 걸린 횟수를 제한 종류별로 저장한다.
    # position은 전체 위치 제한이고, lower와 upper는 그 방향을 나타낸다.
    joint_clamp_counts = {
        kind: np.zeros(joint_count, dtype=int)
        for kind in (
            "velocity",
            "lead",
            "position",
            "position_lower",
            "position_upper",
        )
    }

    state_wait_total = 0.0
    started = time.monotonic()
    last_cycle = started
    deadline = started + args.seconds
    termination = "completed"
    try:
        while time.monotonic() < deadline:
            cycle = time.monotonic()
            elapsed = max(period, cycle - last_cycle)
            last_cycle = cycle
            adapter.pump(timeout_sec=0.0)
            # 드래그 중 새로운 마커 위치가 들어오면 최신 목표로 교체한다.
            # Diagnostic mode instead generates a bounded target from the
            # startup anchor and deliberately ignores live marker updates.
            diagnostic_sample = None
            if (
                diagnostic_profile is not None
                and diagnostic_started_at is not None
            ):
                diagnostic_sample = diagnostic_profile.sample(
                    cycle - diagnostic_started_at,
                    marker_origin,
                    diagnostic_origin_orientation,
                )
                target = Pose(
                    diagnostic_sample.position,
                    diagnostic_sample.orientation,
                    "world",
                )
            else:
                latest_target = adapter.latest_marker_target()
                if latest_target is not None:
                    target = latest_target
            wait_started = time.monotonic()
            state = adapter.read_state(timeout_sec=1.0)
            state_wait_total += time.monotonic() - wait_started
            cycles += 1
            if scales is not None:
                adapter.send_effort(
                    gate.authorize_effort(chain.gravity_torque(state) * scales)
                )
            if target is not None:
                                # 현재 실제 관절각으로 실제 TCP 위치를 계산한다.
                # 실제 관절각으로 현재 TCP의 전체 자세를 계산한다.
                here = chain.pose(state)
                current_position = here[:3, 3].copy()
                current_orientation = _quaternion_from_rotation(
                    here[:3, :3]
                )

                # RViz 마커의 위치와 방향을 모두 가져온다.
                marker_position = np.asarray(
                    target.position,
                    dtype=float,
                )
                marker_orientation = tuple(
                    float(value)
                    for value in _normalize_quaternion(target.orientation)
                )

                # 시작 정렬과 일반 추종 모두 RViz 마커 방향을 목표로 사용한다.
                desired_orientation = marker_orientation

                # marker_origin이 아직 없으면 시작 위치 정렬 단계이다.
                if marker_origin is None:
                    # 정렬 중에는 상대 이동량이 아니라 파란 마커의
                    # 절대 위치를 실제 TCP 목표로 사용한다.
                    desired_position = marker_position.copy()

                    # 실제 TCP와 파란 마커 사이의 직선거리를 계산한다.
                    startup_distance = float(
                        np.linalg.norm(
                            marker_position - current_position
                        )
                    )
                    # 실제 TCP와 RViz 마커 사이의 방향 차이를 계산한다.
                    startup_angle = _quaternion_angular_distance(
                        current_orientation,
                        desired_orientation,
                    )
                    # 시작 마커가 너무 멀면 오래되거나 잘못된 목표일 수 있다.
                    # 이때는 실물을 움직이지 않고 Pose Follow를 중단한다.
                    if startup_distance > args.max_start_distance:
                        raise ValueError(
                            "startup TCP-to-marker distance "
                            f"{startup_distance:.3f} m exceeds "
                            f"--max-start-distance "
                            f"{args.max_start_distance:.3f} m; "
                            "move the RViz marker to Current before retrying"
                        )
                    # 시작 방향이 너무 다르면 갑작스러운 손목 회전을 막는다.
                    if startup_angle > args.max_start_angle:
                        raise ValueError(
                            "startup TCP-to-marker orientation difference "
                            f"{startup_angle:.3f} rad exceeds "
                            f"--max-start-angle "
                            f"{args.max_start_angle:.3f} rad; "
                            "move the RViz marker to Current before retrying"
                        )
                    # 중력보상을 충분히 적용했고 실제 TCP가 파란 마커의
                    # 오차 허용범위 안에 들어왔을 때만 정렬 완료로 처리한다.
                    startup_settle_elapsed = cycle - started
                    if (
                        startup_settle_elapsed
                        >= args.startup_settle_sec
                        and startup_distance <= args.tolerance
                        and startup_angle
                        <= args.orientation_tolerance
                    ):
                        # 두 시작점을 동일한 파란 마커 위치로 저장한다.
                        # 따라서 이후 최종 목표도 파란 마커의 절대 위치와 같다.
                        marker_origin = marker_position.copy()
                        tcp_origin = marker_position.copy()
                        startup_alignment_completed_elapsed_sec = (
                            startup_settle_elapsed
                        )
                        diagnostic_origin_orientation = marker_orientation

                        # 정렬 완료 지점에서 IK 목표를 한 번 새로 계산한다.
                        last_submitted_position = None
                        last_submitted_orientation = None
                        print(
                            "startup alignment complete; "
                            "drag the marker in RViz"
                        )
                        if diagnostic_profile is not None:
                            diagnostic_started_at = cycle
                            print(
                                "diagnostic profile started; do not drag "
                                "the RViz marker"
                            )

                else:
                    # 정렬 완료 후에는 기존 상대 이동 계산을 사용한다.
                    # 두 origin을 같은 위치로 저장했으므로 결과적으로
                    # 실제 TCP 목표는 파란 마커의 절대 위치와 일치한다.
                    desired_position = tcp_origin + (
                        marker_position - marker_origin
                    )

                # 실제 TCP에서 현재 목표까지의 방향을 계산한다.
                direction_to_marker = (
                    desired_position - current_position
                )
                distance_to_marker = float(
                    np.linalg.norm(direction_to_marker)
                )

                # 마커가 2cm보다 멀리 있다면 현재 TCP에서 마커 방향으로
                # 최대 --max-ik-step만큼 앞에 있는 중간 목표를 만든다.
                ik_position = desired_position.copy()

                if distance_to_marker > args.max_ik_step:
                    ik_position = current_position + (
                        direction_to_marker
                        * args.max_ik_step
                        / distance_to_marker
                    )
                # 현재 TCP 방향에서 RViz 목표 방향으로 최대 약 5.7도 앞의
                # quaternion을 만들어 IK의 회전 중간목표로 사용한다.
                ik_orientation, orientation_distance = (
                    _step_quaternion_towards(
                        current_orientation,
                        desired_orientation,
                        args.max_ik_angular_step,
                    )
                )
                # 이전에 제출한 중간 목표와 충분히 달라졌을 때만
                # 새로운 IK 계산을 요청한다.
                # 위치 또는 방향 중 하나라도 충분히 변하면 새 IK를 요청한다.
                position_changed = (
                    last_submitted_position is None
                    or np.linalg.norm(
                        ik_position - last_submitted_position
                    )
                    >= args.tolerance
                )
                orientation_changed = (
                    last_submitted_orientation is None
                    or _quaternion_angular_distance(
                        last_submitted_orientation,
                        ik_orientation,
                    )
                    >= args.orientation_tolerance
                )

                if position_changed or orientation_changed:
                    ik_worker.submit(
                        Pose(
                            tuple(ik_position),
                            ik_orientation,
                            "world",
                        ),
                        state,
                    )

                    # 다음 IK 요청과 비교할 위치·방향 중간목표를 저장한다.
                    last_submitted_position = ik_position.copy()
                    last_submitted_orientation = ik_orientation

                    # IK 결과와 최종 마커 목표의 오차를 연결하기 위해 저장한다.
                    requested_positions.append(desired_position.copy())
                    requested_orientations.append(desired_orientation)
            status = ik_worker.snapshot()
            if status.target is not None and status.target_sequence is not None:
                target_joints = status.target
                if status.target_sequence != last_accepted_ik_sequence:
                    if last_accepted_ik_target is not None:
                        jump = target_joints - last_accepted_ik_target
                        jump_abs = np.abs(jump)
                        ik_target_transition_count += 1
                        ik_target_jump_worst = np.maximum(
                            ik_target_jump_worst, jump_abs
                        )
                        triggered = np.flatnonzero(
                            jump_abs >= args.ik_jump_threshold
                        )
                        ik_target_jump_counts[triggered] += 1
                        if triggered.size:
                            accepted_at = next(
                                (
                                    timing.accepted_at_sec
                                    for timing in status.timings
                                    if timing.sequence
                                    == status.target_sequence
                                ),
                                None,
                            )
                            ik_target_jump_events.append(
                                {
                                    "observed_elapsed_sec": float(
                                        cycle - started
                                    ),
                                    "accepted_elapsed_sec": (
                                        None
                                        if accepted_at is None
                                        else float(accepted_at - started)
                                    ),
                                    "from_sequence": int(
                                        last_accepted_ik_sequence
                                    ),
                                    "to_sequence": int(
                                        status.target_sequence
                                    ),
                                    "joint_delta_rad": [
                                        float(value) for value in jump
                                    ],
                                    "triggered_joints": [
                                        group.joints[index]
                                        for index in triggered
                                    ],
                                }
                            )
                    last_accepted_ik_target = target_joints.copy()
                    last_accepted_ik_sequence = status.target_sequence
                accepted_position = requested_positions[
                    status.target_sequence - 1
                ]
                # 같은 IK 요청에 대응하는 최종 RViz 마커 방향을 가져온다.
                accepted_orientation = requested_orientations[
                    status.target_sequence - 1
                ]
                here = chain.pose(state)
                ik_target_pose = chain.pose(target_joints)
                lag = float(
                    np.linalg.norm(accepted_position - here[:3, 3])
                )
                lag_total += lag
                lag_worst = max(lag_worst, lag)
                lag_last = lag
                within_tolerance += int(lag <= args.tolerance)
                # 현재 실제 TCP 방향을 quaternion으로 변환한다.
                here_orientation = _quaternion_from_rotation(
                    here[:3, :3]
                )

                # RViz 최종 목표 방향과 실제 TCP 방향 사이의 회전각을 계산한다.
                orientation_lag = _quaternion_angular_distance(
                    accepted_orientation,
                    here_orientation,
                )

                # 매 제어주기의 방향 오차를 누적하고 최대값을 갱신한다.
                orientation_lag_total += orientation_lag
                orientation_lag_worst = max(
                    orientation_lag_worst,
                    orientation_lag,
                )
                orientation_lag_last = orientation_lag
                orientation_within_tolerance += int(
                    orientation_lag
                    <= args.orientation_tolerance
                )
                # 역기구학이 만든 목표 관절각과 실제 측정 관절각의 차이를
                # J1~J7별 절댓값으로 계산한다.
                joint_error_by_joint = np.abs(target_joints - state)

                # 현재 제어주기의 관절별 오차를 마지막 오차로 저장한다.
                # copy()를 사용해 다음 주기의 배열 변경과 분리한다.
                joint_error_last_by_joint = joint_error_by_joint.copy()

                # 관절별 오차를 매 제어주기마다 더한다.
                # 실험 종료 후 샘플 수로 나누어 평균 오차를 계산한다.
                joint_error_sum_by_joint += joint_error_by_joint

                # 지금까지의 최대 오차와 현재 오차를 관절별로 비교하여,
                # 더 큰 값을 각 관절의 최대 오차로 저장한다.
                joint_error_worst_by_joint = np.maximum(
                    joint_error_worst_by_joint,
                    joint_error_by_joint,
                )

                # 기존 출력과 호환되도록 J1~J7 중 가장 큰 마지막 오차도 유지한다.
                joint_error_last = float(np.max(joint_error_by_joint))

                # The motor's impedance loop holds by sitting behind its
                # command. Re-sending the IK target preserves that droop;
                # advancing the command while measured joints still trail the
                # target is the outer feedback loop that removes it.
                active_command = command.copy()
                candidate = command + (
                    args.kp * (target_joints - state) * elapsed
                )
                # 현재 명령 자세와 새 후보 명령 자세를 순기구학으로 계산한다.
                command_pose = chain.pose(command)
                candidate_pose = chain.pose(candidate)

                # 두 명령 자세 사이의 직선 이동거리를 계산한다.
                tcp_distance = float(
                    np.linalg.norm(
                        candidate_pose[:3, 3]
                        - command_pose[:3, 3]
                    )
                )

                # 두 명령 자세 사이의 회전 이동각을 계산한다.
                command_orientation = _quaternion_from_rotation(
                    command_pose[:3, :3]
                )
                candidate_orientation = _quaternion_from_rotation(
                    candidate_pose[:3, :3]
                )
                command_angular_distance = (
                    _quaternion_angular_distance(
                        command_orientation,
                        candidate_orientation,
                    )
                )

                # 100 Hz의 한 제어주기 동안 허용되는 최대 직선거리와
                # 최대 회전각을 각각 계산한다.
                permitted_distance = args.max_tcp_speed * period
                permitted_angle = (
                    args.max_tcp_angular_speed * period
                )

                linear_fraction = 1.0
                angular_fraction = 1.0
                linear_speed_limited = False
                angular_speed_was_limited = False

                # 후보 명령의 직선 이동이 너무 크면 이동 비율을 줄인다.
                if (
                    tcp_distance > permitted_distance
                    and tcp_distance > 0.0
                ):
                    linear_fraction = (
                        permitted_distance / tcp_distance
                    )
                    speed_limited += 1
                    linear_speed_limited = True

                # 후보 명령의 회전 이동이 너무 크면 이동 비율을 줄인다.
                if (
                    command_angular_distance > permitted_angle
                    and command_angular_distance > 0.0
                ):
                    angular_fraction = (
                        permitted_angle
                        / command_angular_distance
                    )
                    angular_speed_limited += 1
                    angular_speed_was_limited = True

                # 위치와 회전 중 더 강하게 제한된 비율을 전체 관절 명령에
                # 적용하여 두 제한을 동시에 만족시키도록 한다.
                fraction = min(
                    linear_fraction,
                    angular_fraction,
                )
                candidate = command + fraction * (
                    candidate - command
                )
                command, limited = gate.follow(candidate, state, elapsed)
                if limited is not None:
                    notes[limited] = notes.get(limited, 0) + 1
                for limit_kind, limited_joint_names in (
                    gate.last_follow_limits.items()
                ):
                    # 현재 제한 종류에 해당하는 J1~J7 횟수 배열을 가져온다.
                    counts = joint_clamp_counts[limit_kind]

                    # 이번 제어주기에 제한된 관절들의 횟수를 각각 1씩 증가시킨다.
                    for joint_name in limited_joint_names:
                        joint_index = joint_index_by_name[joint_name]
                        counts[joint_index] += 1

                # Measure against the command that was active when this state
                # was sampled, not the next command sent below. That timing
                # boundary keeps one cycle of intentional motion out of the
                # physical tracking-lag measurement.
                ik_target_orientation = _quaternion_from_rotation(
                    ik_target_pose[:3, :3]
                )

                position_errors = {
                    "live_marker_to_measured": float(
                        np.linalg.norm(desired_position - here[:3, 3])
                    ),
                    "accepted_marker_to_measured": lag,
                    "marker_update_staleness": float(
                        np.linalg.norm(desired_position - accepted_position)
                    ),
                    "accepted_marker_to_ik_target": float(
                        np.linalg.norm(
                            accepted_position - ik_target_pose[:3, 3]
                        )
                    ),
                    "ik_target_to_command": float(
                        np.linalg.norm(
                            ik_target_pose[:3, 3]
                            - command_pose[:3, 3]
                        )
                    ),
                    "command_to_measured": float(
                        np.linalg.norm(
                            command_pose[:3, 3] - here[:3, 3]
                        )
                    ),
                }
                orientation_errors = {
                    "live_marker_to_measured": (
                        _quaternion_angular_distance(
                            desired_orientation,
                            here_orientation,
                        )
                    ),
                    "accepted_marker_to_measured": orientation_lag,
                    "marker_update_staleness": (
                        _quaternion_angular_distance(
                            desired_orientation,
                            accepted_orientation,
                        )
                    ),
                    "accepted_marker_to_ik_target": (
                        _quaternion_angular_distance(
                            accepted_orientation,
                            ik_target_orientation,
                        )
                    ),
                    "ik_target_to_command": (
                        _quaternion_angular_distance(
                            ik_target_orientation,
                            command_orientation,
                        )
                    ),
                    "command_to_measured": (
                        _quaternion_angular_distance(
                            command_orientation,
                            here_orientation,
                        )
                    ),
                }
                live_error_vector = desired_position - here[:3, 3]
                live_error_norm = float(np.linalg.norm(live_error_vector))
                live_error_direction = (
                    np.zeros(3)
                    if live_error_norm <= 1e-12
                    else live_error_vector / live_error_norm
                )
                layer_vectors = {
                    "live_marker_to_measured": live_error_vector,
                    "marker_update_staleness": (
                        desired_position - accepted_position
                    ),
                    "accepted_marker_to_ik_target": (
                        accepted_position - ik_target_pose[:3, 3]
                    ),
                    "ik_target_to_command": (
                        ik_target_pose[:3, 3] - command_pose[:3, 3]
                    ),
                    "command_to_measured": (
                        command_pose[:3, 3] - here[:3, 3]
                    ),
                }
                signed_position_projections = {
                    name: float(np.dot(vector, live_error_direction))
                    for name, vector in layer_vectors.items()
                }
                for name, value in signed_position_projections.items():
                    component = signed_position_components[name]
                    component["total"] += value
                    component["minimum"] = min(component["minimum"], value)
                    component["maximum"] = max(component["maximum"], value)
                    component["last"] = value
                for name, value in position_errors.items():
                    component = position_components[name]
                    component["total"] += value
                    component["worst"] = max(component["worst"], value)
                    component["last"] = value
                live_within_tolerance += int(
                    position_errors["live_marker_to_measured"]
                    <= args.tolerance
                )
                for name, value in orientation_errors.items():
                    component = orientation_components[name]
                    component["total"] += value
                    component["worst"] = max(component["worst"], value)
                    component["last"] = value
                live_orientation_within_tolerance += int(
                    orientation_errors["live_marker_to_measured"]
                    <= args.orientation_tolerance
                )

                target_to_command = np.abs(
                    target_joints - active_command
                )
                command_to_measured = np.abs(active_command - state)
                joint_target_to_command_sum += target_to_command
                joint_target_to_command_worst = np.maximum(
                    joint_target_to_command_worst,
                    target_to_command,
                )
                joint_target_to_command_last = target_to_command.copy()
                joint_command_to_measured_sum += command_to_measured
                joint_command_to_measured_worst = np.maximum(
                    joint_command_to_measured_worst,
                    command_to_measured,
                )
                joint_command_to_measured_last = command_to_measured.copy()

                trace.append(
                    {
                        "elapsed_sec": float(cycle - started),
                        "ik_sequence": int(status.target_sequence),
                        "tcp_positions_m": {
                            "live_marker": [
                                float(value) for value in desired_position
                            ],
                            "accepted_marker": [
                                float(value) for value in accepted_position
                            ],
                            "ik_target": [
                                float(value)
                                for value in ik_target_pose[:3, 3]
                            ],
                            "command": [
                                float(value)
                                for value in command_pose[:3, 3]
                            ],
                            "measured": [
                                float(value) for value in here[:3, 3]
                            ],
                        },
                        "tcp_orientations_xyzw": {
                            "live_marker": [
                                float(value)
                                for value in desired_orientation
                            ],
                            "accepted_marker": [
                                float(value)
                                for value in accepted_orientation
                            ],
                            "ik_target": [
                                float(value)
                                for value in ik_target_orientation
                            ],
                            "command": [
                                float(value)
                                for value in command_orientation
                            ],
                            "measured": [
                                float(value) for value in here_orientation
                            ],
                        },
                        "position_error_m": position_errors,
                        "position_error_signed_projection_m": (
                            signed_position_projections
                        ),
                        "orientation_error_rad": orientation_errors,
                        "diagnostic_profile": (
                            None
                            if diagnostic_sample is None
                            else {
                                "phase": diagnostic_sample.phase,
                                "repetition": diagnostic_sample.repetition,
                                "elapsed_sec": float(
                                    cycle - diagnostic_started_at
                                ),
                            }
                        ),
                        "joint_positions_rad": {
                            "ik_target": [
                                float(value) for value in target_joints
                            ],
                            "command": [
                                float(value) for value in active_command
                            ],
                            "next_command": [
                                float(value) for value in command
                            ],
                            "measured": [
                                float(value) for value in state
                            ],
                        },
                        "joint_error_rad": {
                            "ik_target_to_measured": [
                                float(value)
                                for value in joint_error_by_joint
                            ],
                            "ik_target_to_command": [
                                float(value)
                                for value in target_to_command
                            ],
                            "command_to_measured": [
                                float(value)
                                for value in command_to_measured
                            ],
                        },
                        "limits": {
                            "cartesian_speed": linear_speed_limited,
                            "cartesian_angular_speed": (
                                angular_speed_was_limited
                            ),
                            "joint": {
                                name: list(joints)
                                for name, joints in (
                                    gate.last_follow_limits.items()
                                )
                            },
                        },
                    }
                )

                adapter.stream_positions(command)

                samples += 1
            if (
                diagnostic_sample is not None
                and diagnostic_sample.complete
            ):
                termination = "diagnostic_profile_completed"
                break
            time.sleep(max(0.0, period - (time.monotonic() - cycle)))
    except KeyboardInterrupt:
        termination = "interrupted"
        print("\ninterrupted")
    finally:
        # Stop streaming and release feedforward. Position control holds the
        # last approved command, which may still lead the measured joints by
        # max_lead, so the arm can settle slightly after gravity effort is zeroed.
        if scales is not None:
            adapter.send_effort(np.zeros(len(group.joints)))
        elapsed_total = max(time.monotonic() - started, 1e-12)
        status = ik_worker.snapshot()
        print(f"followed {samples} samples; the arm holds its last commanded pose")
        print(
            f"  actual control rate: {cycles / elapsed_total:.1f} Hz; "
            f"joint-state wait {state_wait_total / max(cycles, 1) * 1000:.1f} ms/sample"
        )
        print(
            f"  IK requests {status.submitted}: {status.succeeded} succeeded, "
            f"{status.failed} failed, {status.superseded} superseded"
        )
        if samples:
            print(
                f"  tool centre point trailed the marker by "
                f"{lag_total / samples * 1000:.1f} mm on average, "
                f"{lag_worst * 1000:.1f} mm at worst"
            )
            print(f"  last TCP position error: {lag_last * 1000:.1f} mm")
            print(
                f"  within {args.tolerance * 1000:.1f} mm on "
                f"{within_tolerance} of {samples} samples "
                f"({within_tolerance / samples * 100:.1f}%)"
            )
            live_position = position_components[
                "live_marker_to_measured"
            ]
            print(
                "  live marker to measured TCP: "
                f"{live_position['total'] / samples * 1000:.1f} mm "
                "on average, "
                f"{live_position['worst'] * 1000:.1f} mm at worst, "
                f"{live_position['last'] * 1000:.1f} mm last"
            )
            print("  mean position lag decomposition (norms are non-additive):")
            for name in (
                "marker_update_staleness",
                "accepted_marker_to_ik_target",
                "ik_target_to_command",
                "command_to_measured",
            ):
                print(
                    f"    {name}: "
                    f"{position_components[name]['total'] / samples * 1000:.1f} mm"
                )
            print("  mean signed projection on live-error direction:")
            for name in (
                "marker_update_staleness",
                "accepted_marker_to_ik_target",
                "ik_target_to_command",
                "command_to_measured",
            ):
                print(
                    f"    {name}: "
                    f"{signed_position_components[name]['total'] / samples * 1000:+.1f} mm"
                )
            # 사람이 이해하기 쉽도록 rad 단위 방향 오차를 deg로 바꿔 출력한다.
            print(
                "  TCP orientation trailed the marker by "
                f"{np.degrees(orientation_lag_total / samples):.1f} deg "
                "on average, "
                f"{np.degrees(orientation_lag_worst):.1f} deg at worst"
            )
            print(
                "  last TCP orientation error: "
                f"{np.degrees(orientation_lag_last):.1f} deg"
            )
            print(
                "  within "
                f"{np.degrees(args.orientation_tolerance):.1f} deg on "
                f"{orientation_within_tolerance} of {samples} samples "
                f"({orientation_within_tolerance / samples * 100:.1f}%)"
            )
            print(
                f"  Cartesian speed limit on {speed_limited} of {samples} samples"
            )
            print(
                "  Cartesian angular speed limit on "
                f"{angular_speed_limited} of {samples} samples"
            )
            print(f"  last maximum joint error: {joint_error_last:.4f} rad")

                         # 실험 중 누적한 관절별 오차를 샘플 수로 나누어
            # J1~J7 각각의 평균 절댓값 오차를 계산한다.
            joint_error_mean_by_joint = (
                joint_error_sum_by_joint / samples
            )

            print("  per-joint tracking diagnostics:")
            print(
                "    joint     mean(rad)  worst(rad)  last(rad)"
                "  velocity  lead  position  lower  upper"
            )

            # J1~J7의 오차와 제한 발생 횟수를 한 줄씩 출력한다.
            for joint_index, joint_name in enumerate(group.joints):
                print(
                    f"    {joint_name:<9}"
                    f" {joint_error_mean_by_joint[joint_index]:>9.4f}"
                    f" {joint_error_worst_by_joint[joint_index]:>11.4f}"
                    f" {joint_error_last_by_joint[joint_index]:>9.4f}"
                    f" {joint_clamp_counts['velocity'][joint_index]:>9d}"
                    f" {joint_clamp_counts['lead'][joint_index]:>5d}"
                    f" {joint_clamp_counts['position'][joint_index]:>9d}"
                    f" {joint_clamp_counts['position_lower'][joint_index]:>6d}"
                    f" {joint_clamp_counts['position_upper'][joint_index]:>6d}"
                )

            print("  per-joint lag decomposition:")
            print(
                "    joint     target-command mean/worst(rad)"
                "  command-measured mean/worst(rad)"
            )
            for joint_index, joint_name in enumerate(group.joints):
                print(
                    f"    {joint_name:<9}"
                    f" {joint_target_to_command_sum[joint_index] / samples:>9.4f}"
                    f"/{joint_target_to_command_worst[joint_index]:<10.4f}"
                    f" {joint_command_to_measured_sum[joint_index] / samples:>9.4f}"
                    f"/{joint_command_to_measured_worst[joint_index]:<10.4f}"
                )

        for note, count in sorted(notes.items()):
            print(f"  {note} clamped on {count} of {samples} samples")

    sample_divisor = max(samples, 1)
    position_summary = {
        name: {
            "mean": float(component["total"] / sample_divisor),
            "worst": float(component["worst"]),
            "last": float(component["last"]),
        }
        for name, component in position_components.items()
    }
    signed_position_summary = {
        name: {
            "mean": float(component["total"] / sample_divisor),
            "minimum": (
                float(component["minimum"]) if samples else 0.0
            ),
            "maximum": (
                float(component["maximum"]) if samples else 0.0
            ),
            "last": float(component["last"]),
        }
        for name, component in signed_position_components.items()
    }
    ik_timing_events = []
    for timing in status.timings:
        relative = lambda value: (
            None if value is None else float(value - started)
        )
        ik_timing_events.append(
            {
                "sequence": int(timing.sequence),
                "outcome": timing.outcome,
                "requested_elapsed_sec": relative(
                    timing.requested_at_sec
                ),
                "started_elapsed_sec": relative(timing.started_at_sec),
                "completed_elapsed_sec": relative(
                    timing.completed_at_sec
                ),
                "accepted_elapsed_sec": relative(
                    timing.accepted_at_sec
                ),
                "request_to_complete_sec": (
                    None
                    if timing.completed_at_sec is None
                    else float(
                        timing.completed_at_sec
                        - timing.requested_at_sec
                    )
                ),
                "request_to_accepted_sec": (
                    None
                    if timing.accepted_at_sec is None
                    else float(
                        timing.accepted_at_sec
                        - timing.requested_at_sec
                    )
                ),
            }
        )
    orientation_summary = {
        name: {
            "mean": float(component["total"] / sample_divisor),
            "worst": float(component["worst"]),
            "last": float(component["last"]),
        }
        for name, component in orientation_components.items()
    }
    per_joint = []
    for joint_index, joint_name in enumerate(group.joints):
        per_joint.append(
            {
                "name": joint_name,
                "ik_target_to_measured_rad": {
                    "mean": float(
                        joint_error_sum_by_joint[joint_index]
                        / sample_divisor
                    ),
                    "worst": float(
                        joint_error_worst_by_joint[joint_index]
                    ),
                    "last": float(
                        joint_error_last_by_joint[joint_index]
                    ),
                },
                "ik_target_to_command_rad": {
                    "mean": float(
                        joint_target_to_command_sum[joint_index]
                        / sample_divisor
                    ),
                    "worst": float(
                        joint_target_to_command_worst[joint_index]
                    ),
                    "last": float(
                        joint_target_to_command_last[joint_index]
                    ),
                },
                "command_to_measured_rad": {
                    "mean": float(
                        joint_command_to_measured_sum[joint_index]
                        / sample_divisor
                    ),
                    "worst": float(
                        joint_command_to_measured_worst[joint_index]
                    ),
                    "last": float(
                        joint_command_to_measured_last[joint_index]
                    ),
                },
                "clamp_samples": {
                    kind: int(counts[joint_index])
                    for kind, counts in joint_clamp_counts.items()
                },
            }
        )

    return {
        "group": group.name,
        "joint_names": list(group.joints),
        "settings": {
            "requested_seconds": (
                float(args.seconds) if np.isfinite(args.seconds) else None
            ),
            "unbounded_duration": not np.isfinite(args.seconds),
            "gravity_scale": (
                None
                if scales is None
                else [float(value) for value in scales]
            ),
            "kp_per_sec": float(args.kp),
            "position_tolerance_m": float(args.tolerance),
            "orientation_tolerance_rad": float(
                args.orientation_tolerance
            ),
            "max_tcp_speed_m_s": float(args.max_tcp_speed),
            "max_tcp_angular_speed_rad_s": float(
                args.max_tcp_angular_speed
            ),
            "max_ik_step_m": float(args.max_ik_step),
            "max_ik_angular_step_rad": float(
                args.max_ik_angular_step
            ),
            "max_joint_lead_sec": LEAD_SEC,
            "command_rate_hz": float(1.0 / period),
            "ik_target_jump_threshold_rad": float(
                args.ik_jump_threshold
            ),
            "diagnostic_profile": (
                None
                if diagnostic_profile is None
                else diagnostic_profile.as_dict()
            ),
        },
        "result": {
            "termination": termination,
            "duration_sec": float(elapsed_total),
            "cycles": int(cycles),
            "samples": int(samples),
            "actual_control_rate_hz": float(cycles / elapsed_total),
            "joint_state_wait_mean_ms": float(
                state_wait_total / max(cycles, 1) * 1000
            ),
            "ik": {
                "submitted": int(status.submitted),
                "succeeded": int(status.succeeded),
                "failed": int(status.failed),
                "superseded": int(status.superseded),
                "events": ik_timing_events,
            },
            "startup_alignment": {
                "completed": (
                    startup_alignment_completed_elapsed_sec is not None
                ),
                "completed_elapsed_sec": (
                    None
                    if startup_alignment_completed_elapsed_sec is None
                    else float(
                        startup_alignment_completed_elapsed_sec
                    )
                ),
            },
            "position_error_m": position_summary,
            "position_error_signed_projection_m": (
                signed_position_summary
            ),
            "orientation_error_rad": orientation_summary,
            "ik_target_jumps": {
                "threshold_rad": float(args.ik_jump_threshold),
                "transitions": int(ik_target_transition_count),
                "events": ik_target_jump_events,
                "per_joint": [
                    {
                        "name": joint_name,
                        "worst_abs_delta_rad": float(
                            ik_target_jump_worst[joint_index]
                        ),
                        "events_over_threshold": int(
                            ik_target_jump_counts[joint_index]
                        ),
                    }
                    for joint_index, joint_name in enumerate(
                        group.joints
                    )
                ],
            },
            "within_accepted_marker_position_tolerance_samples": int(
                within_tolerance
            ),
            "within_live_marker_position_tolerance_samples": int(
                live_within_tolerance
            ),
            "within_accepted_marker_orientation_tolerance_samples": int(
                orientation_within_tolerance
            ),
            "within_live_marker_orientation_tolerance_samples": int(
                live_orientation_within_tolerance
            ),
            "cartesian_speed_limited_samples": int(speed_limited),
            "cartesian_angular_speed_limited_samples": int(
                angular_speed_limited
            ),
            "per_joint": per_joint,
        },
        "trace": trace,
    }


def _quaternion_from_rotation(rotation) -> tuple[float, float, float, float]:
    """Return an x, y, z, w quaternion for a proper 3x3 rotation matrix."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        following = (index + 1) % 3
        remaining = (index + 2) % 3
        scale = 2.0 * np.sqrt(
            max(
                0.0,
                1.0
                + matrix[index, index]
                - matrix[following, following]
                - matrix[remaining, remaining],
            )
        )
        quaternion = np.zeros(4)
        quaternion[index] = 0.25 * scale
        quaternion[3] = (
            matrix[remaining, following] - matrix[following, remaining]
        ) / scale
        quaternion[following] = (
            matrix[following, index] + matrix[index, following]
        ) / scale
        quaternion[remaining] = (
            matrix[remaining, index] + matrix[index, remaining]
        ) / scale
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


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


def _normalize_quaternion(orientation) -> np.ndarray:
    """Quaternion을 길이가 1인 정상적인 quaternion으로 만든다."""
    quaternion = np.asarray(orientation, dtype=float)

    # Quaternion은 반드시 x, y, z, w 네 개의 값이어야 한다.
    if quaternion.shape != (4,):
        raise ValueError("orientation quaternion must contain four values")

    # nan이나 inf가 포함된 방향은 회전 계산에 사용할 수 없다.
    if not np.isfinite(quaternion).all():
        raise ValueError("orientation quaternion must be finite")

    length = float(np.linalg.norm(quaternion))

    # 길이가 0인 quaternion은 어떤 방향도 나타내지 못한다.
    if length <= 1e-12:
        raise ValueError("orientation quaternion must not have zero length")

    return quaternion / length


def _quaternion_angular_distance(first, second) -> float:
    """두 quaternion 사이의 가장 짧은 회전각을 rad 단위로 반환한다."""
    first_quaternion = _normalize_quaternion(first)
    second_quaternion = _normalize_quaternion(second)

    # q와 -q는 같은 방향을 나타내므로 내적의 절댓값을 사용한다.
    cosine_half_angle = abs(
        float(np.dot(first_quaternion, second_quaternion))
    )
    cosine_half_angle = float(
        np.clip(cosine_half_angle, 0.0, 1.0)
    )

    # Quaternion 내적은 회전각의 절반에 대한 cos 값이다.
    return 2.0 * float(np.arccos(cosine_half_angle))


def _step_quaternion_towards(
    current,
    goal,
    max_step_rad: float,
) -> tuple[tuple[float, float, float, float], float]:
    """현재 방향에서 목표 방향으로 최대 max_step_rad만큼 이동한다."""
    current_quaternion = _normalize_quaternion(current)
    goal_quaternion = _normalize_quaternion(goal)

    dot = float(np.dot(current_quaternion, goal_quaternion))

    # q와 -q는 같은 방향이다.
    # 내적이 음수이면 목표 quaternion의 부호를 바꿔 짧은 경로를 선택한다.
    if dot < 0.0:
        goal_quaternion = -goal_quaternion
        dot = -dot

    dot = float(np.clip(dot, 0.0, 1.0))
    angular_distance = 2.0 * float(np.arccos(dot))

    # 목표가 최대 회전 간격 안에 있으면 목표 방향을 그대로 반환한다.
    if angular_distance <= max_step_rad:
        return (
            tuple(float(value) for value in goal_quaternion),
            angular_distance,
        )

    # 전체 회전 중 이번 중간목표까지 진행할 비율을 계산한다.
    fraction = max_step_rad / angular_distance
    half_angle = float(np.arccos(dot))
    sine_half_angle = float(np.sin(half_angle))

    # 두 방향이 거의 같으면 수치적으로 안전한 선형 보간을 사용한다.
    if abs(sine_half_angle) <= 1e-12:
        stepped = (
            current_quaternion
            + fraction * (goal_quaternion - current_quaternion)
        )
    else:
        # SLERP: quaternion 구면 위의 가장 짧은 회전 경로를 따라간다.
        current_weight = np.sin((1.0 - fraction) * half_angle)
        goal_weight = np.sin(fraction * half_angle)
        stepped = (
            current_weight * current_quaternion
            + goal_weight * goal_quaternion
        ) / sine_half_angle

    stepped = _normalize_quaternion(stepped)

    return (
        tuple(float(value) for value in stepped),
        angular_distance,
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


def _is_staircase_sweep(sweep: GravitySweep) -> bool:
    """True for a sweep shaped like `pose torque`'s output, not `pose gravity`'s.

    `pose torque` publishes no fraction of the model — scales is all zero —
    and drives applied_torque directly to trace the staircase. A degenerate
    gravity sweep held at scale zero the whole way also has all-zero scales,
    but nothing was ever fed forward either, so its applied_torque is flat
    too; checking both is what tells the two apart.
    """
    return not np.any(sweep.scales) and bool(np.ptp(sweep.applied_torque))


def _identify(args, profile) -> int:
    """Fit stiffness, stiction and a torque correction from measured sweeps.

    Refuses rather than reports a partial answer. Least squares always returns
    something, and a stiffness for a joint whose load never varied is that
    something; downstream it becomes an inertia, and nothing after this point
    could tell it from a measured one.
    """
    from .ros_adapter import AdapterUnavailable

    reviewing = args.collect and not args.execute
    if not args.output and not reviewing:
        print("error: --output is required")
        return UNUSABLE
    if args.collect and not args.group:
        print("error: --collect needs --group, to say which arm to drive")
        return UNUSABLE
    if args.collect and args.execute and not args.sweep_dir:
        print(
            "error: --collect --execute needs --sweep-dir; the measurements are "
            "the only record of a run that moved the robot"
        )
        return UNUSABLE

    collected: list[Path] = []
    if args.collect:
        try:
            written = _collect_poses(args, profile)
        except (SafetyError, Refused) as error:
            print(f"refused: {error}")
            return REFUSED
        except AdapterUnavailable as error:
            print(f"unavailable: {error}")
            return UNUSABLE
        except (ValueError, OSError) as error:
            print(f"error: {error}")
            return UNUSABLE
        if written is None:
            return 0  # a review, which is all --collect without --execute does
        collected = written

    sweeps = list(args.sweep or []) + collected
    if len(sweeps) < 2:
        print(
            f"refused: --sweep must name at least two poses, got {len(sweeps)}. "
            "At one pose the modelled torque is a constant, so the stiffness, "
            "the friction and the model's own error are one equation in three "
            "unknowns."
        )
        return REFUSED

    try:
        measured = [read_sweep(path, profile) for path in sweeps]
        # Every sweep, gravity and staircase alike: gravity sweeps vary the
        # modelled-torque column and pin alpha, staircase sweeps vary
        # applied_torque instead and pin kp. Both are the joint fit the
        # spec's A3 describes, and it is what keeps torque_scale finite for a
        # joint a staircase alone would leave undetermined.
        estimate = fit_static_gravity(measured, noise_rad=args.noise)
    except ArtifactError as error:
        print(f"error: {error}")
        return UNUSABLE
    except FitError as error:
        print(f"error: {error}")
        return UNUSABLE

    staircase_estimate = None
    staircases = [sweep for sweep in measured if _is_staircase_sweep(sweep)]
    if staircases:
        try:
            staircase_estimate = fit_staircase(staircases)
        except FitError as error:
            # A structural problem (no sweeps, mismatched joints) rather than
            # a per-joint one; the gravity fit already succeeded, so the run
            # continues on that alone instead of losing everything over Fc
            # and Fo it never had a chance to measure.
            print(f"staircase: {error}")
        else:
            # Fc and Fo only. kp is the staircase's own second opinion,
            # printed beside the gravity fit's kp in _report_static, never
            # merged into the record fit_static_gravity owns; torque_scale
            # and offset stay whatever fit_static_gravity measured, since
            # fit_staircase never touches a gravity model at all.
            estimate = dataclasses.replace(
                estimate,
                coulomb_nm=staircase_estimate.coulomb_nm,
                bias_nm=staircase_estimate.bias_nm,
            )

    _report_static(estimate, len(measured), staircase=staircase_estimate)
    if estimate.unidentifiable:
        print(
            "refused: nothing written. Add a pose that loads the joints above "
            "differently, or free a joint sitting in its stiction band, and run "
            "identify again over every sweep."
        )
        return REFUSED

    try:
        write_static_estimate(
            args.output,
            estimate,
            profile,
            group=measured[0].group,
            noise_rad=args.noise,
            sources=[sweep_sha256(sweep) for sweep in measured],
        )
    except ArtifactError as error:
        # A joint can pass the `unidentifiable` gate above (its stiffness fit
        # succeeded) and still leave the writer refusing: torque_scale
        # undetermined, which the gate above cannot see from stiffness alone.
        print(f"refused: {error}")
        return REFUSED
    print(f"identify: {args.output}")
    return 0


def _fit(args, profile) -> int:
    """Fit the dynamic model, and with a static estimate, the physical parameters.

    Without --static this is the three-parameter fit it always was, which is
    right for a track with no standing load in it. With one, the modelled gravity
    torque enters as a fourth column — otherwise the regression has nowhere to
    put a standing load but into the stiffness — and the two fits together give
    the inertia neither can reach alone.
    """
    from .kinematics import KinematicsError

    provenance: dict = {}
    if args.manifest is not None:
        try:
            tracks, provenance = _fit_tracks_from_manifest(args, profile)
        except (FitError, TrackError, ValueError, OSError, KeyError) as error:
            print(f"error: {error}")
            return UNUSABLE
    else:
        tracks = [read_hdf5(args.track)]
    # Every run shares the excitation and the joints, so one stands in for all
    # of them wherever only the shape or the names matter.
    track = tracks[0]
    gravity = None
    static = None
    group = None
    if args.static is not None:
        if not args.urdf:
            print(
                "error: --static needs --urdf, to work out the modelled torque at "
                "every sample along the track. Dump it from the running stack "
                "with: ros2 param get --hide-type /robot_state_publisher "
                "robot_description > robot.urdf"
            )
            return UNUSABLE
        try:
            artifact = read_static_estimate(args.static, profile)
        except ArtifactError as error:
            print(f"error: {error}")
            return UNUSABLE
        static = artifact.estimate
        group = profile.groups[artifact.group]
        name = artifact.group
        source_by_canonical = {
            joint.canonical: joint.source for joint in profile.joints
        }
        sources = tuple(source_by_canonical[joint] for joint in group.joints)
        if tuple(track.joint_names) not in (tuple(group.joints), sources):
            print(
                f"error: the track covers {list(track.joint_names)}, but the "
                f"static estimate is for group {name!r}, whose joints are "
                f"{list(group.joints)} ({list(sources)} at the source)"
            )
            return UNUSABLE
        try:
            chain = _group_chain(Path(args.urdf).read_text(), profile, group)
        except (KinematicsError, OSError) as error:
            print(f"error: {error}")
            return UNUSABLE
        # Corrected by the alpha the static fit measured, so the column is the
        # load the arm actually carries rather than the one the URDF describes.
        gravity = [
            static.torque_scale
            * np.array([chain.gravity_torque(sample) for sample in run.measured])
            for run in tracks
        ]

    try:
        estimate = fit_second_order_runs(
            [
                (
                    run.timestamps_ns * 1e-9,
                    run.command,
                    run.measured,
                    None if gravity is None else gravity[index],
                )
                for index, run in enumerate(tracks)
            ],
            # The staircase's measured Coulomb friction, where it ran: the
            # floor under which a gravity variation cannot reach the encoder.
            coulomb_nm=None if static is None else static.coulomb_nm,
        )
    except FitError as error:
        print(f"error: {error}")
        return UNUSABLE

    if estimate.gravity_disagreed:
        named = ", ".join(track.joint_names[index] for index in estimate.gravity_disagreed)
        print(
            f"note: the gravity column disagreed with how {named} moved, so it "
            "was dropped for those joints — they carry no independent inertia "
            "to cross-check. The model's mass or centre of mass is wrong there; "
            "the rest of the arm agreed with it."
        )

    payload: dict = {
        "population": args.population,
        "joint_names": track.joint_names,
        "stiffness": estimate.stiffness.tolist(),
        "damping": estimate.damping.tolist(),
        "friction": estimate.friction.tolist(),
        "residual_rmse": estimate.residual_rmse.tolist(),
        "track_sha256": track_sha256(track),
        **provenance,
    }

    if static is not None:
        try:
            combined = combine(static, estimate, group.joints)
        except FitError as error:
            print(f"error: {error}")
            return UNUSABLE
        _report_combined(combined)
        worst = float(np.nanmax(combined.disagreement))
        if worst > MAX_INERTIA_DISAGREEMENT:
            joint = combined.joint_names[int(np.nanargmax(combined.disagreement))]
            if not args.accept_inertia_gap:
                print(
                    f"refused: nothing written. The two routes to {joint}'s inertia "
                    f"disagree by {worst:.0%}, over {MAX_INERTIA_DISAGREEMENT:.0%}. "
                    "kp/k and 1/g come from different columns of different "
                    "experiments, so a gap that size means one of them is measuring "
                    "something else — a static estimate from another robot, a URDF "
                    "that is not the arm on the track, or a track with a load on it. "
                    "If the URDF's masses are known to be approximate, "
                    "--accept-inertia-gap writes the fit anyway: kp/k rests on the "
                    "staircase's measured kp and stands, while 1/g inherits the "
                    "model's error."
                )
                return REFUSED
            print(
                f"accepted: the two routes to {joint}'s inertia disagree by "
                f"{worst:.0%}, over {MAX_INERTIA_DISAGREEMENT:.0%}, and "
                "--accept-inertia-gap was passed. The written inertia is kp/k; "
                "1/g is recorded beside it so the gap stays visible downstream."
            )
        width = len(group.joints)
        payload.update(
            {
                "group": group.name,
                "stiffness_nm_per_rad": combined.stiffness.tolist(),
                "inertia_kg_m2": combined.inertia.tolist(),
                "damping_nm_s_per_rad": combined.damping.tolist(),
                "friction_nm": combined.friction.tolist(),
                "inertia_from_gravity_kg_m2": combined.inertia_from_gravity.tolist(),
                "inertia_disagreement": combined.disagreement.tolist(),
                # True when the gap was over the threshold and written anyway,
                # so a reader downstream knows the gravity route was not
                # believed here rather than having to rediscover it.
                "inertia_gap_accepted": bool(
                    args.accept_inertia_gap and worst > MAX_INERTIA_DISAGREEMENT
                ),
                "torque_scale": static.torque_scale.tolist(),
                # Carried through so r2s bundle can cite the whole chain without
                # being handed the static estimate a second time.
                "sweep_sha256": list(artifact.sweep_sha256),
                # Fo: always produced by `combine`, whatever the static
                # estimate was.
                "bias_nm": combined.bias.tolist(),
                # Fc and Fo + tau_gravity, straight from the static fit. nan
                # per joint when it never carried them — a gravity sweep never
                # touches a staircase at all — rather than left out, so a
                # reader does not have to know which fit produced this file to
                # know these were not measured.
                "coulomb_nm": (
                    combined.coulomb_nm.tolist()
                    if combined.coulomb_nm is not None
                    else np.full(width, np.nan).tolist()
                ),
                "static_bias_nm": (
                    combined.static_bias_nm.tolist()
                    if combined.static_bias_nm is not None
                    else np.full(width, np.nan).tolist()
                ),
            }
        )
    else:
        # The bias column that replaced the old refusal (see fit_second_order)
        # absorbs a standing load without complaint, so a loaded joint fitted
        # this way gets a fit that runs clean and is wrong.
        print(
            "note: fit ran without a gravity column (no --static/--urdf), so "
            "any standing load on this track has landed in bias rather than "
            "being separated from it. Fine for a level joint or synthetic "
            "data; wrong for a loaded one."
        )

    # nan means "not measured" for coulomb_nm/static_bias_nm (and, on an
    # older fit file read back through this same payload shape, for bias_nm
    # too) — converted to JSON's own null here rather than the non-standard
    # `NaN` token plain json.dumps would otherwise write, and rather than
    # refused: an unmeasured joint is expected, not an error.
    args.output.write_text(
        json.dumps(nan_to_null(payload), indent=2, allow_nan=False) + "\n"
    )
    print(f"fit: {args.output}")
    return 0


#: What a fit output must carry to be merged into a bundle. Anything less was
#: produced without --static, so it holds ratios rather than parameters.
_BUNDLE_KEYS = (
    "group",
    "inertia_kg_m2",
    "damping_nm_s_per_rad",
    "friction_nm",
    "stiffness_nm_per_rad",
    "inertia_from_gravity_kg_m2",
    "inertia_disagreement",
    "torque_scale",
    "sweep_sha256",
    "track_sha256",
)


def _component_of(group, profile) -> str:
    """Which component of the robot a group belongs to.

    Read from the group's own name against the profile's declared components,
    rather than assumed: `validate_holdout` holds arms and hands to different
    thresholds, so putting a run's error under the wrong name would compare it
    against the wrong bound.
    """
    for component in profile.components:
        if group.name.startswith(f"{component}_"):
            return component
    raise ValueError(
        f"group {group.name!r} does not name one of this profile's components "
        f"{list(profile.components)}, so its holdout error cannot be scored "
        "against the right threshold"
    )


def _score_manifest(args, profile) -> tuple[dict, dict]:
    """Score a fitted model against the run the manifest held out."""
    from .kinematics import KinematicsError

    if not args.fit:
        raise ValueError(
            "--manifest names the run to score against, so it needs --fit: the "
            "model being scored"
        )
    manifest = _read_manifest(args.manifest, profile)
    group = _group(profile, manifest["group"])
    names = [entry["path"] for entry in manifest["runs"]]
    holdout = [names[index] for index in manifest["holdout_runs"]]
    if len(holdout) != 1:
        raise ValueError(
            f"exactly one run is held out, this manifest holds out {len(holdout)}"
        )

    rate = profile.endpoint().command_rate_hz
    track = _load_recording(Path(args.manifest).parent / holdout[0], profile, rate)
    estimate = _estimate_from_fit(args.fit, group)

    gravity = None
    if estimate.inverse_inertia is not None:
        if not args.urdf:
            raise ValueError(
                "the fit carries a gravity term, so scoring it needs --urdf to "
                "work out the modelled torque along the holdout"
            )
        try:
            chain = _group_chain(Path(args.urdf).read_text(), profile, group)
        except KinematicsError as error:
            raise ValueError(str(error)) from error
        gravity = np.array([chain.gravity_torque(q) for q in track.measured])

    scored = score_holdout(
        estimate,
        track.timestamps_ns * 1e-9,
        track.command,
        track.measured,
        gravity_torque=gravity,
    )
    component = _component_of(group, profile)
    metrics = {
        # The run says nothing about the other component, so its metric is left
        # at a value that cannot fail rather than invented from this one.
        "openarm_rmse_rad": 0.0,
        "tesollo_rmse_rad": 0.0,
        "delay_residual_sec": scored.delay_residual_sec,
        "command_period_sec": 1.0 / rate,
        "improvement_fraction": scored.improvement_fraction,
    }
    metrics[f"{component}_rmse_rad"] = scored.rmse_rad
    return metrics, {
        "group": group.name,
        "fit_runs": [names[index] for index in manifest["fit_runs"]],
        "holdout_runs": holdout,
        "baseline_rmse_rad": scored.baseline_rmse_rad,
    }


def _read_manifest(path: Path, profile) -> dict:
    manifest = json.loads(Path(path).read_text())
    asset = manifest.get("asset") or {}
    if (
        manifest.get("profile") != profile.name
        or asset.get("id") != profile.asset_id
        or asset.get("manifest_sha256") != profile.manifest_sha256
    ):
        raise ValueError(
            f"the run manifest names profile {manifest.get('profile')!r} and "
            f"asset {asset.get('id')!r}, which are not this profile and asset"
        )
    return manifest


def _fit_tracks_from_manifest(args, profile) -> tuple[list, dict]:
    """Normalize the runs the manifest names for fitting, and cite them."""
    manifest = _read_manifest(args.manifest, profile)
    group = _group(profile, manifest["group"])
    names = [entry["path"] for entry in manifest["runs"]]
    fit_names = [names[index] for index in manifest["fit_runs"]]
    holdout_names = [names[index] for index in manifest["holdout_runs"]]
    overlap = sorted(set(fit_names) & set(holdout_names))
    if overlap:
        raise ValueError(
            f"the manifest fits on {overlap}, which it also holds out. "
            "Validating against a run the model was fitted on validates nothing."
        )
    root = Path(args.manifest).parent
    rate = profile.endpoint().command_rate_hz
    tracks = [_load_recording(root / name, profile, rate) for name in fit_names]
    return tracks, {
        "group": group.name,
        "fit_runs": fit_names,
        "holdout_runs": holdout_names,
    }


def _load_recording(path: Path, profile, rate_hz):
    """Read one `.npz` from collect and put its two streams on a common grid."""
    raw = np.load(path, allow_pickle=False)
    return normalize_track(
        raw["command_time_ns"],
        raw["command"],
        raw["measured_time_ns"],
        raw["measured"],
        list(raw["joint_names"]),
        rate_hz,
    )


def _estimate_from_fit(path: Path, group) -> SecondOrderEstimate:
    payload = json.loads(Path(path).read_text())
    width = len(group.joints)

    def column(key):
        values = np.asarray(payload[key], dtype=float)
        if values.shape != (width,):
            raise ValueError(
                f"{path}: {key} must carry one value per joint of "
                f"{group.name!r}, {width} of them"
            )
        return values

    inverse = None
    if "inertia_kg_m2" in payload:
        inverse = 1.0 / column("inertia_kg_m2")
    # Absent on a fit file written before Fo was exported, and zero is what an
    # absent bias term means rather than a placeholder for a missing
    # measurement. Present, it is Fo in N.m — `combine` multiplied Fo/J by the
    # inertia written beside it — so that same inertia converts it back to the
    # Fo/J this estimate is built from.
    bias = np.zeros(width)
    if "bias_nm" in payload:
        if inverse is None:
            raise ValueError(
                f"{path}: bias_nm is Fo in N.m and this estimate needs Fo/J, "
                "but the file carries no inertia_kg_m2 to divide it by"
            )
        bias = column("bias_nm") * inverse
    return SecondOrderEstimate(
        column("stiffness"),
        column("damping"),
        column("friction"),
        bias,
        column("residual_rmse"),
        inverse,
    )


def _collect_track(args, profile) -> int:
    """Publish an identification excitation and record what the arm did.

    The two streams are kept apart on purpose. A loop that wrote each command
    beside the state it read in the same cycle would be asserting that the state
    responds to that command; it does not, it responds to one from several cycles
    back. That lag is `ControllerCalibration.delay_sec`, a parameter being
    measured, and pairing at record time bakes in zero and destroys it.
    `normalize_track` puts both on a common grid afterwards, which keeps the
    alignment a decision that can still be revised.
    """
    from .ros_adapter import AdapterUnavailable

    if not args.group:
        print("error: --group is required, to say which arm to excite")
        return UNUSABLE
    if args.execute and not args.output:
        print(
            "error: --execute needs --output; the recording is the only thing a "
            "run that moved the robot leaves behind"
        )
        return UNUSABLE
    if args.repetitions not in (1, REPETITIONS):
        print(
            f"error: --repetitions must be 1 or {REPETITIONS}, got "
            f"{args.repetitions}. An identification run needs exactly three: "
            "two to fit and one held out. Two would leave one of each, and a "
            "model fitted on one run has nothing to be validated against."
        )
        return UNUSABLE
    try:
        return _collect_track_run(args, profile)
    except (SafetyError, Refused) as error:
        print(f"refused: {error}")
        return REFUSED
    except AdapterUnavailable as error:
        print(f"unavailable: {error}")
        return UNUSABLE
    except (ValueError, OSError) as error:
        print(f"error: {error}")
        return UNUSABLE


def _collect_track_run(args, profile) -> int:
    from .ros_adapter import RosAdapter

    group = _group(profile, args.group)
    if group.action == PARALLEL_GRIPPER_COMMAND:
        raise ValueError(
            f"group {group.name!r} is driven by a gripper action, which takes a "
            "position rather than a stream, so it cannot be excited this way"
        )
    rate = profile.endpoint().command_rate_hz
    period = 1.0 / rate
    joints = {joint.canonical: joint for joint in profile.joints}
    limits = [joints[canonical] for canonical in group.joints]
    amplitude = np.array(
        [
            (joint.upper - joint.lower) * 0.05 * args.amplitude_scale
            for joint in limits
        ]
    )
    # One period of travel at the profile's velocity limit, which is what the
    # gate will allow between consecutive samples. Handing it to the excitation
    # lets the phase joins be bridged rather than refused.
    budget = np.array([joint.velocity * period for joint in limits])

    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        # Around where the arm is, not the middle of its range: the midpoint of
        # the arms' symmetric limits is the all-zeros pose, so starting there
        # would mean a large unplanned move before the excitation even begins.
        neutral = _start_pose(profile, group, adapter.read_state())
        clock, command, phases = build_excitation(
            neutral, amplitude, rate, max_step=budget
        )
        print(
            f"{group.name}: amplitude_scale={args.amplitude_scale:g} "
            f"samples={len(clock)} ({len(clock) / rate:.1f} s at {rate:g} Hz) "
            f"phases={','.join(dict.fromkeys(phases))}"
        )

        # The whole track, before any of it is published. A run that stopped
        # partway would leave the arm mid-excitation at a velocity nobody chose.
        _gate(profile, group, seed=neutral).authorize_trajectory(
            list(command), start_time_sec=0.0, period_sec=period
        )
        if not args.execute:
            print("DRY RUN: nothing was published; pass --execute to collect")
            return 0

        written: list[Path] = []
        for index in range(args.repetitions):
            if index:
                # Back to where the first run started, or these are not
                # repetitions of the same experiment. The excitation ends
                # wherever its last phase left the arm, not at neutral.
                print(f"\nrun {index}: returning to the starting pose")
                state = _start_pose(profile, group, adapter.read_state())
                points = _gate(profile, group, seed=state).authorize_trajectory(
                    _ramp(state, neutral, DEFAULT_DURATION_SEC, rate),
                    start_time_sec=0.0,
                    period_sec=period,
                )
                adapter.send_trajectory(points, period_sec=period)
            stamps, recording = _publish_excitation(adapter, command, period)
            _report_recording(len(command), recording, rate)
            path = _repetition_path(args.output, index, args.repetitions)
            _write_recording(
                path,
                profile,
                group,
                stamps,
                command[: len(stamps)],
                recording,
            )
            print(f"collect: {path}")
            written.append(path)

    if args.repetitions > 1:
        # Only once every run is on disk. A manifest naming a recording that was
        # never written is worse than no manifest.
        manifest = _write_run_manifest(args.output, profile, group, written)
        print(f"collect: {manifest}")
    return 0


def _repetition_path(output: Path, index: int, repetitions: int) -> Path:
    if repetitions == 1:
        return Path(output)
    output = Path(output)
    return output.with_name(f"{output.stem}{index}{output.suffix}")


def _publish_excitation(adapter, command, period):
    """Stream every sample, recording throughout, and release on any exit."""
    stamps: list[int] = []
    adapter.start_recording()
    try:
        for sample in command:
            cycle = time.monotonic()
            adapter.pump(timeout_sec=0.0)
            # Stamped at publish, not from the planned clock: the planned time
            # is the intent and this is what happened.
            stamps.append(adapter.now_ns())
            adapter.stream_positions(sample)
            time.sleep(max(0.0, period - (time.monotonic() - cycle)))
        adapter.pump(timeout_sec=0.0)
    finally:
        recording = adapter.stop_recording()
    return np.asarray(stamps, dtype=np.int64), recording


def _write_run_manifest(output: Path, profile, group, written: list[Path]) -> Path:
    """Name the recordings, and which of them is held out.

    `split_repetitions` decides the split rather than this function, so the rule
    lives in one place and the bundle's `fit_runs` and `holdout_runs` cite the
    same one.
    """
    output = Path(output)
    names = [path.name for path in written]
    fit, holdout = split_repetitions(names)
    manifest = output.with_suffix(".json")
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "kind": "excitation_runs",
                "profile": profile.name,
                "asset": {
                    "id": profile.asset_id,
                    "manifest_sha256": profile.manifest_sha256,
                },
                "group": group.name,
                "runs": [{"path": name} for name in names],
                "fit_runs": [names.index(name) for name in fit],
                "holdout_runs": [names.index(name) for name in holdout],
            },
            indent=2,
        )
        + "\n"
    )
    return manifest


def _report_recording(published: int, recording, rate: float) -> None:
    period_ns = 1e9 / rate
    print(
        f"published {published} samples, recorded {len(recording)} "
        f"({recording.incomplete} did not cover the group)"
    )
    print(
        f"  largest gap {recording.largest_gap_ns / 1e6:.1f} ms against a "
        f"{recording.median_period_ns / 1e6:.1f} ms median "
        f"({recording.largest_gap_ns / period_ns:.1f} command periods)"
    )
    if not recording.is_monotonic:
        print("  warning: samples arrived out of order; normalize will refuse them")


def _write_recording(path, profile, group, stamps, command, recording) -> None:
    """Write both streams with their own clocks, never resampled or paired."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        command_time_ns=stamps,
        command=np.asarray(command, dtype=float),
        measured_time_ns=recording.timestamps_ns,
        measured=recording.values,
        joint_names=np.array(list(group.joints)),
        profile=np.array(profile.name),
        asset_id=np.array(profile.asset_id),
        manifest_sha256=np.array(profile.manifest_sha256),
        incomplete=np.array(recording.incomplete),
    )


def _bundle(args, profile) -> int:
    """Merge identified parameters from one or more fits into a v2 bundle."""
    if not args.base or not args.fit or not args.output:
        print("error: --base, --fit and --output are required")
        return UNUSABLE
    try:
        base = load_bundle(args.base, profile)
    except (CalibrationError, OSError, ValueError) as error:
        # Matches validate's and export's own catch on the same call: a base
        # that cannot be read is a checked outcome here too, whatever form
        # that unreadability takes (malformed JSON, an asset mismatch, or a
        # bundle old enough to predate a since-added convention).
        print(f"error: {error}")
        return UNUSABLE
    if base.schema_version != 2:
        print(
            f"error: --base is schema v{base.schema_version}; only v2 may be "
            "written, so it cannot carry identified parameters"
        )
        return UNUSABLE

    payload = json.loads(json.dumps(base.payload))
    payload.pop("checksum_sha256", None)
    for path in args.fit:
        try:
            fit = json.loads(Path(path).read_text())
        except (OSError, ValueError) as error:
            print(f"error: {path}: {error}")
            return UNUSABLE
        missing = [key for key in _BUNDLE_KEYS if key not in fit]
        if missing:
            print(
                f"error: {path} carries no {missing[0]}, so it was fitted without "
                "--static. Its stiffness, damping and friction are ratios to an "
                "inertia, not parameters."
            )
            return UNUSABLE
        name = fit["group"]
        if name not in payload.get("groups", {}):
            print(f"error: {path} is for group {name!r}, which this bundle has no entry for")
            return UNUSABLE
        combined = CombinedEstimate(
            joint_names=tuple(profile.groups[name].joints),
            inertia=np.asarray(fit["inertia_kg_m2"], dtype=float),
            damping=np.asarray(fit["damping_nm_s_per_rad"], dtype=float),
            friction=np.asarray(fit["friction_nm"], dtype=float),
            stiffness=np.asarray(fit["stiffness_nm_per_rad"], dtype=float),
            inertia_from_gravity=np.asarray(
                fit["inertia_from_gravity_kg_m2"], dtype=float
            ),
            disagreement=np.asarray(fit["inertia_disagreement"], dtype=float),
            # Optional: a fit file written before these three fields existed
            # has none of them, and identified_block leaves out what it is
            # not given rather than fabricate a nan-filled array for it.
            bias=(
                np.asarray(fit["bias_nm"], dtype=float) if "bias_nm" in fit else None
            ),
            coulomb_nm=(
                np.asarray(fit["coulomb_nm"], dtype=float)
                if "coulomb_nm" in fit else None
            ),
            static_bias_nm=(
                np.asarray(fit["static_bias_nm"], dtype=float)
                if "static_bias_nm" in fit else None
            ),
        )
        try:
            payload["groups"][name]["identified"] = identified_block(
                combined,
                profile,
                torque_scale=fit["torque_scale"],
                sweep_sha256=fit["sweep_sha256"],
                track_sha256=fit["track_sha256"],
            )
        except CalibrationError as error:
            print(f"error: {path}: {error}")
            return UNUSABLE
        if "fit_runs" in fit:
            # Carried from the fit rather than asked for again: the fit is what
            # knows which runs it actually used.
            source = dict(payload.get("source") or {})
            source.update(
                {
                    "track_sha256": fit["track_sha256"],
                    "fit_runs": fit["fit_runs"],
                    "holdout_runs": fit.get("holdout_runs", []),
                }
            )
            payload["source"] = source

    try:
        written = write_bundle(args.output, payload, profile)
    except CalibrationError as error:
        print(f"refused: {error}")
        return REFUSED
    print(
        f"bundle: {args.output}, identified "
        + (", ".join(sorted(written.identified)) or "nothing")
    )
    return 0


def _report_combined(combined) -> None:
    print(
        "  joint            J (kg.m2)   b (N.m.s)   tau_f (N.m)   Fo (N.m)   "
        "kp (N.m/rad)   J from gravity   gap"
    )
    for index, name in enumerate(combined.joint_names):
        print(
            f"  {name:<16} {combined.inertia[index]:9.5f} "
            f"{combined.damping[index]:11.4f} {combined.friction[index]:13.4f} "
            f"{combined.bias[index]:9.4f} "
            f"{combined.stiffness[index]:14.2f} "
            f"{combined.inertia_from_gravity[index]:16.5f} "
            f"{combined.disagreement[index]:5.1%}"
        )


def _report_identified_friction(bundle) -> None:
    """Print both Coulomb measurements a bundle carries, per group and joint.

    A large gap between them is evidence about the model rather than about the
    arm — see hdgp_export's own comment on the same choice — and printing only
    the one export would pick from would hide that gap here too.
    """
    for name in sorted(bundle.identified):
        params = bundle.identified[name]
        coulomb = ", ".join(
            f"{value:.4f}" if np.isfinite(value) else "—"
            for value in params.coulomb_nm
        )
        dynamic = ", ".join(f"{value:.4f}" for value in params.friction)
        print(f"  {name}: coulomb_nm=[{coulomb}] dynamic_friction_nm=[{dynamic}]")


def _collect_poses(args, profile) -> list[Path] | None:
    """Design a pose set, show it, and with --execute sweep at every pose.

    Returns the sweep files written, or None when this was only a review.

    The whole itinerary is authorized before the first move. A run that stopped
    partway because the fifth pose was out of range would leave the arm somewhere
    nobody chose, which is worse than not starting: the point of validating up
    front is that the refusal costs nothing.
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
    if args.poses < 2:
        raise ValueError(
            f"--poses must be at least 2, got {args.poses}; one pose cannot "
            "separate a joint's stiffness from its torque model"
        )
    if args.hold_sec <= 0 or args.duration <= 0:
        raise ValueError("--hold-sec and --duration must be positive")
    scales = _parse_floats(args.scales, "--scales")
    _check_scales(np.asarray(scales, dtype=float), group)

    joints = {joint.canonical: joint for joint in profile.joints}
    limits = [joints[canonical] for canonical in group.joints]
    rate = profile.endpoint().command_rate_hz

    with RosAdapter(profile, args.group, execute=args.execute) as adapter:
        chain = _gravity_chain(adapter, profile, group, args.urdf)
        design = design_pose_set(
            chain.gravity_torque,
            np.array([joint.lower for joint in limits]),
            np.array([joint.upper for joint in limits]),
            scales=scales,
            poses=args.poses,
            seed=args.seed,
            reach=args.reach,
        )
        _report_design(group, design)

        # Both checks before anything moves, cheapest first.
        if design.worst_condition > MAX_CONDITION:
            joint = group.joints[design.worst_joint]
            raise Refused(
                f"the designed poses do not vary {joint}'s load enough to tell "
                f"its stiffness from its torque model: condition "
                f"{design.worst_condition:.3g} over {MAX_CONDITION:g}. Raise "
                "--poses or --reach, or try another --seed."
            )
        start = _start_pose(profile, group, adapter.read_state())
        _gate(profile, group, seed=start).authorize_trajectory(
            list(design.poses), start_time_sec=0.0, period_sec=args.duration
        )

        if not args.execute:
            print(
                "DRY RUN: nothing moved and nothing was written. Nothing here "
                "checks the arm against itself or its surroundings for "
                "collision — the profile bounds each joint, not the arm — so "
                "review the poses above in RViz, then run the same --seed with "
                "--execute."
            )
            return None

        written: list[Path] = []
        rounds = [np.full(len(group.joints), scale) for scale in scales]
        for index, pose in enumerate(design.poses):
            print(f"\npose {index}: moving over {args.duration:g} s")
            state = _start_pose(profile, group, adapter.read_state())
            gate = _gate(profile, group, seed=state)
            points = gate.authorize_trajectory(
                _ramp(state, pose, args.duration, rate),
                start_time_sec=0.0,
                period_sec=1.0 / rate,
            )
            adapter.send_trajectory(points, period_sec=1.0 / rate)
            sweep = _measure_sweep(
                adapter, chain, gate, group, rounds, args.hold_sec, None, None
            )
            path = Path(args.sweep_dir) / f"pose{index}.json"
            write_sweep(path, sweep, profile)
            print(f"wrote {path}")
            written.append(path)
        return written


def _report_design(group, design) -> None:
    """Print the itinerary and how well it conditions each joint's fit."""
    print(
        f"{group.name}: {len(design.poses)} poses, scales "
        + ",".join(f"{scale:g}" for scale in design.scales)
    )
    print("  " + " " * 7 + "".join(f"{canonical:>9}" for canonical in group.joints))
    for index, pose in enumerate(design.poses):
        print(f"  pose {index}" + "".join(f"{value:+9.3f}" for value in pose))
    print("  cond  " + "".join(f"{value:9.1f}" for value in design.condition))
    joint = group.joints[design.worst_joint]
    print(f"  worst conditioned: {joint} at {design.worst_condition:.1f}")


def _static_column(values, index: int) -> str:
    """Format one entry of an optional per-joint array with the file's own
    '—' convention, for Fc/Fo/staircase-kp — none of which every joint has:
    `fit_static_gravity` leaves them `None` outright with no staircase sweep
    at all, and `fit_staircase` leaves a joint it never covered `nan` inside
    the array it does return. Both read the same here.
    """
    if values is None or not np.isfinite(values[index]):
        return "—"
    return f"{values[index]:.4f}"


def _report_static(estimate, poses: int, *, staircase=None) -> None:
    """Print the fit per joint, so a marginal one is visible as such.

    *staircase* is the staircase fit's own `StaticEstimate`, printed only for
    its `stiffness` (a second opinion on kp) and its `unidentifiable`
    (reasons distinct from the gravity fit's own, labelled apart below) — Fc
    and Fo are already folded into *estimate* by the caller before this runs,
    with `dataclasses.replace`, so they are read from *estimate* here like
    every other column.
    """
    # Widths of the gravity-fit columns between kp and the three staircase
    # columns (alpha, offset, residual, cond, rounds, frozen), shared by the
    # full row below and by the short row a fully-unidentified joint prints
    # instead. `blank_middle` is built from these same numbers rather than a
    # counted-by-hand literal, so the short row's padding cannot silently
    # drift out of alignment if any of these widths ever changes.
    ALPHA_WIDTH = 7
    OFFSET_WIDTH = 14
    RESIDUAL_WIDTH = 15
    CONDITION_WIDTH = 6
    USED_WIDTH = 7
    EXCLUDED_WIDTH = 7
    blank_middle = " ".join(
        f"{'':>{width}}"
        for width in (
            ALPHA_WIDTH, OFFSET_WIDTH, RESIDUAL_WIDTH,
            CONDITION_WIDTH, USED_WIDTH, EXCLUDED_WIDTH,
        )
    )

    print(f"identify: {poses} poses, {estimate.used.max()} rounds at most per joint")
    print(
        "  joint            kp (N.m/rad)   alpha   offset (rad)  "
        "residual (rad)   cond  rounds  frozen  staircase kp   Fc (N.m)   Fo (N.m)"
    )
    for index, name in enumerate(estimate.joint_names):
        stiffness = estimate.stiffness[index]
        stair_kp = (
            f"{staircase.stiffness[index]:.2f}"
            if staircase is not None and np.isfinite(staircase.stiffness[index])
            else "—"
        )
        fc = _static_column(estimate.coulomb_nm, index)
        fo = _static_column(estimate.bias_nm, index)
        if not np.isfinite(stiffness):
            print(
                f"  {name:<16} {'—':>12} {blank_middle} "
                f"{stair_kp:>12} {fc:>9} {fo:>9}"
            )
            continue
        # alpha alone can be nan here even though stiffness was identified:
        # a joint whose model was zero at every used round. Same "—" the
        # fully-unidentified row above uses, not a bare nan next to numbers
        # that are all real answers.
        alpha = estimate.torque_scale[index]
        alpha_column = (
            f"{alpha:{ALPHA_WIDTH}.3f}"
            if np.isfinite(alpha)
            else f"{'—':>{ALPHA_WIDTH}}"
        )
        print(
            f"  {name:<16} {stiffness:12.2f} {alpha_column} "
            f"{estimate.offset[index]:+{OFFSET_WIDTH}.5f} "
            f"{estimate.residual_rmse[index]:{RESIDUAL_WIDTH}.5f} "
            f"{estimate.condition[index]:{CONDITION_WIDTH}.1f} "
            f"{estimate.used[index]:{USED_WIDTH}d} "
            f"{estimate.excluded[index]:{EXCLUDED_WIDTH}d} "
            f"{stair_kp:>12} {fc:>9} {fo:>9}"
        )
    for name, reason in estimate.unidentifiable:
        print(f"  {name}: not identified (gravity) — {reason}")
    if staircase is not None:
        for name, reason in staircase.unidentifiable:
            print(f"  {name}: not identified (staircase) — {reason}")


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
        return _collect_track(args, profile)
    elif args.stage == "normalize":
        if not args.input or not args.output:
            raise SystemExit("--input and --output are required")
        try:
            raw = np.load(args.input, allow_pickle=False)
            track = normalize_track(
                raw["command_time_ns"],
                raw["command"],
                raw["measured_time_ns"],
                raw["measured"],
                list(raw["joint_names"]),
                profile.endpoint().command_rate_hz,
                max_gap_periods=args.max_gap_periods,
            )
            write_hdf5(args.output, track)
        except (ArtifactError, TrackError, OSError, KeyError) as error:
            # A missing optional extra and an unusable recording are both
            # answers about the environment, not crashes.
            print(f"error: {error}")
            return UNUSABLE
        print(f"normalize: {args.output} sha256={track_sha256(track)}")
    elif args.stage == "fit":
        if not args.output:
            raise SystemExit("--output is required")
        if not args.track and not args.manifest:
            print(
                "error: fit needs --track, one normalized HDF5 track, or "
                "--manifest, which fits across the runs it names"
            )
            return UNUSABLE
        if args.population <= 0:
            raise SystemExit("--population must be positive")
        return _fit(args, profile)
    elif args.stage == "identify":
        return _identify(args, profile)
    elif args.stage == "bundle":
        return _bundle(args, profile)
    elif args.stage == "validate":
        if not args.bundle:
            raise SystemExit("--bundle is required")
        # A bundle that cannot be read is a checked outcome, not a crash: the
        # whole job of this stage is to say whether a bundle can be trusted.
        try:
            bundle = load_bundle(args.bundle, profile)
        except (CalibrationError, OSError, ValueError) as error:
            print(f"error: {error}")
            return UNUSABLE
        if not args.output:
            raise SystemExit("--output is required")
        extra: dict = {}
        if args.manifest is not None:
            try:
                metrics, extra = _score_manifest(args, profile)
            except (FitError, TrackError, ValueError, OSError, KeyError) as error:
                print(f"error: {error}")
                return UNUSABLE
        elif args.metrics is not None:
            metrics = json.loads(args.metrics.read_text())
        else:
            print(
                "error: validate needs either --metrics, a verdict computed "
                "elsewhere, or --manifest with --fit, which scores the held-out "
                "run itself"
            )
            return UNUSABLE
        result = validate_holdout(**metrics)
        args.output.write_text(
            json.dumps(
                {
                    "status": result.status,
                    "failures": result.failures,
                    "metrics": metrics,
                    **extra,
                },
                indent=2,
            )
            + "\n"
        )
        print(f"validate: schema v{bundle.schema_version}, status={result.status}")
        print(
            "  identified parameters: "
            + (", ".join(sorted(bundle.identified)) or "none")
        )
        _report_identified_friction(bundle)
        for name, value in sorted(metrics.items()):
            print(f"  {name}: {value:.5g}")
        return 0 if result.status == "validated" else 3
    elif args.stage == "export":
        if not args.bundle or not args.validation or not args.output:
            raise SystemExit("--bundle, --validation, and --output are required")
        try:
            bundle = load_bundle(args.bundle, profile)
        except (CalibrationError, OSError, ValueError) as error:
            print(f"error: {error}")
            return UNUSABLE
        validation = json.loads(args.validation.read_text())
        if validation.get("status") != "validated":
            print("export blocked: model_inadequate")
            return 3
        args.output.write_bytes(args.bundle.read_bytes())
        print(f"export: {args.output}")
        if args.hdgp:
            try:
                payload = write_hdgp_calibration(
                    args.hdgp, bundle, profile, max_spread=args.hdgp_max_spread
                )
            except (HdgpExportError, OSError) as error:
                print(f"error: {error}")
                return UNUSABLE
            print(f"export hdgp: {args.hdgp}")
            for name in sorted(payload["groups"]):
                body = payload["groups"][name]
                print(
                    f"  {name}: stiffness={body['stiffness']:.4g} "
                    f"damping={body['damping']:.4g} "
                    f"friction={body['joint_friction']:.4g}"
                )
            # Named, not silent: these keep the env's own gain, and the run is
            # only partly calibrated as a result.
            defaulted = sorted(
                group.hdgp_group
                for name, group in profile.groups.items()
                if group.hdgp_group and group.hdgp_group not in payload["groups"]
            )
            if defaulted:
                print(f"  left at the env's defaults: {', '.join(defaulted)}")
    else:
        print(f"{args.stage}: profile={profile.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
