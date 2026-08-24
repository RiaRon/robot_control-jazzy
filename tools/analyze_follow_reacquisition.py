"""Validate the deterministic-follow A-prime reacquisition neighbourhood.

The offline lattice covers every {-0.05, 0, +0.05} joint combination for
limits and Jacobian metrics. With --ros it additionally asks MoveIt's fake
planning scene about the centre, all single-joint extrema, and every hypercube
corner, then repeats six-axis IK requests from every single-joint extremum.
Nothing is published.
"""

from __future__ import annotations

import argparse
from itertools import product
import json
from pathlib import Path

import numpy as np

from robot_control.cli import _group_chain, _quaternion_from_rotation
from robot_control.kinematics import _rotation
from robot_control.profile import load_builtin_profile
from robot_control.ready import (
    FOLLOW_REACQUISITION_TOLERANCE_RAD,
    READY_TARGET_RAD,
)
from robot_control.ros_adapter import IkFailed, Pose, RosAdapter


def _joint_limits(profile, group):
    by_name = {joint.canonical: joint for joint in profile.joints}
    return [by_name[name] for name in group.joints]


def _neighbourhood():
    tolerance = FOLLOW_REACQUISITION_TOLERANCE_RAD
    for offset in product((-tolerance, 0.0, tolerance), repeat=7):
        yield READY_TARGET_RAD + np.asarray(offset)


def _collision_samples():
    tolerance = FOLLOW_REACQUISITION_TOLERANCE_RAD
    yield READY_TARGET_RAD.copy()
    for joint in range(7):
        for sign in (-1.0, 1.0):
            sample = READY_TARGET_RAD.copy()
            sample[joint] += sign * tolerance
            yield sample
    for signs in product((-1.0, 1.0), repeat=7):
        yield READY_TARGET_RAD + tolerance * np.asarray(signs)


def _seed_samples():
    yield READY_TARGET_RAD.copy()
    tolerance = FOLLOW_REACQUISITION_TOLERANCE_RAD
    for joint in range(7):
        for sign in (-1.0, 1.0):
            sample = READY_TARGET_RAD.copy()
            sample[joint] += sign * tolerance
            yield sample


def _pose(matrix):
    return Pose(
        tuple(float(value) for value in matrix[:3, 3]),
        _quaternion_from_rotation(matrix[:3, :3]),
        "world",
    )


def _ik_goals(chain, seed):
    origin = chain.pose(seed)
    for axis in range(3):
        moved = origin.copy()
        moved[axis, 3] += 0.01
        yield f"world_{'xyz'[axis]}_10mm", _pose(moved)
    angle = np.deg2rad(5.0)
    axes = np.eye(3)
    for axis in range(3):
        moved = origin.copy()
        moved[:3, :3] = origin[:3, :3] @ _rotation(axes[axis], angle)
        yield f"local_{'xyz'[axis]}_5deg", _pose(moved)


def analyze(*, use_ros):
    profile = load_builtin_profile("openarm_tesollo")
    group = profile.groups["openarm_right_arm"]
    limits = _joint_limits(profile, group)
    lower = np.array([joint.lower for joint in limits])
    upper = np.array([joint.upper for joint in limits])

    adapter = None
    if use_ros:
        adapter = RosAdapter(profile, group.name, execute=False)
        urdf = adapter.read_robot_description(timeout_sec=5.0)
    else:
        if profile.asset_urdf_path is None:
            raise RuntimeError("profile has no offline asset URDF")
        urdf = profile.asset_urdf_path.read_text()
    chain = _group_chain(urdf, profile, group)

    ranks = []
    sigma_min = []
    conditions = []
    minimum_margin = float("inf")
    limit_failures = []
    for index, joints in enumerate(_neighbourhood()):
        margin = np.minimum(joints - lower, upper - joints)
        minimum_margin = min(minimum_margin, float(np.min(margin)))
        if np.any(margin < 0):
            limit_failures.append(index)
        singular = np.linalg.svd(chain.jacobian(joints), compute_uv=False)
        rank = int(np.sum(singular > 1e-9))
        ranks.append(rank)
        sigma_min.append(float(singular[-1]))
        conditions.append(float(singular[0] / singular[-1]))

    result = {
        "posture_name": "openarm_right_ready_v2",
        "target_rad": READY_TARGET_RAD.tolist(),
        "tolerance_rad": FOLLOW_REACQUISITION_TOLERANCE_RAD,
        "offline_lattice": {
            "samples": len(ranks),
            "joint_limit_failures": len(limit_failures),
            "minimum_joint_limit_margin_rad": minimum_margin,
            "minimum_jacobian_rank": min(ranks),
            "minimum_sigma": min(sigma_min),
            "maximum_condition_number": max(conditions),
        },
        "ros_fake": None,
    }

    if adapter is None:
        return result

    try:
        collision_failures = []
        collision_count = 0
        for index, joints in enumerate(_collision_samples()):
            valid, contacts = adapter.check_state_validity(joints, timeout_sec=2.0)
            collision_count += 1
            if not valid:
                collision_failures.append(
                    {"sample": index, "contacts": list(contacts)}
                )

        ik_failures = []
        ik_requests = 0
        worst_seed_delta = 0.0
        worst_repeat_delta = 0.0
        solution_limit_failures = 0
        for seed_index, seed in enumerate(_seed_samples()):
            for goal_name, goal in _ik_goals(chain, seed):
                previous = None
                for repetition in range(2):
                    ik_requests += 1
                    try:
                        solution = adapter.solve_ik(
                            goal,
                            seed,
                            timeout_sec=1.0,
                        )
                    except IkFailed as error:
                        ik_failures.append(
                            {
                                "seed": seed_index,
                                "goal": goal_name,
                                "repetition": repetition,
                                "error": str(error),
                            }
                        )
                        continue
                    delta = np.abs(solution - seed)
                    worst_seed_delta = max(worst_seed_delta, float(np.max(delta)))
                    if np.any(solution < lower) or np.any(solution > upper):
                        solution_limit_failures += 1
                    if previous is not None:
                        worst_repeat_delta = max(
                            worst_repeat_delta,
                            float(np.max(np.abs(solution - previous))),
                        )
                    previous = solution

        result["ros_fake"] = {
            "collision_samples": collision_count,
            "collision_failures": collision_failures,
            "ik_requests": ik_requests,
            "ik_failures": ik_failures,
            "ik_solution_limit_failures": solution_limit_failures,
            "worst_seed_to_solution_delta_rad": worst_seed_delta,
            "worst_repeat_solution_delta_rad": worst_repeat_delta,
            "continuity_boundary_rad": 0.30,
        }
        return result
    finally:
        adapter.close()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = analyze(use_ros=args.ros)
    encoded = json.dumps(result, indent=2) + "\n"
    if args.output is not None:
        args.output.write_text(encoded)
    print(encoded, end="")
    offline = result["offline_lattice"]
    failed = (
        offline["joint_limit_failures"]
        or offline["minimum_jacobian_rank"] < 6
        or result["ros_fake"] is not None
        and (
            result["ros_fake"]["collision_failures"]
            or result["ros_fake"]["ik_failures"]
            or result["ros_fake"]["ik_solution_limit_failures"]
            or result["ros_fake"]["worst_seed_to_solution_delta_rad"] >= 0.30
        )
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
