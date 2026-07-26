# Pose Setting and ROS Adapter Design

## Goal

Give `robot_control` the missing execution edge: a project-owned ROS adapter
that turns canonical commands into controller traffic, and an operator-facing
pose-setting surface that reaches a target either by direct joint values or by
end-effector inverse kinematics. The interactive case is an operator dragging
the 6-DoF end-effector marker in RViz and committing the result.

This closes the gap the README already names: the core CLI deliberately fails
without a ROS adapter, and no adapter exists.

## Context and Role of This Directory

`robot_control` executes commands that a learned policy issues; it does not
choose them. `sim2real` owns policy execution and task orchestration, `hdgp`
owns assets and RL. Only the lower-level control contract lives here, plus the
Real2Sim artifact pipeline that measures the real robot so simulation can be
corrected against it.

Pose setting serves that pipeline rather than being a separate product. Real2Sim
identification runs from a known starting configuration, and an operator needs a
reliable way to put the robot there and to nudge it between runs. The same
canonical boundary must carry both the policy path and the operator path, or the
two will disagree about joint order, sign, and limits.

## Scope

- Target configuration: OpenArm bimanual, matching
  `src/robot_control/profiles/openarm_tesollo.yaml`.
- Planning model: plan-then-execute through MoveIt. Continuous jogging
  (`moveit_servo`) is out of scope.
- Motion sources: direct canonical joint values, SRDF named states, and
  end-effector pose through `/compute_ik`.
- Execution: `FollowJointTrajectory` to the existing per-arm controllers.
- Default hardware mode is fake. Real hardware requires explicit opt-in.
- Tesollo DG5F is addressable by direct joint values only; no hand IK.
- No CAN transmission and no real robot motion are authorized by this work.

## Verified Starting Point

Checked on 2026-07-26 against the built `jazzy` workspace with
`use_fake_hardware:=true`:

- `openarm_bimanual_moveit_config` provides groups `left_arm`, `right_arm`,
  `left_gripper`, `right_gripper`; end effectors `left_ee` and `right_ee` with
  parent links `openarm_left_hand` and `openarm_right_hand`; KDL IK for both
  arms.
- `move_group`, `/compute_ik`, `/compute_fk`, and `/plan_kinematic_path` come
  up, and all five controllers reach `active`.
- FK and IK both resolve for each arm. A 3 cm vertical end-effector offset
  solved with `avoid_collisions=true`.
- The SRDF joint names `openarm_{left,right}_joint1..7` and the DG5F names
  `rj_dg_*_*` match the profile's `source` names exactly, so the canonical
  layer already lines up with the real robot.

Two environment facts constrain the design:

- `demo.launch.py` defaults `use_fake_hardware` to `false`, so an operator who
  omits the argument opens `can0` and `can1`. The project wrapper must invert
  this default.
- `sensors_3d.yaml` references `occupancy_map_monitor/DepthImageOctomapUpdater`,
  which is not installed. Depth-sensor octomap collision is therefore
  unavailable; kinematics and planning are unaffected.

## Architecture

```text
learned policy (sim2real)  ─┐
                            ├─→ canonical command ─→ CommandGate ─→ RosAdapter ─→ controllers
operator pose setting ─────┘         (ndarray)        (safety)       (ROS)
   │                                     ▲
   ├─ --joints                           │
   ├─ --named  (SRDF group_state)        │
   └─ --ee-pose ─→ /compute_ik ──────────┘
```

Every path converges on `CanonicalInterface` and `CommandGate` before anything
is published. Pose setting adds no second route to the hardware.

### Canonical boundary

`CanonicalInterface` already maps canonical order and sign to source joint
names and back. Pose setting reuses it unchanged. IK returns a solution keyed by
source joint names, which is exactly what `state_to_canonical` consumes, so the
solution re-enters the canonical order before the gate sees it.

### Safety boundary

`CommandGate` already enforces position limits, velocity limits, a watchdog,
E-stop, and refusal without `execute`. Pose setting adds two obligations:

- A pose target is authorized as a whole. Every waypoint of the planned
  trajectory passes `authorize` before any point is sent; a rejection anywhere
  discards the entire trajectory rather than sending a truncated one.
- Profile limits, not URDF limits, are authoritative. The profile is the
  contract shared with `sim2real`; a solution that satisfies the URDF but
  violates the profile is rejected.

### ROS adapter

`RosAdapter` is a thin, optional layer. Importing `robot_control` must not
require `rclpy`, so the adapter lives in its own module and the CLI imports it
lazily, keeping the existing "core CLI fails without an adapter" contract as an
explicit, tested error rather than an import crash.

Responsibilities:

- Subscribe to `/joint_states` and expose the latest canonical state.
- Send a canonical trajectory through `FollowJointTrajectory` to the controller
  that owns those joints, resolved from the profile group.
- Call `/compute_ik` for an end-effector pose and return a canonical solution.
- Refuse to construct at all when `execute` was not requested.

Group-to-controller mapping is profile data, not code. The profile gains an
optional `controller` and `moveit_group` per group; absent entries mean the
group is not executable and the CLI says so.

## CLI Surface

A new `robotctl pose` command, consistent with the existing `r2s` stages:
`--dry-run` is the default, `--execute` is required to publish, and the
non-execute path prints exactly what would be sent.

```text
robotctl pose show    [--profile P] [--group G]
robotctl pose joints  --group G (--values v1,.. | --named NAME) [--execute]
robotctl pose ee      --group G --xyz x,y,z [--rpy r,p,y] [--relative] [--execute]
robotctl pose rviz    [--profile P] [--real]
```

- `show` reports the current canonical state and, for an arm group, the
  end-effector pose from `/compute_fk`.
- `joints` sets a group directly, either from explicit values or from an SRDF
  named state such as `home`.
- `ee` solves IK for a target pose and executes the plan. `--relative` treats
  the argument as an offset from the current end-effector pose, which is the
  common case when nudging a setup.
- `rviz` launches the bimanual MoveIt stack with `use_fake_hardware:=true`, the
  inverted default. `--real` is the only way to reach hardware and must state
  the CAN interfaces explicitly.

Exit codes follow the existing convention: `0` success, `2` missing backend or
unusable request, `3` refused by policy such as a failed IK solve or a gate
rejection.

## Deliverable Documentation

`docs/cli.md` is a required deliverable, not a byproduct. It documents every
`robotctl` command that exists after this work, including the `r2s` stages that
are currently undocumented outside the README's four-line excerpt. It contains
for each command: purpose, full argument list with defaults, exit codes, a
worked example with real output, and the safety default in force.

It opens with the operator flow for the interactive case end to end: launch the
stack, drag the end-effector marker in RViz, read the pose back with
`pose show`, and commit it with `pose ee --execute`.

A test asserts that every subcommand and option in the parser appears in
`docs/cli.md`, so the document cannot silently fall behind the code.

## Validation

- Unit tests cover canonical/IK conversion, gate authorization of a whole
  trajectory, controller resolution from the profile, and the adapter's refusal
  without `execute`. These run without ROS.
- An integration smoke script launches the bimanual stack with fake hardware,
  commands a bounded pose through the CLI, and asserts the state converges,
  reusing the existing fail-closed harness.
- The documentation test keeps `docs/cli.md` synchronized with the parser.
- No test may open CAN or command real hardware.

## Completion Criteria

- `robotctl pose` reaches a target by joint values, named state, and
  end-effector IK, with `--dry-run` as the default in every case.
- An operator can drag the RViz end-effector marker and commit the pose without
  editing a launch file, and cannot reach hardware without an explicit flag.
- Policy commands and operator commands pass through the same canonical
  interface and the same safety gate.
- `docs/cli.md` documents every command, and the parser/document test passes.
- The full suite, the vendor snapshot verification, and the existing three
  smoke tests still pass.

## Deferred

- `moveit_servo` continuous jogging.
- Tesollo DG5F end-effector IK.
- Depth-sensor octomap collision, which needs `moveit_ros_perception`.
- Real-time scheduling for the controller manager, currently unavailable to an
  unprivileged process and relevant only for real hardware.
