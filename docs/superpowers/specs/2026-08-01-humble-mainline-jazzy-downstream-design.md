# Humble Mainline, Jazzy Downstream Integration Design

**Date:** 2026-08-01

## Goal

Keep `humble` unchanged as the canonical mainline and rebuild Jazzy as a
downstream distribution layer. Jazzy must consume Humble's latest robot-control,
tuning, gravity/inertia identification, and Real2Sim implementation while
retaining its MoveIt/RViz operator workflow and the current uncommitted
GUI/servo work. The first acceptance target is operation without physical
hardware.

## Repository State and Constraints

- `origin/humble@b5dee53` is the canonical implementation baseline.
- The valid committed Jazzy distribution baseline is local `jazzy@2433a30`.
- Current `origin/jazzy@d6bed9a` is not a usable Jazzy baseline: its tree declares
  Humble in `.rosdistro`, the README, and the build wrapper.
- The Jazzy worktree contains uncommitted pose-follow, IK-follow, servo, CLI,
  ROS-adapter, tests, and documentation changes. These are user work and must be
  preserved independently before integration.
- The root Humble checkout contains an untracked `guide.md`. It must not be
  modified, staged, or removed.
- No push or remote branch rewrite is part of this integration. Remote repair is
  a separate, explicit operation after local review.
- Commands that can publish to physical controllers remain opt-in through
  `--execute`; verification must not use CAN or publish real torque.

## Selected Approach

Create an isolated integration branch from the latest `origin/humble`, then
overlay only the Jazzy-owned distribution surface and the preserved local
GUI/servo work.

This direction makes ancestry express ownership: Humble is upstream, and Jazzy
is a small downstream delta. It avoids selecting dozens of interdependent
Humble commits and avoids merging the current mislabeled `origin/jazzy` tree.

## Ownership Boundary

### Humble-owned common core

The integration starts with these areas exactly as supplied by the latest
Humble baseline, except where a small Jazzy compatibility adapter is required:

- `src/robot_control` Real2Sim collection, normalization, identification,
  fitting, bundling, validation, and export;
- direct-torque excitation and stiffness/friction/bias estimation;
- gravity and inertia fitting, including canonical asset URDF support;
- artifact checksums, provenance, calibration loading, and HDGP export;
- canonical joint profiles and safety gates;
- common offline and mocked-ROS tests.

Common behavior fixes belong upstream in Humble first. Jazzy-specific code must
not fork the numerical identification or safety implementation.

### Jazzy-owned downstream surface

The following are restored from the valid Jazzy baseline and adapted to the
new common core:

- `.rosdistro` with `jazzy`;
- Jazzy build and dependency wrappers;
- Jazzy OpenArm and Tesollo vendor metadata and patches;
- Gazebo Harmonic and `gz_ros2_control` launch/configuration;
- MoveIt/RViz configuration, `pose_bringup.sh`, and operator documentation;
- Jazzy contract, dependency, build-wrapper, and smoke tests;
- distribution-specific controller actions, SRDF/TCP frames, and ROS endpoints.

The downstream delta must not restore older copies of common Python modules.

### Current local GUI/servo work

Before rebuilding the branch, all tracked and relevant untracked work in the
current Jazzy worktree is captured without including unexplained scratch files.
The integration then reapplies the intentional changes:

- `src/robot_control/ik_follow.py` and `servo.py`;
- pose-follow CLI and ROS-adapter changes;
- corresponding unit tests;
- pose-follow and realtime TCP-follow documentation/specifications.

Untracked files whose names are numeric or shell-fragment-like are quarantined
from the integration until their contents and purpose are identified. They are
not deleted.

## Runtime Architecture

RViz remains a goal-selection GUI, not an independent command path:

```text
RViz MotionPlanning interactive marker
    -> robotctl marker reader
    -> one-shot MoveIt IK or continuous IK/Jacobian servo
    -> common safety gate
    -> Jazzy ros2_control adapter
    -> dry-run report or, only with --execute, controller action/topic
```

Real2Sim and tuning use the same profile, canonical joint ordering, artifact
formats, and safety limits as pose control. Jazzy adapters translate only ROS
interfaces; they do not duplicate fitting or control policy.

## Compatibility Rules

- The profile selects endpoints from `.rosdistro`; no common module hard-codes
  Humble or Jazzy.
- Jazzy controller/action declarations must match the restored Jazzy bringup.
  In particular, gripper actions are not copied from Humble without checking
  their available controller plugins.
- MoveIt marker names and IK tip links must match the Jazzy SRDF. The canonical
  asset URDF's `asset_tip_link` remains distinct from the MoveIt `tip_link`.
- Missing ROS, RViz, marker server, action server, or controller produces a
  clear unavailable/configuration result. It must not silently enable publish.
- `--execute` remains the sole CLI authority to move hardware. Dry-run paths may
  calculate targets and inspect configuration but never publish commands.

## Error Handling and Preservation

- Integration happens in a new worktree; neither the Humble checkout nor the
  dirty Jazzy worktree is reset or cleaned.
- User changes are captured before any branch reconstruction and verified by
  file list and diff statistics.
- A conflict in common Python code is resolved against Humble behavior first,
  then the GUI/servo caller is adapted to the resulting interface.
- A conflict in ROS packages or launch files is resolved against the valid
  Jazzy surface.
- Tests that require unavailable ROS binaries are reported separately from
  actual test failures.

## Dry-run Acceptance Criteria

Without physical hardware, the integration must demonstrate:

1. `.rosdistro` declares Jazzy and wrappers reject the wrong ROS distribution.
2. The full Python unit suite passes, with environment-dependent skips stated.
3. `robotctl r2s preflight` succeeds against the configured asset inputs.
4. `robotctl r2s collect --dry-run` produces a plan without publishing.
5. Pose commands expose usable dry-run behavior with mocked state/IK/marker
   backends.
6. One-shot marker control and continuous follow/servo tests cover marker
   updates, joint/velocity clamps, unreachable targets, and stop behavior.
7. Jazzy build, dependency, vendor, MoveIt, controller, and Gazebo contracts
   pass static tests.
8. If the installed Jazzy environment permits it, packages build and
   fake-hardware/headless smoke tests pass under bounded timeouts.
9. No CAN interface is opened and no real trajectory or effort command is
   published during verification.

## Deferred Physical Validation

Physical validation is a later gate and does not block this integration. Its
checklist will cover CAN discovery, controller activation, E-stop readiness,
low-speed one-shot marker motion, continuous follow error, gravity-compensation
torque limits, excitation bounds, artifact capture, and safe shutdown.

## Deliverables

- An isolated Jazzy integration branch based on the latest Humble commit.
- Restored and tested Jazzy distribution, GUI, MoveIt, and Gazebo surface.
- Preserved and integrated local GUI/servo work.
- Humble's current tuning and Real2Sim implementation running on Jazzy.
- A dry-run verification record and a separate deferred physical checklist.
- No modification to the Humble branch and no remote push.
