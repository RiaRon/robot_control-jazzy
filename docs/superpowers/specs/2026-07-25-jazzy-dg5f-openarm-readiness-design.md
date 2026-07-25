# Jazzy DG5F and OpenArm Readiness Design

## Goal

Make the `jazzy` branch reproducibly buildable on Ubuntu 24.04 with ROS 2
Jazzy, validate Tesollo DG5F in fake hardware and Gazebo Harmonic, and prepare
OpenArm for a later supervised hardware connection without issuing hardware
commands during this work.

## Scope

- Tesollo model: DG5F only.
- DG5F validation: xacro, fake hardware, headless Gazebo, controller
  activation, 20-joint coverage, and a bounded trajectory/state round trip.
- OpenArm validation: dependency resolution, build, description/controller
  checks, and fake-hardware bringup only.
- DG3F-M and DG4F remain in the imported snapshot but are excluded from the
  branch's supported Jazzy build and validation set.
- No CAN transmission, real Tesollo connection, or real OpenArm motion is
  authorized.

## Upstream and Local Patch Ownership

The complete Tesollo upstream tree remains pinned to
`tesollodelto/delto_m_ros2@3926c2eab8d011046f64874d6252213b2cf18f48`.
Jazzy compatibility changes are project-owned patches, not undocumented edits.

`vendor_metadata/tesollo/UPSTREAM.yaml` records:

- the upstream repository, branch, and commit;
- the ordered patch files applied to that commit;
- the post-patch SHA-256 for every locally changed file.

The snapshot verifier reconstructs the expected tree by applying the declared
patches to a clean pinned source tree, then compares every included file. It
must fail for an undeclared edit, a missing patch, a patch application failure,
or a post-patch hash mismatch. This preserves rollback and auditability while
allowing the checked-in `ros_ws/src` tree to build directly.

## DG5F Jazzy Migration

Only these DG5F files are migrated:

- `dg5f_gz/package.xml`
- `dg5f_gz/urdf/dg5f_right_gz.xacro`
- `dg5f_gz/urdf/dg5f_left_gz.xacro`
- `dg5f_gz/urdf/dg5f_both_gz.xacro`
- DG5F launch files when Jazzy launch compatibility requires it

The migration uses the ROS 2 Jazzy and Gazebo Harmonic contract:

- package dependency: `gz_ros2_control`
- ros2_control hardware plugin:
  `gz_ros2_control/GazeboSimSystem`
- Gazebo system library: `libgz_ros2_control-system.so`
- Gazebo plugin class:
  `gz_ros2_control::GazeboSimROS2ControlPlugin`
- resource path: `GZ_SIM_RESOURCE_PATH`, without relying on the legacy
  `IGN_GAZEBO_RESOURCE_PATH`

Controller configuration remains position-command based. Existing joint names
and limits are preserved. The migration does not tune dynamics or change the
physical model.

## Branch Build Contract

The default Jazzy build includes:

- OpenArm packages;
- OpenArm CAN and description;
- Tesollo hardware/common packages;
- DG5F driver and DG5F Gazebo package.

DG3F-M and DG4F Gazebo packages are excluded by explicit package selection,
not deleted. The build wrapper continues to reject any `ROS_DISTRO` other than
`jazzy` and keeps all generated products under `ros_ws/{build,install,log}`.

An idempotent dependency helper prints the exact Ubuntu 24.04/Jazzy packages
and invokes apt/rosdep only when explicitly run by the operator. It does not
embed credentials or bypass sudo. `libcli11-dev` is included because
`openarm_can` requires CLI11 in CMake but does not currently expose a working
rosdep resolution on the target host.

## Validation

Static tests verify:

- no DG5F package/xacro/launch file contains legacy
  `ign_ros2_control`, `IgnitionSystem`, `IgnitionROS2ControlPlugin`,
  `libign_ros2_control`, or `IGN_GAZEBO_RESOURCE_PATH`;
- all right-hand DG5F canonical joints exist exactly once in the xacro and
  controller configuration;
- plugin names and package dependencies match the Jazzy contract;
- the build wrapper selects the supported package set;
- patched snapshot provenance is fail-closed.

Runtime tests are split into two commands:

1. Fake-hardware smoke test: bring up controllers without any physical
   interface, verify controller states and joint coverage, send a bounded
   trajectory, and verify returned joint state.
2. DG5F Gazebo smoke test: launch Gazebo Harmonic headlessly, spawn the right
   DG5F, wait with bounded timeouts for controller activation, send the same
   bounded trajectory, verify state feedback, and always terminate child
   processes.

A smoke test fails on timeout, missing joints, inactive controllers, NaN
states, excessive position error, or unexpected process exit. Hardware
interfaces are never used as fallback.

## OpenArm Safety Boundary

OpenArm readiness ends at build, URDF/controller validation, and fake-hardware
smoke testing. A future hardware run must be a separate documented procedure
with:

- operator present;
- E-stop verified;
- robot workspace cleared;
- CAN interface explicitly named and checked;
- low gain and velocity limits;
- an explicit execution flag;
- a hold/disable path confirmed before motion.

No test in this implementation may open a CAN interface or publish to a real
hardware controller.

## Completion Criteria

- Core Python tests pass on Python 3.12.
- All four vendor snapshots pass provenance verification, including declared
  Tesollo Jazzy patches.
- `rosdep check` has no unresolved keys for the supported package set.
- Supported packages complete `colcon build` on Ubuntu 24.04/Jazzy after the
  documented dependency installation.
- DG5F fake-hardware and headless Gazebo smoke tests pass with all 20 joints.
- OpenArm fake-hardware smoke test passes without opening CAN.
- The verification record distinguishes completed checks from checks blocked
  by missing operator-level system packages.
