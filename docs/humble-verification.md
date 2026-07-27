# Humble Snapshot Verification

Date: 2026-07-25

## Imported sources

| Component | Revision | Snapshot result |
|---|---|---|
| OpenArm ROS 2 | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | verified |
| OpenArm CAN | `c32ecd31da267967f0c913c2118c843177d88b91` | verified |
| OpenArm description | `teleopration_openarm_tesollo@c8696ebfd64ea08ee0a212a9bae21055b6f381bc`, `src/openarm_description` | verified |
| Tesollo Delto M ROS 2 | `a68335919ee490d5293581574acc7aff12fe969d` | verified |

Verification compares all non-excluded files by relative path and SHA-256.
All four source repositories were clean before import and remained unchanged.
No nested Git metadata or files larger than 90 MB were imported.

## Package discovery

`colcon list --base-paths ros_ws/src` discovers 18 local packages:

- OpenArm: `openarm_can`, `openarm_description`, `openarm`,
  `openarm_hardware`, `openarm_bringup`, and
  `openarm_bimanual_moveit_config`
- Tesollo: `delto_hardware`, `delto_tcp_comm`, `dg_description`, `dg_msgs`,
  `dg_sdk_ros2_bridge`, four hardware driver packages, and three Gazebo
  packages

The previously missing `openarm_can` and `openarm_description` dependencies
are now resolved by branch-local source packages.

## Environment limitation

The current verification host is Ubuntu 24.04 with ROS 2 Jazzy. This branch's
build wrapper correctly rejects that environment. A Humble `colcon build`
was therefore not attempted on this host.

Running `rosdep check` under Jazzy reports Jazzy packages plus unresolved
legacy Humble keys `ign_ros2_control` for the Tesollo Gazebo packages. That
result is not a Humble compatibility verdict; it confirms that dependency
resolution must be repeated on Ubuntu 22.04 with `/opt/ros/humble` sourced.

Required Humble-host commands:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros_ws/src --ignore-src -r -y
./ros_ws/build.sh
```

No real-hardware command was executed during this migration.

## Core brought forward from Jazzy

Date: 2026-07-26. Against
`docs/superpowers/plans/2026-07-26-humble-parity.md`, Tasks 1 to 3.

Merged from `jazzy` rather than copied, so the neutral modules share history and
a future fix on either branch merges cleanly. What arrived:

- `src/robot_control/`: `kinematics`, `layout`, `srdf`, `ros_adapter` are new
  here, and `artifacts`, `calibration`, `cli`, `identification`, `interface`,
  `profile`, `safety`, `track` are brought forward. `cli.py` goes from 137 lines
  to 1991.
- The tests for all of the above.
- The plan and design documents.

What was deliberately **not** merged, because it is the surface the branches
exist to separate: `.rosdistro`, `README.md`, `ros_ws/`, `vendor_metadata/`, and
the tests that exercise Jazzy's shell scripts. `docs/cli.md` was held back too —
it documents `ros_ws/pose_bringup.sh`, which does not exist here, and shipping a
reference that tells a Humble operator to source Jazzy would be worse than
having none. Tasks 4 and 6 bring the scripts and the documentation together.

### Verified here

`339 passed, 2 skipped`, on this host's Python 3.12. The whole Real2Sim pipeline
runs offline:

```text
$ robotctl r2s preflight
profile: openarm_tesollo
asset: openarm_tesollo_sensor_rl
joints: 35
publish_enabled: false

$ python3 -c "...declared_distro(), profile.endpoint().command_rate_hz"
humble 100 Hz
```

That second line is the point of Task 1: the same source file that resolves to
Jazzy's endpoint on the other branch resolves to Humble's here, because it reads
`.rosdistro` rather than naming a distribution.

`r2s preflight`, `identify`, `fit`, `bundle`, `validate` and `export` need no
ROS at all, so the identification half of the pipeline is verifiable on this
branch today. `collect` and every `pose` command need the adapter, which is
Task 4.

### Finding: this branch's vendored MoveIt configuration has no TCP frames

The two skipped tests are not a gap in the port. The profile names
`openarm_right_hand_tcp` and `openarm_left_hand_tcp` as its tip links, and this
branch's vendored `openarm_ros2` (`main` @ `4e837e1`) declares:

```xml
<end_effector name="right_ee" parent_link="openarm_right_hand" group="right_gripper"/>
<end_effector name="left_ee"  parent_link="openarm_left_hand"  group="left_gripper"/>
```

The string `openarm_right_hand_tcp` does not appear in that SRDF at all. Jazzy's
snapshot (`jazzy` @ `8087bbc`) does declare it.

This matters more than a naming difference. MoveIt anchors the interactive
end-effector marker at the end effector's parent link and names it
`EE:goal_<parent_link>`, so on this branch's snapshot the marker would be
`EE:goal_openarm_right_hand`. `pose ee --from-marker` and `pose follow` would
look for a marker that does not exist, and the failure would arrive as a service
timeout rather than as anything explaining itself.

It also means the tip is 0.18 m short of the tool centre point — the same
distance Jazzy's verification records for the fallback case.

**Resolution belongs to Task 5**, the Humble vendor snapshot: re-importing
`openarm_ros2` may bring the TCP frames, and if it does not, the profile needs a
tip link this branch's tree actually declares. Nothing downstream should be
built on the current value.

### Still not verified on Humble

Everything that needs a Humble host. This host is Ubuntu 24.04 with Jazzy, so
the suite above ran on Python 3.12 rather than Humble's 3.10, and no ROS
interface was imported. In particular the two breaks the parity plan predicts —
`ParallelGripperCommand` and the trajectory controller's state topic name — are
still predictions.

## Measured on a Humble host

Date: 2026-07-27. Host `5070ti-control`: Ubuntu 22.04.5, ROS 2 Humble,
Python 3.10.12, RTX 5070 Ti. This is the section the one above was waiting for.

### Both predicted breaks are not breaks here

The parity plan's §2 and §3 were the two things that could not be settled from a
Jazzy machine. Both were measured rather than reasoned about, and both are fine:

| Prediction | Measured |
|---|---|
| `control_msgs.action.ParallelGripperCommand` absent on Humble | present |
| `joint_trajectory_controller` publishes `~/state`, not `~/controller_state` | publishes **both** |

`ros-humble-joint-trajectory-controller` is at `2.53.1-1jammy.20260611`, well
past the rename, and its shared object carries both `~/state` and
`~/controller_state`. `CONTROLLER_STATE_TOPIC = "controller_state"` is therefore
correct on this host as written, and no `REQUIRED_INTERFACES` entry needed
editing. All nine interfaces import.

This is a fact about *this installation's* version, not about Humble in general.
An older `ros2_controllers` predates the rename and would still need the other
name.

### The neutral core on Python 3.10

`339 passed, 2 skipped` — the same counts as Jazzy's Python 3.12, and the two
skips are the same MoveIt tip-link skips, for the same reason. `robotctl r2s
preflight` reports the profile, the asset and its 35 joints, and
`profile.endpoint()` resolves Humble's endpoint at 100 Hz from `.rosdistro`.

One prerequisite that is not a code change: the host had no `python3.10-venv`
and no passwordless sudo, so the venv was created with `--without-pip` and
bootstrapped with `get-pip.py`.

### `colcon build`: 18 of 18, after three fixes

The first build on a Humble host found three things a Jazzy host structurally
could not, because `build.sh` refuses to run there at all.

1. **`build.sh` passed `--log-base` after the verb.** It is a global colcon
   option; `colcon-core` 0.21 rejects it there and the build aborts before any
   package is configured. Fixed in `ros_ws/build.sh`.
2. **`openarm_can` links CLI11 without declaring it.** `CMakeLists.txt:129`
   calls `find_package(CLI11 REQUIRED)` for `openarm-can-cli`, and
   `package.xml` declares only `ament_cmake`, so rosdep reports nothing missing
   and the build fails on the first package. This is an upstream defect;
   `install_dependencies_humble.sh` installs CLI11 v2.4.2 to `~/opt/cli11`
   rather than patching the vendor tree.
3. **`dg_sdk_ros2_bridge` links `libs/libDGSDK.so`, which upstream does not
   ship.** The tree vendors `libDGSDK_{140,160,171}.so` and its README says to
   rename one. `install_dependencies_humble.sh` copies `171`; the file is
   gitignored so the snapshot stays byte-identical to upstream.

Reproduce with:

```bash
source /opt/ros/humble/setup.bash
./ros_ws/install_dependencies_humble.sh
export CMAKE_PREFIX_PATH=$HOME/opt/cli11:$CMAKE_PREFIX_PATH
./ros_ws/build.sh
```

`rosdep` leaves three keys unresolved — `ign_ros2_control`, `ros_gz` and
`realsense2_description`. The first two belong to the three DG5F/DG4F/DG3F
Gazebo packages and the third to `openarm_description`'s xacro. None of them
blocks the build: all 18 packages finish without them. They matter only for
Gazebo simulation and for xacro-expanding the RealSense mount, which is
Task 5's territory.

### The TCP finding holds here

Measured on this host's snapshot rather than inferred: `openarm_bimanual.srdf`
(v2.0) declares

```xml
<end_effector name="left_ee"  parent_link="openarm_left_ee_base_link"  group="left_gripper"/>
<end_effector name="right_ee" parent_link="openarm_right_ee_base_link" group="right_gripper"/>
```

and the string `openarm_right_hand_tcp` appears nowhere in the file. The two
skipped tests are correct to skip, and Task 5 still owns the resolution.

### Not verified: anything needing hardware

`r2s collect` reaches the adapter, imports rclpy, and times out waiting for
`/joint_states` — which is the correct behaviour with no bringup running. No
arm, hand or CAN interface was connected, so `pose show`, `pose ee
--from-marker` and `pose gravity --sweep` remain unmeasured. Task 4's
`pose_bringup.sh` and the smoke harness do not exist on this branch yet.

`tools/verify_vendor_snapshot.py` was not re-run here; it compares against
upstream clones that this host does not have.
