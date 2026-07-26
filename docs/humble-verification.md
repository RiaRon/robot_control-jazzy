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
