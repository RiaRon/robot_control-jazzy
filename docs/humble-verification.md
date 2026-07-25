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
