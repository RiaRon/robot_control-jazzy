# Jazzy Snapshot Verification

Verified on 2026-07-25 using Ubuntu 24.04 and ROS 2 Jazzy.

## Pinned snapshots

- OpenArm ROS 2: `enactic/openarm_ros2@8087bbc2b37c0b2b2652c0134a9b2b369c57567e`
  (`jazzy`)
- Tesollo ROS 2: `tesollodelto/delto_m_ros2@3926c2eab8d011046f64874d6252213b2cf18f48`
  (`jazzy-dev`)
- OpenArm CAN: `enactic/openarm_can@c32ecd31da267967f0c913c2118c843177d88b91`
  (`main`, shared)
- OpenArm description:
  `divingyoon/teleopration_openarm_tesollo@c8696ebfd64ea08ee0a212a9bae21055b6f381bc`
  subpath `src/openarm_description` (shared)

All four imported trees pass `tools/verify_vendor_snapshot.py` against their
pinned source trees. `colcon list --base-paths ros_ws/src` discovers 18
branch-local packages.

## Build status

The Jazzy build wrapper was invoked under `/opt/ros/jazzy`. It reaches the
package build and then stops at `openarm_can` because `libcli11-dev` is not
installed on this host. `rosdep check` additionally reports missing Jazzy
MoveIt, ros2_control, controller, and ros_gz binary packages.

The Tesollo `jazzy-dev` snapshot still declares `ign_ros2_control` in
`dg3f_m_gz`, `dg4f_gz`, and `dg5f_gz`. Ubuntu Noble/Jazzy has no rosdep
definition for that legacy key. The snapshot is intentionally unmodified, so
these simulation packages require an upstream Jazzy migration before they can
be considered validated. Hardware driver packages can be tested while
explicitly skipping that key.

Install the resolvable dependencies and repeat the build on a host with sudo
access:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y libcli11-dev
sudo rosdep install \
  --from-paths ros_ws/src \
  --ignore-src \
  -r -y \
  --skip-keys ign_ros2_control
./ros_ws/build.sh
```

Do not treat a partial build that skips the three Tesollo Gazebo packages as
full snapshot validation. Their `ign_ros2_control` package and URDF plugin
references must be migrated to the Jazzy `gz_ros2_control` interface upstream
or carried as an explicitly documented project patch.
