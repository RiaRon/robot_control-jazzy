# Jazzy Snapshot and Readiness Verification

Snapshot and runtime checks were recorded on 2026-07-25 using Ubuntu 24.04
and ROS 2 Jazzy. The runtime status is reported separately below.

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

## Verification run

The commands below were run on 2026-07-25 (Asia/Seoul) on Ubuntu 24.04.4 LTS
with `/opt/ros/jazzy/setup.bash` sourced. A blocked prerequisite is not counted
as a pass, and the runtime checks were not attempted after the dependency and
build failures.

| Check | Status | Exit status | Evidence |
| --- | --- | ---: | --- |
| Dependency helper | **BLOCKED** | 1 | `sudo` rejected elevation because `/etc/sudo.conf` is owned by UID 65534 and the container has `no new privileges`; no password was requested or bypassed. |
| Supported rosdep graph | **FAIL** | 1 | 16 required Jazzy apt dependencies are absent. No unresolved `ign_ros2_control` key appeared. |
| Supported build | **FAIL** | 1 | Four supported roots expanded to nine packages: 0 finished, 1 failed, 4 aborted, and 4 were not processed. `openarm_hardware` could not find `hardware_interfaceConfig.cmake`. |
| OpenArm fake smoke | **BLOCKED** | not run | Prerequisites were not green. Expected controllers: `joint_trajectory_controller=active` and `joint_state_broadcaster=active`; expected command/state joint counts: 7/8; observed states/counts: none. |
| DG5F fake smoke | **BLOCKED** | not run | Prerequisites were not green. Expected controllers: `joint_trajectory_controller=active` and `joint_state_broadcaster=active`; expected command/state joint counts: 20/20; observed states/counts: none. |
| DG5F Gazebo smoke | **BLOCKED** | not run | Prerequisites were not green. Expected controllers: `joint_trajectory_controller=active` and `joint_state_broadcaster=active`; expected command/state joint counts: 20/20; observed states/counts: none. |

### Dependency installation

Command:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh
```

Result: **BLOCKED**, exit status 1.

```text
sudo: /etc/sudo.conf is owned by uid 65534, should be 0
sudo: The "no new privileges" flag is set, which prevents sudo from running as root.
sudo: If sudo is running in a container, you may need to adjust the container configuration to disable the flag.
```

The helper was invoked once without attempting to obtain, store, or bypass an
operator password.

### Supported dependency graph

Command:

```bash
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths \
  ros_ws/src/openarm_ros2 \
  ros_ws/src/openarm_can \
  ros_ws/src/openarm_description \
  ros_ws/src/delto_m_ros2/delto_hardware \
  ros_ws/src/delto_m_ros2/delto_tcp_comm \
  ros_ws/src/delto_m_ros2/dg_description \
  ros_ws/src/delto_m_ros2/dg_msgs \
  ros_ws/src/delto_m_ros2/dg5f_driver \
  ros_ws/src/delto_m_ros2/dg5f_gz \
  --ignore-src
```

Result: **FAIL**, exit status 1. Rosdep reported these 16 missing apt
dependencies:

```text
ros-jazzy-gz-ros2-control
ros-jazzy-hardware-interface
ros-jazzy-ros-gz
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-joint-state-publisher-gui
ros-jazzy-moveit-ros-move-group
ros-jazzy-moveit-kinematics
ros-jazzy-moveit-planners
ros-jazzy-moveit-simple-controller-manager
ros-jazzy-joint-state-publisher
ros-jazzy-controller-manager
ros-jazzy-moveit-configs-utils
ros-jazzy-moveit-ros-visualization
ros-jazzy-moveit-setup-assistant
ros-jazzy-ros2-control-test-assets
```

The result did not contain an unresolved `ign_ros2_control` dependency.

### Supported build

Command:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
```

Result: **FAIL**, exit status 1. The four roots in
`ros_ws/supported-packages.txt` expanded to a nine-package build graph.
Colcon reported:

```text
Summary: 0 packages finished
  1 package failed: openarm_hardware
  4 packages aborted: delto_tcp_comm dg5f_gz openarm_bringup openarm_description
  4 packages not processed
```

The first reproducible failure was:

```text
Could not find a package configuration file provided by "hardware_interface"
with any of the following names:
  hardware_interfaceConfig.cmake
  hardware_interface-config.cmake
```

This is consistent with the missing `ros-jazzy-hardware-interface` dependency
reported by rosdep.

### Runtime smoke checks

The following commands were **not run**:

```bash
./ros_ws/smoke_openarm_fake.sh
./ros_ws/smoke_dg5f_fake.sh
./ros_ws/smoke_dg5f_gazebo.sh
```

All three are **BLOCKED** because dependency installation, rosdep validation,
and the supported build were not green. Therefore there are no observed
controller states, joint counts, or state round-trip measurements. No CAN
interface or Tesollo TCP device was opened.

To complete verification, an operator must run the dependency helper on a
Jazzy host where sudo is permitted, then repeat the rosdep check and supported
build. Run the three smoke scripts only after both checks pass.
