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

The final static repair added declared OpenArm and Tesollo manifest patches
and exact `post_patch_sha256` inventories. The verifier reconstructs each
patched tree from a clean archive, checks that the inventory paths exactly
match the patch-induced changes, validates every digest, and then retains the
whole-tree comparison.

The four pinned verification commands are:

```bash
set -euo pipefail
VERIFY_TMP="$(mktemp -d /tmp/robot-control-jazzy-vendor.XXXXXX)"
mkdir -p "$VERIFY_TMP"/{openarm,tesollo,openarm_can,description}

git -C /home/user/rl_ws/repo/openarm/openarm_ros2 \
  archive 8087bbc2b37c0b2b2652c0134a9b2b369c57567e |
  tar -xf - -C "$VERIFY_TMP/openarm"
git -C /home/user/rl_ws/repo/tesollo/delto_m_ros2 \
  archive 3926c2eab8d011046f64874d6252213b2cf18f48 |
  tar -xf - -C "$VERIFY_TMP/tesollo"
git -C /home/user/rl_ws/repo/openarm/openarm_can \
  archive c32ecd31da267967f0c913c2118c843177d88b91 |
  tar -xf - -C "$VERIFY_TMP/openarm_can"
git -C /home/user/rl_ws/teleopration_openarm_tesollo \
  archive c8696ebfd64ea08ee0a212a9bae21055b6f381bc \
  src/openarm_description |
  tar -xf - -C "$VERIFY_TMP/description"

PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py \
  --metadata vendor_metadata/openarm/UPSTREAM.yaml \
  --source "$VERIFY_TMP/openarm" \
  --snapshot ros_ws/src/openarm_ros2
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py \
  --metadata vendor_metadata/tesollo/UPSTREAM.yaml \
  --source "$VERIFY_TMP/tesollo" \
  --snapshot ros_ws/src/delto_m_ros2
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py \
  --metadata vendor_metadata/openarm_can/UPSTREAM.yaml \
  --source "$VERIFY_TMP/openarm_can" \
  --snapshot ros_ws/src/openarm_can
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py \
  --metadata vendor_metadata/openarm_description/UPSTREAM.yaml \
  --source "$VERIFY_TMP/description/src/openarm_description" \
  --snapshot ros_ws/src/openarm_description
```

The final static run produced four `snapshot verified` results.

## Verification run

The commands below were run on 2026-07-25 (Asia/Seoul) on Ubuntu 24.04.4 LTS
with `/opt/ros/jazzy/setup.bash` sourced. A blocked prerequisite is not counted
as a pass, and the runtime checks were not attempted after the dependency and
build failures. The final review fixed the package-graph defects statically;
it did not rerun or reclassify the blocked runtime work.

| Check | Status | Exit status | Evidence |
| --- | --- | ---: | --- |
| Snapshot provenance and inventories | **PASS** | 0 | Four fresh pinned archives each produced `snapshot verified`; OpenArm has one inventoried changed file and Tesollo has seven. |
| Declared local package closure | **PASS** | 0 | The four supported roots resolve to 11 local packages. `openarm_can` precedes `openarm_hardware`; `dg_description` precedes `dg5f_gz`; `dg3f_m_gz` and `dg4f_gz` remain outside the closure. |
| Dependency helper | **BLOCKED** | 1 | `sudo` rejected elevation because `/etc/sudo.conf` is owned by UID 65534 and the container has `no new privileges`; no password was requested or bypassed. |
| Recorded supported rosdep graph | **FAIL** | 1 | The pre-repair run found 16 absent Jazzy apt dependencies. No unresolved `ign_ros2_control` key appeared. It was not promoted after static manifest repair. |
| Recorded supported build | **FAIL** | 1 | The pre-repair graph incorrectly expanded to nine packages: 0 finished, 1 failed, 4 aborted, and 4 were not processed. The graph now resolves statically to 11, but no build rerun was possible without the blocked dependencies. |
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

Recorded result before the final manifest repair: **FAIL**, exit status 1.
Rosdep reported these 16 missing apt dependencies:

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

Recorded result before the final manifest repair: **FAIL**, exit status 1.
The four roots in `ros_ws/supported-packages.txt` expanded to an incomplete
nine-package build graph. Colcon reported:

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

The final static manifest repair makes the declared closure:

```text
openarm_bringup
openarm_description
openarm_can
openarm_hardware
openarm
openarm_bimanual_moveit_config
delto_tcp_comm
delto_hardware
dg5f_driver
dg_description
dg5f_gz
```

`openarm_can` and `dg_description` are now selected and ordered by their
consumers' `package.xml` dependencies rather than by appending unordered build
roots. `dg5f_gz` also declares its CMake-required `trajectory_msgs`
dependency. This repair is covered by:

```bash
PYTHONPATH=src:. pytest -q \
  tests/test_jazzy_build_wrapper.py \
  tests/test_dg5f_jazzy_contract.py
```

No `colcon build` rerun was attempted because dependency installation remains
blocked. The runtime build status therefore remains **FAIL**, not PASS.

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
