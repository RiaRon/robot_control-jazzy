# Jazzy Snapshot and Readiness Verification

Snapshots were first recorded on 2026-07-25. The runtime checks were blocked
that day because dependency installation required an elevation the container
refused. They were completed on 2026-07-26 on Ubuntu 24.04.4 LTS with ROS 2
Jazzy, and every check now passes.

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

Completed on 2026-07-26 (Asia/Seoul) on Ubuntu 24.04.4 LTS with
`/opt/ros/jazzy/setup.bash` sourced. Every check below was run to completion;
none is recorded as blocked.

| Check | Status | Exit status | Evidence |
| --- | --- | ---: | --- |
| Snapshot provenance and inventories | **PASS** | 0 | Four fresh pinned archives each produced `snapshot verified` after the patch and inventory update described below. |
| Declared local package closure | **PASS** | 0 | The four supported roots resolve to 11 local packages. `openarm_can` precedes `openarm_hardware`; `dg_description` precedes `dg5f_gz`; `dg3f_m_gz` and `dg4f_gz` remain outside the closure. |
| Dependency helper | **PASS** | 0 | 269 packages newly installed. `rosdep install` finished with `#All required rosdeps installed successfully`. |
| Supported rosdep graph | **PASS** | 0 | `All system dependencies have been satisfied`. The 16 previously absent apt dependencies are resolved and no `ign_ros2_control` key appears. |
| Supported build | **PASS** | 0 | Clean `colcon build`: `11 packages finished [18.4s]`. Three packages emit stderr warnings only (`on_init` deprecation, unused parameter); no errors. |
| OpenArm fake smoke | **PASS** | 0 | `joint_trajectory_controller`, `joint_state_broadcaster`, and `gripper_controller` activated; 7 command joints and 8 state joints round-tripped within 0.02 rad. |
| DG5F fake smoke | **PASS** | 0 | `joint_trajectory_controller` and `joint_state_broadcaster` activated; 20/20 command and state joints round-tripped within 0.02 rad. |
| DG5F Gazebo smoke | **PASS** | 0 | Headless `gz sim` stepped at real-time factor ~1.0; 20/20 joints reached target within 0.02 rad; zero `No clock received` warnings. |

### Dependency installation

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh
```

Result: **PASS**. The 2026-07-25 blocker was an environment restriction, not a
branch defect: `sudo` refused elevation because `/etc/sudo.conf` was owned by
UID 65534 under a `no new privileges` container. On 2026-07-26 the helper was
run from an interactive terminal on the same host and completed normally. No
password was requested, stored, or bypassed by any automated step.

### Supported dependency graph

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

Result: **PASS**, exit status 0, `All system dependencies have been satisfied`.

### Supported build

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
```

Result: **PASS**, exit status 0, from a cleaned `ros_ws/{build,install,log}`.

```text
Summary: 11 packages finished [18.4s]
  3 packages had stderr output: delto_hardware openarm_can openarm_hardware
```

The stderr output is limited to compiler warnings: the deprecated
`hardware_interface::SystemInterface::on_init(const HardwareInfo &)` overload
in `openarm_hardware` and `delto_hardware`, and an unused parameter in an
`openarm_can` example. Nothing failed to compile.

### Runtime smoke checks

```bash
./ros_ws/smoke_openarm_fake.sh
./ros_ws/smoke_dg5f_fake.sh
./ros_ws/smoke_dg5f_gazebo.sh
```

All three reported `smoke test passed` with exit status 0. No CAN interface
and no Tesollo TCP device was opened; the OpenArm run used
`can_interface:=robot_control_fake_only`, and every run used `gui:=false`. No
real-hardware command was executed.

Run the three scripts from a clean process table. A leftover launch from a
previous run keeps publishing `/joint_states`, and the validator then reports
the other robot's joints as `extra joints`.

## Defects found and fixed during this run

Three defects were exposed only once the build blocker was cleared. Each was
invisible beforehand because the checks that would have caught them were
themselves blocked.

### Smoke harness aborted while sourcing the workspace

All three smoke scripts exited 1 with no diagnostic. The entrypoints enable
`set -euo pipefail` before `run_smoke_harness` sources
`ros_ws/install/setup.bash`, but colcon and ament setup files read
`COLCON_TRACE` and `AMENT_TRACE_SETUP_FILES` with no default. Under `set -u`
that aborts, and `set -e` then killed the script silently.

`ros_ws/smoke_harness.sh` now clears `set -eu` for the source and restores
strict mode immediately afterwards, matching the original intent. The existing
regression test wrote an **empty** fake `setup.bash`, so it could not observe
this; `tests/test_ros_smoke.py` now uses a setup file that dereferences both
variables the way real generated files do.

### DG5F Gazebo simulation had no clock source

`gz sim` stepped correctly and the model spawned, but ROS `/clock` had zero
publishers, so `/joint_states` stamps stayed at zero and
`joint_trajectory_controller` never advanced a trajectory. The controller
manager logged `No clock received, using time argument instead` once per
second.

The cause was in this branch's own migration patch, not upstream: rewriting
the three `dg5f_gz` launch files from `ign_ros2_control` to `gz_ros2_control`
dropped the `/clock` bridge. `dg5f_right_gz.launch.py`,
`dg5f_left_gz.launch.py`, and `dg5f_both_gz.launch.py` now start a
`ros_gz_bridge` `parameter_bridge` for
`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`, conditioned on
`UnlessCondition(use_fake_hardware)`. No new dependency was needed —
`dg5f_gz/package.xml` already declares `ros_gz`.

### Smoke plan commanded a target outside a joint limit

With simulation time restored, 19 of 20 DG5F joints tracked the target and
`rj_dg_1_2` stayed at 0. The DG5F description limits that joint to
`[-pi, 0.0]`, so the shared `+0.05` target is unreachable; Gazebo physics
clamps it. Fake hardware echoes any command back, which is why the fake runs
passed and hid the defect.

`smoke_plan("dg5f")` now commands `-0.05` for `rj_dg_1_2` and `+0.05`
elsewhere, keeping the displacement magnitude identical for all 20 joints. A
test parses the joint limits out of the DG5F description and asserts every
commanded target lies inside its range.

### Provenance impact

The launch-file change edits a vendored tree, so the declared patch and its
inventory were regenerated:
`vendor_metadata/tesollo/patches/0001-dg5f-gz-ros2-control-jazzy.patch` was
rebuilt against the pinned archive, and the `post_patch_sha256` entries for
the three launch files were updated. The other four inventory entries and all
three other components are unchanged. Re-running the four pinned verification
commands produced four `snapshot verified` results.
