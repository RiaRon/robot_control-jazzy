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

## Pose-setting verification

Completed on 2026-07-26 (Asia/Seoul), after the pose-setting work landed. Every
check was run from a clean process table.

| Check | Status | Exit status | Evidence |
| --- | --- | ---: | --- |
| Static suite | **PASS** | 0 | `156 passed`, up from 84 before this work. `python3 -m compileall -q src tests tools` and `bash -n` on both new scripts are clean. |
| OpenArm fake smoke | **PASS** | 0 | `smoke test passed`. |
| DG5F fake smoke | **PASS** | 0 | `smoke test passed`. |
| DG5F Gazebo smoke | **PASS** | 0 | `smoke test passed`. |
| OpenArm pose smoke | **PASS** | 0 | New. `pose smoke: after z=+0.1919 moved +0.0300` against a commanded +0.0300 m, tolerance 0.005 m. |
| Snapshot provenance | **PASS** | 0 | Four `snapshot verified` results. This work changed no vendored tree, so no patch or inventory needed updating. |

```bash
PYTHONPATH=src:. pytest -q
python3 -m compileall -q src tests tools
bash -n ros_ws/pose_bringup.sh ros_ws/smoke_pose_openarm.sh
./ros_ws/smoke_openarm_fake.sh
./ros_ws/smoke_dg5f_fake.sh
./ros_ws/smoke_dg5f_gazebo.sh
./ros_ws/smoke_pose_openarm.sh
```

`smoke_pose_openarm.sh` launches through `ros_ws/pose_bringup.sh`, the same
wrapper an operator uses, so it would catch that wrapper regressing to the
vendor's real-hardware default. Its validator measures the end effector, runs
the real `robotctl pose ee --relative --xyz 0,0,0.03 --execute`, and measures
again; a library-only check would pass even with the CLI wiring broken.

Kill leftovers between runs with patterns that bracket their first character:

```bash
for pattern in "[m]ove_group" "[r]os2_control_node" "[r]obot_state_publisher" \
               "[b]in/ros2 launch" "[r]viz2" "[g]z sim" "[p]arameter_bridge"; do
    pkill -f "$pattern" || true
done
```

Without the bracket the pattern matches the `pkill` command line itself and
kills the shell running it, which happened once during this run and left a
half-torn-down controller manager that the next check then read from.

### Environment noise seen during the pose runs

Neither affects kinematics or control, and neither is caused by this branch:

- `move_group` logs `Unable to transform object from frame 'camera_*' to
  planning frame 'world'`. A RealSense node running on this host publishes
  collision objects in camera frames that are not connected to the robot's TF
  tree.
- `rviz2` logs `unrealistic inertia` for the four gripper finger links, and
  `occupancy_map_monitor` fails to load `DepthImageOctomapUpdater` because
  `moveit_ros_perception` is not installed. Depth-sensor octomap collision is
  therefore unavailable, as already recorded under Deferred in the design.

## Gravity compensation, measured on the real right arm

First real measurement of `robotctl pose gravity --sweep`, on the physical
OpenArm right arm over `can0`, at a loaded pose. Errors are the trajectory
controller's own `error.positions`, in radians.

| scale | r_aj_1 | r_aj_2 | r_aj_3 | r_aj_4 | r_aj_5 | r_aj_6 | r_aj_7 | worst | mean |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.00 | +0.0428 | +0.2977 | +0.1509 | +0.1242 | −0.0029 | +0.0121 | −0.0433 | 0.2977 | 0.0963 |
| 0.25 | +0.0428 | +0.2424 | +0.1204 | +0.1120 | −0.0029 | +0.0121 | −0.0425 | 0.2424 | 0.0822 |
| 0.50 | +0.0428 | +0.1783 | +0.0922 | +0.0784 | −0.0029 | +0.0121 | −0.0391 | 0.1783 | 0.0637 |
| 0.75 | +0.0378 | +0.1142 | +0.0678 | +0.0513 | −0.0029 | +0.0125 | −0.0391 | 0.1142 | 0.0465 |
| 1.00 | +0.0344 | +0.0429 | +0.0331 | +0.0334 | −0.0014 | +0.0125 | −0.0395 | 0.0429 | 0.0282 |

**PASS**: compensation reduces the worst joint error sevenfold, 0.2977 to 0.0429
rad, and the mean by 3.4 times. The modelled torque at this pose was +1.09,
+5.27, +2.38, +1.55, −0.06, −0.06, −0.01 N·m over a 5.035 kg chain.

Two findings the numbers make plain.

**The model under-predicts the load.** Error falls monotonically all the way to
scale 1.0 with no sign of over-compensation, so the optimum lies above 1.0.
Extrapolating each joint's slope to zero puts it at about 1.17 for `r_aj_2`, 1.28
for `r_aj_3`, and 1.37 for `r_aj_4` — all inside the 1.5 ceiling. That those
three differ also means one global scale can only reach a compromise; the
distribution of modelled torque is slightly off, not just its magnitude.

**Three joints are limited by something other than gravity.** `r_aj_1`, `r_aj_6`
and `r_aj_7` barely move across the whole sweep, and `r_aj_6` ends fractionally
worse. Their modelled torques are −0.06 to +1.09 N·m, near nothing, yet they hold
0.03 to 0.04 rad of error at every scale. A scale-independent residual against a
near-zero modelled load is the signature of Coulomb friction: roughly constant in
magnitude, opposing motion, and therefore untouched by any gravity scale. On
`r_aj_7`, whose `kp` is 5, 0.2 N·m of stiction alone accounts for 0.04 rad.

Gravity compensation cannot remove that component. A friction feedforward would
need the sign of motion, which a stationary joint does not have, so standing
accuracy stays the business of `pose ee --settle`.

## Real2Sim parameter identification

Implemented against the plan in
`docs/superpowers/plans/2026-07-26-real2sim-parameter-identification.md`. **Not
yet run on hardware** — what follows is what is verified in simulation, what is
still owed from the real arm, and one defect the work exposed.

### Verified against a synthetic robot

Each check builds a robot with known parameters, simulates it, and asks whether
the pipeline recovers them.

| Check | Result |
| --- | --- |
| Static fit recovers `kp`, `alpha` and the offset from exact sweeps | exact to 1e-6 relative |
| Static fit under 2e-4 rad of noise | within 5% |
| Dynamic fit with the gravity column recovers `k`, `d`, `f` and `1/J` | within 1–2% |
| `combine` recovers `J`, `b`, `tau_f` end to end | within 2% |
| The two independent inertias agree | under 2% apart |
| `r2s identify --collect` drives a stub arm and recovers its `kp` and `alpha` | within 2% |

The last one is the one worth naming: it designs a pose set, moves a stub arm
through it, sweeps at each pose, writes a file per pose, and refits from disk to
the same numbers. Every stage of the static half runs in that one test.

The simulation uses semi-implicit Euler so that the finite differences
`fit_second_order` takes are exactly the accelerations the simulation used. A
correct fit is then exact, and any bias shows up as bias rather than hiding inside
discretisation error.

### Defect exposed: the dynamic fit had no gravity term

`fit_second_order` fitted `qdd = k(q_cmd−q) − d·qd − f·sign(qd)`. The real
equation carries `−tau_g(q)/J`, and with that term missing the regression has
nowhere to put a standing load but the stiffness and damping columns. It had only
ever run on synthetic tracks with no load in them, so this had never shown.

It is not a small error. On a simulated seven-joint arm under its own weight, the
fit without the gravity term **refuses outright** — the load drives a fitted
parameter out of the range the fit accepts, so `r_aj_2` comes back
`unidentifiable dynamics`. A lighter load would be worse: it would return a
plausible wrong number. A test pins both the refusal and the recovery once the
term is supplied.

### Still owed from the real arm

- **A dynamic track.** `r2s collect --execute` still prints `ROS publisher
  backend is required` and publishes nothing, so no real track exists. The
  static half of the pipeline (`pose gravity --output`, `r2s identify`,
  `r2s identify --collect`) reaches hardware; `r2s fit --static` cannot be run on
  real data until collect does.
- **Measured parameters.** No `J`, `b`, `tau_f` or `kp` has been measured on this
  arm yet. The `kp` values of 7.5, 15.4 and 28.4 quoted in the plan are what
  fitting the *existing single-pose* sweep gives, and the point of quoting them is
  that they scatter in both directions around the vendor header's 20 — the
  signature of an under-determined fit, which is why several poses are needed.
  They are not a result.
- **Whether the designed pose set is collision-free on the real arm.** Nothing in
  the tool checks the arm against itself or its surroundings; the profile bounds
  each joint separately. The pose set is deterministic in `--seed` and the dry run
  prints the itinerary so the review is real, but the review is a person looking
  at RViz.

### Real hardware run checklist

When the arm is next available:

1. `robotctl r2s identify --collect --group openarm_right_arm --poses 4 --output static.json`
   and read every pose in RViz before going further.
2. The same command with `--seed 0 --sweep-dir sweeps --execute`.
3. Record the `kp`, `alpha`, `offset`, residual, conditioning and frozen counts
   here, and whether `r_aj_4` is still frozen once the poses vary its load.
4. Note whether `r_aj_1`, `r_aj_6` and `r_aj_7` — the three the gravity sweep
   showed to be friction-limited rather than gravity-limited — are identifiable
   at all, or come back named.

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
