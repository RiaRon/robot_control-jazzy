# Pose Setting and ROS Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the project-owned ROS adapter and a `robotctl pose` surface that sets an OpenArm bimanual configuration by joint values, SRDF named state, or end-effector IK, and document every resulting command in `docs/cli.md`.

**Architecture:** Operator pose setting and learned-policy commands converge on the existing `CanonicalInterface` and `CommandGate` before anything is published. IK comes from the already-configured `openarm_bimanual_moveit_config`; execution goes to the per-arm `FollowJointTrajectory` controllers. `rclpy` stays an optional import so the core CLI keeps failing cleanly rather than crashing when no adapter is installed.

**Tech Stack:** ROS 2 Jazzy, MoveIt 2, rclpy, ros2_control, Python 3.12, pytest, Bash, YAML

## Global Constraints

- Work only on the long-lived `jazzy` branch.
- Target the OpenArm bimanual configuration and the `openarm_tesollo` profile.
- Fake hardware is the default everywhere; reaching hardware requires an explicit flag.
- Do not open CAN, publish to physical controllers, or connect to a Tesollo device.
- Profile limits are authoritative over URDF limits when the two disagree.
- `import robot_control` must not require `rclpy`.
- Every vendor tree change needs a declared patch and a `post_patch_sha256` update.
- Plan-then-execute only; do not introduce `moveit_servo`.

---

### Task 1: Executable Group Contract in the Profile

**Files:**
- Modify: `src/robot_control/profile.py`
- Modify: `src/robot_control/profiles/openarm_tesollo.yaml`
- Modify: `tests/test_profile.py`

**Interfaces:**
- Produces: `Group.controller: str | None` and `Group.moveit_group: str | None`
- Produces: `RobotProfile.executable_groups() -> dict[str, Group]`

- [ ] **Step 1: Write failing group-contract tests**

Assert that `openarm_right_arm` resolves to controller
`right_joint_trajectory_controller` and MoveIt group `right_arm`, that
`openarm_left_arm` resolves to the left pair, that a Tesollo group has a
controller but no `moveit_group`, and that a group with neither is absent from
`executable_groups()`. Assert that every declared controller joint set is a
subset of the group's canonical joints.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_profile.py -k group_contract`

Expected: FAIL because `Group` has no `controller` or `moveit_group` field.

- [ ] **Step 3: Add the optional fields and accessor**

Extend `Group` with the two optional fields, parse them in `load_profile`, and
add `executable_groups()` returning only groups that declare a controller.
Reject a profile that names a `moveit_group` without a `controller`.

- [ ] **Step 4: Declare the mapping for OpenArm and Tesollo**

In `openarm_tesollo.yaml`, give `openarm_right_arm` controller
`right_joint_trajectory_controller` and MoveIt group `right_arm`,
`openarm_left_arm` the left pair, `openarm_left_gripper` controller
`left_gripper_controller`, and each Tesollo group controller
`joint_trajectory_controller` with no `moveit_group`.

- [ ] **Step 5: Verify GREEN**

Run: `PYTHONPATH=src:. pytest -q tests/test_profile.py tests/test_interface.py`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/robot_control/profile.py src/robot_control/profiles/openarm_tesollo.yaml tests/test_profile.py
git commit -m "feat: declare executable group contract in the profile"
```

### Task 2: Trajectory Authorization in the Safety Gate

**Files:**
- Modify: `src/robot_control/safety.py`
- Modify: `tests/test_safety.py`

**Interfaces:**
- Produces: `CommandGate.authorize_trajectory(points: Sequence[np.ndarray], start_time_sec: float, period_sec: float) -> list[np.ndarray]`

- [ ] **Step 1: Write failing trajectory-authorization tests**

Assert that a trajectory whose every waypoint is in bounds returns all
waypoints; that a trajectory with one out-of-bounds waypoint raises
`SafetyError` and leaves gate state untouched, so a following single-point
`authorize` still succeeds against the pre-trajectory pose; that a trajectory
violating the velocity limit between two adjacent waypoints is rejected; and
that an empty trajectory is rejected.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_safety.py -k trajectory`

Expected: FAIL because `authorize_trajectory` does not exist.

- [ ] **Step 3: Implement all-or-nothing authorization**

Validate every waypoint against position and velocity limits on a copy of the
gate state, and commit `_last` and `_last_time` only after the whole trajectory
passes. Preserve the existing `execute` and E-stop refusals. Reject an empty
trajectory.

- [ ] **Step 4: Verify GREEN**

Run: `PYTHONPATH=src:. pytest -q tests/test_safety.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/safety.py tests/test_safety.py
git commit -m "feat: authorize whole trajectories all-or-nothing"
```

### Task 3: ROS Adapter

**Files:**
- Create: `src/robot_control/ros_adapter.py`
- Create: `tests/test_ros_adapter.py`

**Interfaces:**
- Produces: `RosAdapter(profile, group_name, execute: bool)`
- Produces: `RosAdapter.read_state() -> np.ndarray`
- Produces: `RosAdapter.solve_ik(pose: Pose, seed: np.ndarray) -> np.ndarray`
- Produces: `RosAdapter.send_trajectory(points, period_sec) -> None`
- Produces: `AdapterUnavailable(RuntimeError)`

- [ ] **Step 1: Write failing adapter tests with an injected ROS backend**

Design `RosAdapter` to take an injected backend object so the tests need no
running ROS. Assert that constructing with `execute=False` raises
`SafetyError`; that a missing `rclpy` raises `AdapterUnavailable` with a message
naming the adapter; that `read_state` converts source joint names to canonical
order and sign; that `solve_ik` converts an IK solution keyed by source names
into canonical order; that `solve_ik` raises when the service reports a failure
code; and that `send_trajectory` targets the controller declared by the group.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_ros_adapter.py`

Expected: FAIL because `robot_control.ros_adapter` does not exist.

- [ ] **Step 3: Implement the adapter**

Import `rclpy` and the message packages inside the constructor, raising
`AdapterUnavailable` on `ImportError`. Resolve the controller and MoveIt group
from the profile. Convert through `CanonicalInterface` in both directions. Use
`call_async` with `spin_until_future_complete` for `/compute_ik` and
`/compute_fk`; a synchronous `call` from an unspun node deadlocks. Send
trajectories through the `FollowJointTrajectory` action.

- [ ] **Step 4: Confirm the core package still imports without rclpy**

Run:

```bash
PYTHONPATH=src:. python3 -c "import robot_control, sys; assert 'rclpy' not in sys.modules; print('core import clean')"
PYTHONPATH=src:. pytest -q tests/test_ros_adapter.py
```

Expected: both PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/ros_adapter.py tests/test_ros_adapter.py
git commit -m "feat: add optional ROS adapter for canonical execution"
```

### Task 4: `robotctl pose` Command Surface

**Files:**
- Modify: `src/robot_control/cli.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Produces: `robotctl pose show|joints|ee|rviz`

- [ ] **Step 1: Write failing CLI tests**

Assert that `pose joints --group openarm_right_arm --values ...` without
`--execute` prints the resolved canonical target and exits 0 without importing
an adapter; that `--execute` without an adapter exits 2 with a message naming
the adapter; that an unknown group exits 2; that a `--values` count mismatch
exits 2; that `--named` rejects a state absent from the SRDF; that `pose ee`
requires `--xyz`; and that `--relative` is rejected when no current pose can be
read.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_cli.py -k pose`

Expected: FAIL because the `pose` subcommand does not exist.

- [ ] **Step 3: Implement the subcommand**

Add `pose` with the four sub-stages. Keep `--dry-run` the default and require
`--execute` to publish. Import `ros_adapter` lazily and only when the command
needs ROS, so dry-run paths stay importable without `rclpy`. Map exit codes to
the existing convention: `2` for a missing backend or an unusable request, `3`
for an IK failure or a gate rejection.

- [ ] **Step 4: Verify GREEN**

Run: `PYTHONPATH=src:. pytest -q tests/test_cli.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/robot_control/cli.py tests/test_cli.py
git commit -m "feat: add robotctl pose command surface"
```

### Task 5: Fake-Hardware Bringup Wrapper

**Files:**
- Create: `ros_ws/pose_bringup.sh`
- Create: `tests/test_pose_bringup.py`
- Modify: `README.md`

**Interfaces:**
- Produces: `ros_ws/pose_bringup.sh` launching the bimanual MoveIt stack with `use_fake_hardware:=true`

- [ ] **Step 1: Write failing bringup tests**

Using a fake `ros2` executable that records its arguments, assert that the
wrapper passes `use_fake_hardware:=true` by default; that `--real` is required
to pass `use_fake_hardware:=false` and additionally requires explicit
`--right-can` and `--left-can` values; that a non-Jazzy `ROS_DISTRO` exits 2
before invoking `ros2`; and that a missing workspace `install/setup.bash` exits
2.

- [ ] **Step 2: Run the tests and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_pose_bringup.py`

Expected: FAIL because `ros_ws/pose_bringup.sh` does not exist.

- [ ] **Step 3: Implement the wrapper**

Use Bash strict mode. Clear `set -eu` only across the workspace `source`, as
`ros_ws/smoke_harness.sh` already does, because colcon and ament setup files
read `COLCON_TRACE` and `AMENT_TRACE_SETUP_FILES` with no default. Invert the
vendor default so fake hardware is what an operator gets by omission.

- [ ] **Step 4: Document the operator flow**

Add a README section covering launch, dragging the RViz end-effector marker,
reading the pose back, and committing it, with a pointer to `docs/cli.md`.

- [ ] **Step 5: Verify**

```bash
PYTHONPATH=src:. pytest -q tests/test_pose_bringup.py
bash -n ros_ws/pose_bringup.sh
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws/pose_bringup.sh tests/test_pose_bringup.py README.md
git commit -m "build: add fake-by-default pose bringup wrapper"
```

### Task 6: CLI Usage Documentation

**Files:**
- Create: `docs/cli.md`
- Create: `tests/test_cli_documentation.py`

**Interfaces:**
- Produces: `docs/cli.md` covering every `robotctl` command
- Produces: a test that fails when the parser and the document diverge

- [ ] **Step 1: Write the failing documentation test**

Walk the `argparse` parser built by `robot_control.cli._parser()`, collect
every subcommand path and every option string, and assert each appears in
`docs/cli.md`. Assert the document contains an exit-code table and that no
example uses `--execute` without an accompanying safety note.

- [ ] **Step 2: Run the test and verify RED**

Run: `PYTHONPATH=src:. pytest -q tests/test_cli_documentation.py`

Expected: FAIL because `docs/cli.md` does not exist.

- [ ] **Step 3: Write the document**

Open with the interactive operator flow end to end. Then document `pose show`,
`pose joints`, `pose ee`, `pose rviz`, and each existing `r2s` stage
(`preflight`, `collect`, `normalize`, `fit`, `validate`, `export`) with purpose,
arguments and defaults, exit codes, a worked example with real output, and the
safety default in force. Close with the exit-code table and a troubleshooting
section covering the missing-adapter error, IK failure, gate rejection, and the
stale-process symptom in which a leftover launch makes a validator report
another robot's joints as extra joints.

- [ ] **Step 4: Verify GREEN**

Run: `PYTHONPATH=src:. pytest -q tests/test_cli_documentation.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add docs/cli.md tests/test_cli_documentation.py
git commit -m "docs: document every robotctl command"
```

### Task 7: Integration Smoke and Final Verification

**Files:**
- Create: `ros_ws/smoke_pose_openarm.sh`
- Modify: `tests/test_ros_smoke.py`
- Modify: `docs/jazzy-verification.md`

**Interfaces:**
- Consumes: `ros_ws/pose_bringup.sh` and `robotctl pose`
- Produces: a recorded pose-setting verification result

- [ ] **Step 1: Add the pose smoke script**

Reuse `ros_ws/smoke_harness.sh`. Launch the bimanual stack with fake hardware,
run `robotctl pose ee --group openarm_right_arm --relative --xyz 0,0,0.03
--execute`, and assert the measured end-effector pose converges within
tolerance. Keep the displacement bounded exactly as the existing smoke plans do.

- [ ] **Step 2: Extend the harness tests**

Add the new script to `SMOKE_SCRIPTS` so it inherits the existing non-Jazzy
rejection, signal-teardown, and setup-sourcing regression coverage.

- [ ] **Step 3: Run the full static suite**

```bash
PYTHONPATH=src:. pytest -q
python3 -m compileall -q src tests tools
bash -n ros_ws/pose_bringup.sh ros_ws/smoke_pose_openarm.sh
```

Expected: PASS.

- [ ] **Step 4: Run every smoke test from a clean process table**

Run the four smoke scripts one at a time, killing leftover `ros2 launch`,
`gz sim`, `robot_state_publisher`, and `parameter_bridge` processes between
runs. A leftover launch keeps publishing `/joint_states` and makes the
validator report the other robot's joints as extra joints.

Expected: four `smoke test passed` results.

- [ ] **Step 5: Re-verify vendor provenance**

Run the four pinned verification commands from `docs/jazzy-verification.md`.

Expected: four `snapshot verified` results.

- [ ] **Step 6: Record and commit**

Append a pose-setting section to `docs/jazzy-verification.md` with the exact
commands, statuses, and evidence.

```bash
git add ros_ws/smoke_pose_openarm.sh tests/test_ros_smoke.py docs/jazzy-verification.md
git commit -m "test: verify pose setting on fake hardware"
```
