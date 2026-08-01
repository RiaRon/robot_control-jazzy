# MoveIt IK TCP Follow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace per-cycle differential IK with a latest-wins MoveIt IK target that the real joints continuously track from `/joint_states`.

**Architecture:** A dedicated worker owns the blocking MoveIt IK calls and atomically latches only the newest successful result. The existing follow loop anchors the RViz marker, submits coalesced position-only poses, and streams gated joint-position steps toward the latched target while measuring actual TCP and loop timing.

**Tech Stack:** Python 3, ROS 2 Jazzy `rclpy`, MoveIt `/compute_ik`, NumPy, pytest.

## Global Constraints

- Follow TCP position only and hold the measured startup orientation.
- Treat the first marker feedback as a clutch point at the measured TCP.
- Seed every IK request from the latest measured canonical joints.
- Keep only one pending IK request; stale results may never replace a newer target.
- Existing joint position, velocity, effort, and maximum-lead gates remain authoritative.
- No automated test or development command may execute the physical robot.
- Do not create Git commits.

---

### Task 1: Latest-wins IK target worker

**Files:**
- Create: `src/robot_control/ik_follow.py`
- Create: `tests/test_ik_follow.py`

**Interfaces:**
- Produces: `IkRequest(sequence: int, pose: Pose, seed: np.ndarray)`
- Produces: `IkStatus(target: np.ndarray | None, submitted: int, succeeded: int, failed: int, superseded: int)`
- Produces: `LatestIkWorker(solve: Callable[[Pose, np.ndarray], np.ndarray])`
- Produces methods: `submit(pose, seed)`, `snapshot() -> IkStatus`, `close()`

- [ ] **Step 1: Write failing tests**

Test that a successful solve latches canonical joints, several pending submissions coalesce to the newest request, an old in-flight result cannot replace a newer request, failures preserve the last valid target, and `close()` joins the worker.

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_ik_follow.py
```

Expected: collection fails because `robot_control.ik_follow` does not exist.

- [ ] **Step 3: Implement the minimal worker**

Use one condition variable, one daemon thread, monotonically increasing sequence numbers, a single pending request slot, and copied finite NumPy arrays. Catch `IkFailed` and other solver exceptions as failed requests; never expose a partially updated target.

- [ ] **Step 4: Verify GREEN**

Run the same command and expect all worker tests to pass.

### Task 2: Integrate latched MoveIt IK into `pose follow`

**Files:**
- Modify: `src/robot_control/cli.py`
- Modify: `src/robot_control/ros_adapter.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `LatestIkWorker` and `IkStatus` from Task 1.
- Reuses: `RosAdapter.solve_ik(pose, seed)` and `CommandGate.follow(target, measured, period)`.
- Produces: `_follow_loop(..., ik_worker)` that submits anchored `Pose` targets and tracks `snapshot().target`.

- [ ] **Step 1: Write failing CLI tests**

Add deterministic fake-worker tests proving:

```python
assert first_submitted_pose.position == pytest.approx(start_tcp)
assert submitted_seed == pytest.approx(latest_measured_joints)
assert streamed[-1] moves_toward latched_joint_target
assert failed_ik_keeps_previous_latched_target
assert marker_orientation_does_not_change_submitted_pose.orientation
```

Also assert actual loop Hz and IK request/success/failure/superseded counts appear in the report.

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_cli.py -k 'pose_follow'
```

Expected: new assertions fail because follow still calls `CartesianPI` and Jacobian `delta_q`.

- [ ] **Step 3: Implement worker lifecycle**

Create a second non-executing `RosAdapter` dedicated to `solve_ik`, pass its bound solver to `LatestIkWorker`, and close the worker before closing the adapter. Keep the main adapter exclusively responsible for marker/joint subscriptions and controller publications.

- [ ] **Step 4: Replace differential IK control**

Anchor the first marker at startup TCP, construct every requested `Pose` in `world` with startup orientation, submit only when position changed by at least `--tolerance`, and use the latest successful IK result as the persistent joint target. Feed the target and current measured joints through `CommandGate.follow`; do not accumulate Jacobian increments.

- [ ] **Step 5: Preserve speed behavior**

Before `CommandGate.follow`, limit progress toward the latched joint target so the FK displacement between the current command and candidate does not exceed `--max-tcp-speed * actual_cycle_seconds`. Use the measured cycle duration clamped to a safe upper bound, rather than assuming 10 ms when hardware delivers slower joint states.

- [ ] **Step 6: Report measured behavior**

Measure elapsed monotonic time, joint-state wait duration, samples/elapsed Hz, worker statistics, TCP error to the currently submitted Cartesian target, and maximum target-joint error.

- [ ] **Step 7: Verify GREEN**

Run the focused command and expect all follow tests to pass.

### Task 3: Documentation and complete verification

**Files:**
- Modify: `README.md`
- Modify: `docs/cli.md`
- Modify: `docs/pose-follow.md`

**Interfaces:**
- Documents the behavior implemented in Tasks 1–2 without changing public command syntax.

- [ ] **Step 1: Update operator documentation**

Explain that RViz ghost joints are not motor feedback, MoveIt IK is computed asynchronously from measured joint seeds, the last successful joint target remains latched, and displayed 100 Hz is a target while the report contains achieved Hz.

- [ ] **Step 2: Run focused tests**

```bash
PYTHONPATH=src:. pytest -q tests/test_ik_follow.py tests/test_cli.py -k 'ik_follow or pose_follow'
```

- [ ] **Step 3: Run the full suite**

```bash
PYTHONPATH=src:. pytest -q
```

- [ ] **Step 4: Check patch integrity**

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; all implementation, test, and documentation changes remain uncommitted.
