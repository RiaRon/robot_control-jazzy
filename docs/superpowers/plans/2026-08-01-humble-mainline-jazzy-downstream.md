# Humble Mainline, Jazzy Downstream Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a locally committed Jazzy downstream branch from the latest Humble mainline, preserving the current Jazzy GUI/servo work and proving hardware-free operation.

**Architecture:** Create a standalone clone under `/tmp` because the managed source repository's Git metadata is read-only. Start the integration branch at `origin/humble`, restore only the valid Jazzy distribution surface from `2433a30`, then apply the intentional dirty-worktree GUI/servo delta and adapt it to Humble's current common core. Export a Git bundle and verification report into the writable workspace without changing either source checkout.

**Tech Stack:** Git, Python 3.12, pytest, ROS 2 Jazzy, MoveIt 2, ros2_control, RViz, Bash, YAML

## Global Constraints

- `humble` is the canonical mainline and must not be modified.
- Do not reset, clean, stage, or commit the dirty Jazzy worktree.
- Do not delete or include unexplained numeric/shell-fragment-like untracked files.
- Do not push or rewrite `origin/jazzy`.
- Physical CAN, trajectory, and effort publication are forbidden during verification.
- `--execute` remains the only authority for physical publication.
- The integration baseline is the fetched `origin/humble`; record its exact SHA.
- The valid Jazzy distribution baseline is `2433a30`.
- Work in `/tmp/robot-control-jazzy-integration-*`; export durable results under `artifacts/jazzy-integration/`.

---

## File and Ownership Map

- `/tmp/.../`: standalone integration clone and writable Git metadata.
- `artifacts/jazzy-integration/local-gui.patch`: preserved intentional dirty Jazzy changes.
- `artifacts/jazzy-integration/local-gui-files.txt`: tracked/untracked preservation inventory.
- `artifacts/jazzy-integration/jazzy-downstream.bundle`: committed integration branch.
- `artifacts/jazzy-integration/verification.md`: exact SHAs, commands, results, skips, and deferred physical checks.
- `.rosdistro`, `README.md`, `docs/cli.md`: Jazzy distribution/operator surface.
- `ros_ws/*.sh`, `ros_ws/config/`, Jazzy tests: Jazzy build, bringup, GUI, and verification surface.
- `vendor_metadata/` and `ros_ws/src/`: Jazzy vendor snapshots and ROS distribution compatibility.
- `src/robot_control/`: Humble-owned common core; only interface adaptation for GUI/servo is permitted.
- `src/robot_control/ik_follow.py`, `src/robot_control/servo.py`: preserved local GUI/servo units.

### Task 1: Preserve and Classify the Dirty Jazzy Worktree

**Files:**
- Create: `artifacts/jazzy-integration/local-gui.patch`
- Create: `artifacts/jazzy-integration/local-gui-files.txt`
- Inspect: `.worktrees/jazzy/README.md`
- Inspect: `.worktrees/jazzy/docs/cli.md`
- Inspect: `.worktrees/jazzy/src/robot_control/{cli.py,ros_adapter.py,ik_follow.py,servo.py}`
- Inspect: `.worktrees/jazzy/tests/{test_cli.py,test_ros_adapter.py,test_ik_follow.py,test_servo.py}`

**Interfaces:**
- Consumes: dirty Jazzy worktree at local branch `jazzy`.
- Produces: a patch for tracked changes and explicit content copies for approved untracked source/docs/tests.

- [ ] **Step 1: Record source identities and status**

Run:

```bash
git -C /home/user/rl_ws/robot_control rev-parse origin/humble
git -C /home/user/rl_ws/robot_control/.worktrees/jazzy rev-parse HEAD
git -C /home/user/rl_ws/robot_control/.worktrees/jazzy status --short
```

Expected: Humble SHA is recorded; Jazzy HEAD is `2433a30...`; dirty files are listed without modification.

- [ ] **Step 2: Classify untracked files by content**

Run `file`, `wc -c`, and bounded `sed -n '1,80p'` on every untracked file. Mark only source, tests, plans, specs, and operator documentation as intentional. Numeric/shell-fragment-like files stay excluded and untouched.

- [ ] **Step 3: Export tracked and approved untracked changes**

Use `git diff --binary HEAD` for tracked files. Append approved untracked files as `/dev/null` patches using `git diff --no-index --binary /dev/null <file>`. Write the exact included/excluded inventory beside the patch.

- [ ] **Step 4: Verify preservation completeness**

Run:

```bash
git apply --stat artifacts/jazzy-integration/local-gui.patch
git apply --check artifacts/jazzy-integration/local-gui.patch
```

Expected: patch parses; file inventory matches intentional GUI/servo work; original worktree status is unchanged.

### Task 2: Create the Standalone Humble-Based Integration Branch

**Files:**
- Create: `/tmp/robot-control-jazzy-integration-*/.git/`
- Create branch: `integration/humble-mainline-jazzy-downstream`

**Interfaces:**
- Consumes: read-only source repository and exact `origin/humble` SHA from Task 1.
- Produces: writable branch whose initial tree and commit equal `origin/humble`.

- [ ] **Step 1: Clone locally without network access**

Create a unique directory with `mktemp -d`, then run `git clone --no-hardlinks /home/user/rl_ws/robot_control <dir>/repo`.

- [ ] **Step 2: Create the integration branch at the recorded SHA**

Run:

```bash
git -C <dir>/repo switch -c integration/humble-mainline-jazzy-downstream <humble-sha>
```

Expected: `HEAD` equals the recorded Humble SHA and `.rosdistro` contains `humble` before the downstream overlay.

- [ ] **Step 3: Record the baseline contract**

Run the offline Python suite and `robotctl r2s preflight` with the project's required `PYTHONPATH`. Record failures caused solely by missing external asset paths separately; do not alter behavior to mask them.

### Task 3: Restore the Jazzy-Owned Distribution Surface

**Files:**
- Modify: `.rosdistro`
- Modify: `README.md`
- Create/Modify: `docs/cli.md`, `docs/jazzy-verification.md`
- Create/Modify: `ros_ws/build.sh`, `ros_ws/install_dependencies_jazzy.sh`, `ros_ws/pose_bringup.sh`
- Create/Modify: `ros_ws/config/effort_controllers.yaml`, `ros_ws/load_effort_controllers.sh`
- Create/Modify: `ros_ws/smoke_*.sh`, `tools/pose_smoke.py`, `tools/ros_smoke.py`
- Modify: Jazzy-owned `ros_ws/src/` package files and `vendor_metadata/`
- Restore: `tests/test_jazzy_build_wrapper.py`, `tests/test_jazzy_dependencies.py`, `tests/test_dg5f_jazzy_contract.py`, `tests/test_pose_bringup.py`, `tests/test_ros_smoke.py`, `tests/test_cli_documentation.py`

**Interfaces:**
- Consumes: valid Jazzy tree at `2433a30` and Humble common core branch from Task 2.
- Produces: Jazzy distro/build/MoveIt/Gazebo surface without replacing Humble-owned common modules.

- [ ] **Step 1: Write/restore failing Jazzy distribution contract tests**

Restore the six Jazzy-specific test modules from `2433a30`. Run the distro, build-wrapper, dependency, repository-layout, vendor, and MoveIt contract subsets.

Expected: failures show the Humble `.rosdistro`, wrappers, vendor tree, and missing Jazzy scripts.

- [ ] **Step 2: Generate the Jazzy surface path list**

Derive the path list from the `68b34fd` merge's documented exclusion boundary and verify each path against `2433a30`. Explicitly exclude `src/robot_control/*.py` and common tests from bulk restoration.

- [ ] **Step 3: Restore distro-owned paths from `2433a30`**

Use `git restore --source=2433a30 -- <explicit paths>` inside the standalone clone. Restore Jazzy `.rosdistro`, wrappers, MoveIt/RViz/Gazebo files, vendor metadata/patches, docs, and Jazzy-only tests.

- [ ] **Step 4: Run static Jazzy contracts**

Run:

```bash
PYTHONPATH=src:. pytest -q \
  tests/test_distro_neutrality.py \
  tests/test_jazzy_build_wrapper.py \
  tests/test_jazzy_dependencies.py \
  tests/test_dg5f_jazzy_contract.py \
  tests/test_pose_bringup.py \
  tests/test_moveit_contract.py \
  tests/test_vendor_snapshot.py
bash -n ros_ws/*.sh
```

Expected: restored-surface tests pass or expose precise compatibility conflicts with the newer common core.

- [ ] **Step 5: Commit the Jazzy distribution overlay**

Commit only the explicit downstream surface with message `build: restore Jazzy downstream distribution surface`.

### Task 4: Reconcile Profile, Controller, and Asset Contracts

**Files:**
- Modify: `src/robot_control/profile.py`
- Modify: `src/robot_control/profiles/openarm_tesollo.yaml`
- Modify: `tests/test_profile.py`
- Modify: `tests/test_moveit_contract.py`
- Modify: relevant Jazzy controller YAML under `ros_ws/src/openarm_ros2/`

**Interfaces:**
- Consumes: Humble `RobotProfile`, `Group.asset_tip_link`, `RobotProfile.asset_urdf_path`, and Jazzy controller/SRDF surface.
- Produces: one Jazzy profile whose actions, tip frames, asset frames, and endpoints match both the common core and restored runtime.

- [ ] **Step 1: Add failing Jazzy profile contract tests**

Tests must assert `.rosdistro` selects the Jazzy endpoint, every executable group's action exists in Jazzy controller configuration, `tip_link` exists in the Jazzy SRDF, and `asset_tip_link` exists in the canonical asset URDF when available.

- [ ] **Step 2: Run tests and capture exact mismatches**

Run the new tests with `PYTHONPATH=src:. pytest -q`. Expected failures identify gripper-action, TCP-frame, manifest, or asset-path differences rather than generic import errors.

- [ ] **Step 3: Apply the minimal Jazzy adapter/configuration changes**

Keep Humble's profile parser and canonical URDF fields. Change only Jazzy endpoint/action/controller values or restored Jazzy controller configuration needed to make the declared interfaces real. Do not add distro conditionals to fitting or safety code.

- [ ] **Step 4: Run profile, MoveIt, controller, and artifact tests**

Run `tests/test_profile.py`, `tests/test_moveit_contract.py`, `tests/test_effort_controllers.py`, `tests/test_canonical_urdf.py`, `tests/test_bundle_identified.py`, and `tests/test_sweep_artifacts.py`.

- [ ] **Step 5: Commit compatibility reconciliation**

Commit with message `fix: align Jazzy runtime contracts with the Humble core`.

### Task 5: Apply and Adapt the Current GUI/Servo Work

**Files:**
- Create: `src/robot_control/ik_follow.py`
- Create: `src/robot_control/servo.py`
- Modify: `src/robot_control/cli.py`
- Modify: `src/robot_control/ros_adapter.py`
- Modify: `tests/test_cli.py`, `tests/test_ros_adapter.py`
- Create: `tests/test_ik_follow.py`, `tests/test_servo.py`
- Modify: `README.md`, `docs/cli.md`
- Create: `docs/pose-follow.md`

**Interfaces:**
- Consumes: Task 1 patch, Humble `CommandGate`, `RosAdapter`, `RobotProfile`, and Jazzy marker/MoveIt endpoints.
- Produces: dry-run-capable one-shot and continuous marker control using the latest common safety and profile interfaces.

- [ ] **Step 1: Apply the preserved GUI patch with rejects disabled**

Run `git apply --3way <local-gui.patch>` in the standalone clone. If a hunk conflicts, leave the affected file uncommitted and reconstruct the change from the patch against Humble's version; never replace the whole Humble file with the older Jazzy copy.

- [ ] **Step 2: Run focused GUI/servo tests to observe failures**

Run:

```bash
PYTHONPATH=src:. pytest -q \
  tests/test_ik_follow.py tests/test_servo.py \
  tests/test_ros_adapter.py tests/test_cli.py
```

Expected: preservation tests pass where interfaces are unchanged; failures pinpoint newer Humble CLI/profile/safety signatures.

- [ ] **Step 3: Adapt GUI callers to Humble interfaces**

Preserve the independent `ik_follow` and `servo` units. Adapt CLI parser/dispatch and ROS adapter calls to the current Humble signatures. Every execution method receives or derives an explicit `execute: bool`; dry-run returns the target/plan without creating publishers, action goals, or torque commands.

- [ ] **Step 4: Add missing dry-run regression tests**

Tests must use recording backends and assert zero publish/action calls for pose, marker, follow, and servo commands without `--execute`. Add cases for unreachable targets, stale/missing marker data, joint clamps, velocity clamps, timeout/stop, and malformed group/tip configuration.

- [ ] **Step 5: Run the focused suite until green**

Run the same focused command plus `tests/test_safety.py`, `tests/test_kinematics.py`, and `tests/test_pose_design.py`.

- [ ] **Step 6: Commit GUI/servo integration**

Commit with message `feat: integrate Jazzy marker GUI and servo downstream`.

### Task 6: Verify Humble Tuning and Real2Sim Parity on Jazzy

**Files:**
- Modify only if a failing distro-neutral test demonstrates a Jazzy adapter defect: `src/robot_control/ros_adapter.py`, profile YAML, or Jazzy wrappers.
- Create/Modify: `artifacts/jazzy-integration/verification.md`

**Interfaces:**
- Consumes: complete downstream branch from Tasks 3–5.
- Produces: evidence that Humble's tuning/R2S behavior remains intact under Jazzy configuration.

- [ ] **Step 1: Run all offline/common tests**

Run `PYTHONPATH=src:. pytest -q`. Record pass/fail/skip counts and exact skipped reasons.

- [ ] **Step 2: Run dry-run CLI contracts**

Run help/preflight/collection and pose dry-run commands using available fixture asset paths. Verify output explicitly states publication is disabled. If the external HDGP asset is unavailable, use the repository's fixture-profile mechanism rather than weakening production manifest checks.

- [ ] **Step 3: Run static ROS/Jazzy contracts**

Run vendor snapshot, repository layout, MoveIt, controller, build/dependency wrapper, shell syntax, and smoke-script unit tests.

- [ ] **Step 4: Run installed Jazzy build and fake/headless smoke tests when available**

Source `/opt/ros/jazzy/setup.bash`, build in the standalone clone, and run only bounded fake-hardware/headless scripts. Record environment-based omissions separately. Do not run `--real`, configure CAN, or publish effort.

- [ ] **Step 5: Fix only demonstrated integration defects and rerun their focused tests**

Each fix starts with a failing regression test and stays within the ownership boundary. Common numerical defects are not patched only in Jazzy; record them for upstream Humble instead.

- [ ] **Step 6: Commit verification-only fixes and record**

Commit code fixes, if any, with focused messages. Commit the in-clone verification document with message `test: verify Jazzy downstream without hardware`.

### Task 7: Export and Validate the Integration Deliverables

**Files:**
- Create: `artifacts/jazzy-integration/jazzy-downstream.bundle`
- Create: `artifacts/jazzy-integration/verification.md`
- Update: `artifacts/jazzy-integration/local-gui-files.txt`

**Interfaces:**
- Consumes: committed standalone branch and verification evidence.
- Produces: a portable branch artifact the user can fetch without granting this environment write access to the source `.git`.

- [ ] **Step 1: Verify clean integration repository and ancestry**

Run:

```bash
git status --short
git merge-base --is-ancestor <humble-sha> HEAD
git diff --name-status <humble-sha>..HEAD
```

Expected: clean tree, Humble is an ancestor, and the downstream delta contains the intended Jazzy/GUI surface only.

- [ ] **Step 2: Run final verification commands from a clean checkout**

Clone the standalone repository locally at the integration branch into a second temporary directory and rerun the full Python suite plus static contracts. This catches untracked-file dependencies.

- [ ] **Step 3: Create and verify the Git bundle**

Run:

```bash
git bundle create <workspace>/artifacts/jazzy-integration/jazzy-downstream.bundle \
  integration/humble-mainline-jazzy-downstream
git bundle verify <workspace>/artifacts/jazzy-integration/jazzy-downstream.bundle
```

Expected: bundle verifies and advertises the integration branch.

- [ ] **Step 4: Write import instructions without executing them**

Record the exact read-only inspection and later import commands, including fetching the bundle into a new local branch. Do not update the source repository refs in this environment.

- [ ] **Step 5: Confirm both original checkouts remain untouched**

Compare final `git status --short` outputs for root Humble and dirty Jazzy against Task 1 inventories. The only allowed workspace additions are integration artifacts and the already-approved spec/plan documents.
