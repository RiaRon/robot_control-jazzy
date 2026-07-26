# Jazzy DG5F and OpenArm Readiness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate the supported Jazzy OpenArm plus Tesollo DG5F package set on Ubuntu 24.04 without contacting physical hardware.

**Architecture:** Preserve pinned upstream driver trees while declaring an ordered, deterministic Tesollo Jazzy patch set in provenance metadata. Build only the supported OpenArm/DG5F graph, then validate static contracts, fake hardware, and headless Gazebo through bounded smoke-test scripts that cannot fall back to physical interfaces.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, gz_ros2_control, ros2_control, colcon, Python 3.12, pytest, Bash, YAML, unified diff

## Global Constraints

- Work only on the long-lived `jazzy` branch.
- Target Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.
- Tesollo support is DG5F only; retain DG3F-M and DG4F sources without building their Gazebo packages.
- Do not open CAN, publish to physical hardware controllers, or connect to a Tesollo device.
- OpenArm hardware testing stops at fake hardware until a later operator-supervised procedure.
- Preserve upstream licenses and the pinned Tesollo commit `3926c2eab8d011046f64874d6252213b2cf18f48`.
- Every local vendor change must be represented by an ordered patch and verified fail-closed.

---

### Task 1: Patch-Aware Snapshot Provenance

**Files:**
- Modify: `tools/verify_vendor_snapshot.py`
- Modify: `tests/test_vendor_snapshot.py`
- Create: `vendor_metadata/tesollo/patches/0001-dg5f-gz-ros2-control-jazzy.patch`
- Modify: `vendor_metadata/tesollo/UPSTREAM.yaml`

**Interfaces:**
- Consumes: metadata key `patches`, an ordered list of paths relative to the metadata directory
- Produces: `materialize_expected_tree(metadata_path: Path, source_checkout: Path, destination: Path) -> None`
- Produces: existing `verify_snapshot(...) -> list[str]` with patch-aware behavior

- [ ] **Step 1: Write failing verifier tests**

Add tests that create a source file, a unified diff under `patches/`, and a
metadata document containing:

```yaml
excluded: []
patches:
  - patches/0001-change.patch
```

Assert that `verify_snapshot` accepts a snapshot containing the patched
content. Add separate assertions that an absent patch and an undeclared edit
return errors.

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```bash
PYTHONPATH=src:. pytest -q \
  tests/test_vendor_snapshot.py::test_snapshot_verifier_applies_declared_patches \
  tests/test_vendor_snapshot.py::test_snapshot_verifier_rejects_missing_patch
```

Expected: FAIL because patch metadata is ignored.

- [ ] **Step 3: Implement deterministic patch materialization**

Copy included source files to a `TemporaryDirectory`, resolve each patch below
`metadata_path.parent`, reject missing or escaping paths, and invoke:

```python
subprocess.run(
    ["patch", "--batch", "--forward", "-p1", "-i", str(patch_path)],
    cwd=destination,
    text=True,
    capture_output=True,
    check=True,
)
```

Compare the materialized tree with the checked-in snapshot using the existing
sorted missing/unexpected/content-mismatch messages. Convert patch failures to
a single deterministic `patch failed: <relative path>` error.

- [ ] **Step 4: Generate and declare the DG5F Jazzy patch**

The patch must make only these substitutions in DG5F files:

```text
ign_ros2_control                         -> gz_ros2_control
ign_ros2_control/IgnitionSystem          -> gz_ros2_control/GazeboSimSystem
libign_ros2_control-system.so            -> libgz_ros2_control-system.so
ign_ros2_control::IgnitionROS2ControlPlugin
                                           -> gz_ros2_control::GazeboSimROS2ControlPlugin
IGN_GAZEBO_RESOURCE_PATH                 -> GZ_SIM_RESOURCE_PATH
```

Apply it to right, left, and both xacros plus DG5F package/launch files. Add the
patch path to `vendor_metadata/tesollo/UPSTREAM.yaml`. Each xacro also gains a
`use_fake_hardware` argument: true selects
`mock_components/GenericSystem`, while false selects
`gz_ros2_control/GazeboSimSystem`. Each DG5F launch file declares the same
argument and forwards it to xacro; its default remains false.

- [ ] **Step 5: Verify provenance and tests**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_vendor_snapshot.py
SOURCE_TMP="$(mktemp -d /tmp/robot-control-tesollo.XXXXXX)"
mkdir -p "$SOURCE_TMP/source"
git -C /home/user/rl_ws/repo/tesollo/delto_m_ros2 \
  archive 3926c2eab8d011046f64874d6252213b2cf18f48 |
  tar -x -C "$SOURCE_TMP/source"
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py \
  --metadata vendor_metadata/tesollo/UPSTREAM.yaml \
  --source "$SOURCE_TMP/source" \
  --snapshot ros_ws/src/delto_m_ros2
```

Expected: PASS and `snapshot verified`.

- [ ] **Step 6: Commit**

```bash
git add tools tests/test_vendor_snapshot.py vendor_metadata/tesollo ros_ws/src/delto_m_ros2/dg5f_gz
git commit -m "fix: migrate DG5F simulation to Jazzy gz_ros2_control"
```

### Task 2: Supported Jazzy Package Graph and Dependency Helper

**Files:**
- Modify: `ros_ws/build.sh`
- Create: `ros_ws/install_dependencies_jazzy.sh`
- Create: `ros_ws/supported-packages.txt`
- Modify: `tests/test_jazzy_build_wrapper.py`
- Create: `tests/test_jazzy_dependencies.py`
- Modify: `README.md`
- Modify: `ros_ws/README.md`

**Interfaces:**
- Produces: newline-separated `ros_ws/supported-packages.txt`
- Produces: explicit operator command `./ros_ws/install_dependencies_jazzy.sh`
- Build wrapper passes `--packages-up-to` for supported leaf packages and never selects `dg3f_m_gz` or `dg4f_gz`

- [ ] **Step 1: Write failing package-selection and dependency tests**

Extend the fake-colcon test to assert the arguments include:

```text
--packages-up-to
openarm
openarm_bimanual_moveit_config
dg5f_driver
dg5f_gz
```

Assert the arguments exclude `dg3f_m_gz` and `dg4f_gz`. Test that the
dependency helper rejects non-Jazzy environments before invoking fake `sudo`
or `rosdep`.

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_jazzy_build_wrapper.py tests/test_jazzy_dependencies.py
```

Expected: FAIL because supported package selection and the helper do not exist.

- [ ] **Step 3: Add supported package graph**

Store exact top-level targets in `supported-packages.txt`:

```text
openarm
openarm_bimanual_moveit_config
dg5f_driver
dg5f_gz
```

Make `build.sh` read non-empty, non-comment lines into a Bash array and invoke
`colcon ... build --packages-up-to "${SUPPORTED_PACKAGES[@]}"`.

- [ ] **Step 4: Add explicit dependency installer**

The strict-mode helper must require `ROS_DISTRO=jazzy`, then run:

```bash
sudo apt-get update
sudo apt-get install -y \
  libcli11-dev \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -r -y \
  --skip-keys "ign_ros2_control"
```

Document that sudo prompts are operator-controlled.

- [ ] **Step 5: Verify GREEN**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_jazzy_build_wrapper.py tests/test_jazzy_dependencies.py
bash -n ros_ws/build.sh ros_ws/install_dependencies_jazzy.sh
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws README.md tests/test_jazzy_build_wrapper.py tests/test_jazzy_dependencies.py
git commit -m "build: define supported Jazzy OpenArm DG5F graph"
```

### Task 3: DG5F Static Jazzy Contract

**Files:**
- Create: `tests/test_dg5f_jazzy_contract.py`
- Modify: DG5F patch and patched source files from Task 1 if the tests expose a mismatch

**Interfaces:**
- Consumes: canonical right-hand DG5F joint names from `openarm_tesollo.yaml`
- Validates: DG5F package dependency, plugin identifiers, xacro joint coverage, and controller YAML coverage

- [ ] **Step 1: Write contract tests**

Load the canonical profile and select source names starting with `rj_dg_`.
Assert exactly 20 names. Parse all three xacros with
`xml.etree.ElementTree`, load the right controller YAML, and assert:

```python
assert set(controller_joints) == expected_right_joints
assert package_dependencies.count("gz_ros2_control") == 1
```

Search DG5F package/xacro/launch text and reject the five legacy identifiers
listed in Task 1. Assert both fake and Gazebo plugin branches exist and every
DG5F launch forwards `use_fake_hardware` into its xacro command.

- [ ] **Step 2: Run tests and verify any remaining RED**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_dg5f_jazzy_contract.py
```

Expected: PASS after Task 1; any failure identifies an incomplete patch and
must be fixed in the declared patch, never as an undeclared edit.

- [ ] **Step 3: Re-run provenance after adjustments**

Run the Tesollo snapshot command from Task 1 and require `snapshot verified`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_dg5f_jazzy_contract.py vendor_metadata/tesollo ros_ws/src/delto_m_ros2/dg5f_gz
git commit -m "test: enforce DG5F Jazzy simulation contract"
```

### Task 4: Safe Runtime Smoke-Test Harness

**Files:**
- Create: `tools/ros_smoke.py`
- Create: `tests/test_ros_smoke.py`
- Create: `ros_ws/smoke_openarm_fake.sh`
- Create: `ros_ws/smoke_dg5f_fake.sh`
- Create: `ros_ws/smoke_dg5f_gazebo.sh`

**Interfaces:**
- Produces: `wait_for(predicate: Callable[[], bool], timeout_s: float, interval_s: float = 0.1) -> None`
- Produces: `validate_joint_state(expected: set[str], names: Sequence[str], positions: Sequence[float]) -> None`
- Shell smoke tests require `ROS_DISTRO=jazzy`, source `ros_ws/install/setup.bash`, use bounded `timeout`, and install cleanup traps

- [ ] **Step 1: Write failing harness unit tests**

Test successful waiting, timeout, exact joint coverage, finite positions, and
position error rejection. Use a 20-joint DG5F state and a deliberately missing
joint case.

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_ros_smoke.py
```

Expected: FAIL because `tools.ros_smoke` does not exist.

- [ ] **Step 3: Implement pure validation helpers**

Raise `TimeoutError` on deadline and `ValueError` for missing/extra joints,
length mismatch, non-finite positions, or error above the supplied tolerance.
Keep ROS imports outside these pure helpers.

- [ ] **Step 4: Add fail-closed smoke scripts**

Each script must:

- reject non-Jazzy environments;
- reject a missing install setup;
- pass `use_fake_hardware:=true` for OpenArm;
- launch DG5F with `use_fake_hardware:=true`, selecting
  `mock_components/GenericSystem` and never the TCP driver;
- launch DG5F Gazebo with `gui:=false`;
- wait at most 30 seconds for controllers;
- command at most `0.05 rad` from neutral;
- kill the complete child process group on success, failure, or signal.

The OpenArm script must set a sentinel invalid CAN interface
`can_interface:=robot_control_fake_only` in addition to
`use_fake_hardware:=true`.

- [ ] **Step 5: Verify unit and shell tests**

Run:

```bash
PYTHONPATH=src:. pytest -q tests/test_ros_smoke.py
bash -n ros_ws/smoke_openarm_fake.sh \
  ros_ws/smoke_dg5f_fake.sh \
  ros_ws/smoke_dg5f_gazebo.sh
```

Expected: PASS without starting ROS processes.

- [ ] **Step 6: Commit**

```bash
git add tools/ros_smoke.py tests/test_ros_smoke.py ros_ws/smoke_*.sh
git commit -m "test: add fail-closed Jazzy ROS smoke harness"
```

### Task 5: Build and Runtime Verification

**Files:**
- Modify: `docs/jazzy-verification.md`

**Interfaces:**
- Consumes: operator-installed Jazzy dependencies and the smoke scripts
- Produces: evidence record separating PASS, FAIL, and BLOCKED checks

- [ ] **Step 1: Install dependencies with operator authorization**

Run:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh
```

Expected: apt and rosdep complete. If sudo needs an interactive password,
record the exact command as BLOCKED and do not bypass authentication.

- [ ] **Step 2: Check supported rosdep graph**

Run:

```bash
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

Expected: all required dependencies installed and no unresolved
`ign_ros2_control`.

- [ ] **Step 3: Build supported packages**

Run:

```bash
./ros_ws/build.sh
```

Expected: supported package graph completes with exit code 0.

- [ ] **Step 4: Run fake and Gazebo smoke tests**

Run sequentially:

```bash
./ros_ws/smoke_openarm_fake.sh
./ros_ws/smoke_dg5f_fake.sh
./ros_ws/smoke_dg5f_gazebo.sh
```

Expected: controllers active, exact joint coverage, and bounded state round
trip. No CAN or Tesollo TCP device is opened.

- [ ] **Step 5: Record exact evidence**

Update `docs/jazzy-verification.md` with command, date, exit code, package
counts, controller states, joint counts, and any operator-level blocker.

- [ ] **Step 6: Commit**

```bash
git add docs/jazzy-verification.md
git commit -m "docs: record Jazzy DG5F OpenArm readiness"
```

### Task 6: Final Verification and Publish

**Files:**
- Modify: none unless verification reveals a defect

**Interfaces:**
- Produces: clean, pushed `origin/jazzy`

- [ ] **Step 1: Run complete static verification**

```bash
PYTHONPATH=src:. pytest -q
python3 -m compileall -q src tests tools
bash -n ros_ws/*.sh
git diff --check
```

Expected: zero failures and no syntax/diff errors.

- [ ] **Step 2: Re-verify all vendor trees**

Run the four `verify_vendor_snapshot.py` commands recorded in
`docs/jazzy-verification.md`. Expected: four `snapshot verified` messages.

- [ ] **Step 3: Confirm clean branch and remote relationship**

```bash
git status --short --branch
git log --oneline origin/jazzy..HEAD
```

Expected: only intended commits ahead of `origin/jazzy`.

- [ ] **Step 4: Push**

```bash
git push origin jazzy
```

- [ ] **Step 5: Verify pushed ref**

```bash
test "$(git rev-parse HEAD)" = "$(git rev-parse origin/jazzy)"
git status --short --branch
```

Expected: clean `jazzy...origin/jazzy`.
