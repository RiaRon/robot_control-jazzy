# Humble Driver Snapshot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import the validated OpenArm and Tesollo ROS 2 Humble driver snapshots into the `robot_control` Humble branch and make the branch build and verify only its local sources.

**Architecture:** Vendor sources live under `ros_ws/src` as immutable provenance-tracked snapshots, while project-owned canonical control and Real2Sim code stays outside those trees. A verification tool compares every imported file against an upstream checkout using an explicit exclusion list. The Humble build wrapper rejects other ROS distributions and isolates all generated products inside `robot_control/ros_ws`.

**Tech Stack:** Git, Python 3.10+, ROS 2 Humble, colcon, pytest, Bash, YAML

## Global Constraints

- Source branch is the long-lived `humble` branch.
- OpenArm source revision is `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`.
- Tesollo source revision is `a68335919ee490d5293581574acc7aff12fe969d`.
- OpenArm CAN source revision is `c32ecd31da267967f0c913c2118c843177d88b91`.
- OpenArm description comes from clean repository
  `teleopration_openarm_tesollo@c8696ebfd64ea08ee0a212a9bae21055b6f381bc`
  at subpath `src/openarm_description`.
- Preserve upstream license files and copyright headers.
- Do not modify or delete the source repositories under `../repo`.
- Do not issue real-hardware commands.
- Do not add Jazzy driver sources or configuration to this branch.
- Keep `build`, `install`, and `log` products under `ros_ws` and out of Git.

---

### Task 1: Repository Hygiene and Humble Branch Contract

**Files:**
- Create: `.gitignore`
- Create: `.rosdistro`
- Create: `tests/test_repository_layout.py`
- Remove: `ros_ws/humble/build.sh`
- Remove: `ros_ws/humble/drivers.repos`
- Remove: `ros_ws/jazzy/build.sh`
- Remove: `ros_ws/jazzy/drivers.repos`
- Modify: `ros_ws/README.md`

**Interfaces:**
- Produces: `.rosdistro` containing exactly `humble`
- Produces: branch-local `ros_ws/src`, `ros_ws/build`, `ros_ws/install`, and `ros_ws/log` conventions

- [ ] **Step 1: Write failing repository layout tests**

```python
from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_branch_declares_humble_only():
    assert (ROOT / ".rosdistro").read_text().strip() == "humble"
    assert not (ROOT / "ros_ws/jazzy").exists()


def test_generated_products_are_ignored():
    ignored = (ROOT / ".gitignore").read_text()
    for pattern in ("__pycache__/", ".pytest_cache/", "ros_ws/build/", "ros_ws/install/", "ros_ws/log/", "*.hdf5", "*.db3"):
        assert pattern in ignored
```

- [ ] **Step 2: Run the layout tests and verify they fail**

Run: `PYTHONPATH=src pytest -q tests/test_repository_layout.py`

Expected: FAIL because `.rosdistro` and top-level `.gitignore` do not exist and Jazzy files remain.

- [ ] **Step 3: Add the Humble contract and ignore rules**

Write `.rosdistro` as `humble`, add the exact patterns asserted above plus `.venv/`, `*.pyc`, `*.egg-info/`, `dist/`, `ros_ws/src/.snapshot-tmp/`, `artifacts/`, `bags/`, and remove both old distribution overlay directories. Rewrite `ros_ws/README.md` to describe the single-distribution branch model.

- [ ] **Step 4: Run the layout and existing core tests**

Run: `PYTHONPATH=src pytest -q tests/test_repository_layout.py tests/test_profile.py tests/test_interface.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add .gitignore .rosdistro ros_ws tests/test_repository_layout.py
git commit -m "chore: establish Humble branch layout"
```

### Task 2: Snapshot Provenance and Verification

**Files:**
- Create: `vendor_metadata/openarm/UPSTREAM.yaml`
- Create: `vendor_metadata/tesollo/UPSTREAM.yaml`
- Create: `vendor_metadata/openarm_can/UPSTREAM.yaml`
- Create: `vendor_metadata/openarm_description/UPSTREAM.yaml`
- Create: `tools/verify_vendor_snapshot.py`
- Create: `tests/test_vendor_snapshot.py`

**Interfaces:**
- Produces: `verify_snapshot(metadata_path: Path, source_checkout: Path, snapshot: Path) -> list[str]`
- Metadata keys: `name`, `repository`, `branch`, `commit`, `license`, `excluded`

- [ ] **Step 1: Write failing provenance tests**

```python
from pathlib import Path
import yaml

from tools.verify_vendor_snapshot import verify_snapshot

ROOT = Path(__file__).parents[1]


def test_metadata_pins_validated_commits():
    openarm = yaml.safe_load((ROOT / "vendor_metadata/openarm/UPSTREAM.yaml").read_text())
    tesollo = yaml.safe_load((ROOT / "vendor_metadata/tesollo/UPSTREAM.yaml").read_text())
    assert openarm["commit"] == "4e837e1d0dae692ff67b560b69d8d281d7a8d4ed"
    assert tesollo["commit"] == "a68335919ee490d5293581574acc7aff12fe969d"


def test_snapshot_verifier_detects_changed_file(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "LICENSE").write_text("license")
    (snapshot / "LICENSE").write_text("modified")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: [.git, .github]\n")
    assert verify_snapshot(metadata, source, snapshot) == ["content mismatch: LICENSE"]
```

- [ ] **Step 2: Run the tests and verify missing-module failure**

Run: `PYTHONPATH=src:. pytest -q tests/test_vendor_snapshot.py`

Expected: FAIL because `tools.verify_vendor_snapshot` does not exist.

- [ ] **Step 3: Implement deterministic tree comparison**

Implement `verify_snapshot` using `Path.rglob`, relative POSIX paths, `fnmatch`, and SHA-256 file content comparison. Report sorted `missing`, `unexpected`, and `content mismatch` messages. The CLI accepts `--metadata`, `--source`, and `--snapshot`, prints each mismatch, and exits 1 when any mismatch exists.

- [ ] **Step 4: Add exact provenance metadata**

OpenArm metadata uses repository `https://github.com/enactic/openarm_ros2.git`, branch `main`, Apache-2.0, and excludes `.git/**`, `.github/**`, `build/**`, `install/**`, `log/**`, and Python/pytest caches. Tesollo uses `https://github.com/tesollodelto/delto_m_ros2.git`, branch `humble`, BSD-3-Clause, and the same exclusions.

- [ ] **Step 5: Run provenance unit tests**

Run: `PYTHONPATH=src:. pytest -q tests/test_vendor_snapshot.py`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add vendor_metadata tools tests/test_vendor_snapshot.py
git commit -m "feat: add vendor snapshot provenance verification"
```

### Task 3: Import Validated Driver Snapshots

**Files:**
- Create: `ros_ws/src/openarm_ros2/**`
- Create: `ros_ws/src/delto_m_ros2/**`
- Create: `ros_ws/src/openarm_can/**`
- Create: `ros_ws/src/openarm_description/**`

**Interfaces:**
- Consumes: provenance exclusion patterns from Task 2
- Produces: complete source snapshots with no nested `.git` directories

- [ ] **Step 1: Verify source repositories and revisions**

Run:

```bash
git -C ../repo/openarm/openarm_ros2 rev-parse HEAD
git -C ../repo/tesollo/delto_m_ros2 rev-parse HEAD
git -C ../repo/openarm/openarm_ros2 status --short
git -C ../repo/tesollo/delto_m_ros2 status --short
```

Expected: exact pinned commits and clean source trees.

- [ ] **Step 2: Copy snapshots with the documented exclusions**

Use `rsync -a` from each source checkout to its `ros_ws/src` destination while excluding `.git`, `.github`, `build`, `install`, `log`, `__pycache__`, `.pytest_cache`, `*.pyc`, and `*.pyo`. Do not use symlinks.

- [ ] **Step 3: Run snapshot verification against both sources**

Run:

```bash
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py --metadata vendor_metadata/openarm/UPSTREAM.yaml --source ../repo/openarm/openarm_ros2 --snapshot ros_ws/src/openarm_ros2
PYTHONPATH=src:. python3 tools/verify_vendor_snapshot.py --metadata vendor_metadata/tesollo/UPSTREAM.yaml --source ../repo/tesollo/delto_m_ros2 --snapshot ros_ws/src/delto_m_ros2
```

Expected: exit 0 with `snapshot verified`.

- [ ] **Step 4: Confirm rollback repositories were not modified**

Run:

```bash
git -C ../repo/openarm/openarm_ros2 status --short
git -C ../repo/tesollo/delto_m_ros2 status --short
```

Expected: no output.

- [ ] **Step 5: Commit**

```bash
git add ros_ws/src
git commit -m "vendor: import validated Humble driver snapshots"
```

### Task 4: Humble-Only Build Wrapper

**Files:**
- Create: `ros_ws/build.sh`
- Create: `tests/test_humble_build_wrapper.py`
- Modify: `README.md`

**Interfaces:**
- Produces: `ros_ws/build.sh` that rejects non-Humble environments before invoking colcon
- Build products: `ros_ws/build`, `ros_ws/install`, `ros_ws/log`

- [ ] **Step 1: Write failing build-wrapper behavior tests**

Create a fake `colcon` executable that records its arguments. Verify `ROS_DISTRO=jazzy` exits 2 without calling it, while `ROS_DISTRO=humble` calls it with `--base-paths <root>/ros_ws/src`, `--build-base`, `--install-base`, and `--log-base` under `ros_ws`.

- [ ] **Step 2: Run the tests and verify failure**

Run: `PYTHONPATH=src pytest -q tests/test_humble_build_wrapper.py`

Expected: FAIL because `ros_ws/build.sh` does not exist.

- [ ] **Step 3: Implement the build wrapper**

Use Bash strict mode, resolve the script directory, require `ROS_DISTRO=humble`, and invoke `colcon build --base-paths ... --build-base ... --install-base ... --log-base ... --symlink-install`.

- [ ] **Step 4: Document setup and build**

Update `README.md` with Humble prerequisites, `rosdep install --from-paths ros_ws/src --ignore-src -r -y`, `./ros_ws/build.sh`, and the branch policy for future Jazzy work.

- [ ] **Step 5: Run wrapper, shell, and core tests**

Run:

```bash
PYTHONPATH=src pytest -q tests/test_humble_build_wrapper.py
bash -n ros_ws/build.sh
PYTHONPATH=src pytest -q
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws/build.sh README.md tests/test_humble_build_wrapper.py
git commit -m "build: add isolated Humble driver workspace"
```

### Task 5: Dependency and Colcon Verification

**Files:**
- Create: `docs/humble-verification.md`

**Interfaces:**
- Consumes: branch-local `ros_ws/src` and `ros_ws/build.sh`
- Produces: recorded build result and any environment-only blockers

- [ ] **Step 1: Inspect package graph**

Run: `colcon list --base-paths ros_ws/src`

Expected: OpenArm and Tesollo packages are discovered from branch-local source only.

- [ ] **Step 2: Check dependencies without modifying source**

Run: `rosdep check --from-paths ros_ws/src --ignore-src`

Record exact missing system dependencies in `docs/humble-verification.md`.

- [ ] **Step 3: Build when Humble is active**

Run: `ROS_DISTRO=humble ./ros_ws/build.sh`

Expected: all imported packages build, or the document records the exact dependency/environment blocker. Never substitute a Jazzy environment for this check.

- [ ] **Step 4: Run final verification**

Run:

```bash
PYTHONPATH=src:. pytest -q
python3 -m compileall -q src tests tools
bash -n ros_ws/build.sh
git diff --check
git status --short
```

Expected: tests and syntax checks pass; only intended verification documentation remains uncommitted.

- [ ] **Step 5: Commit verification record**

```bash
git add docs/humble-verification.md
git commit -m "docs: record Humble snapshot verification"
```
