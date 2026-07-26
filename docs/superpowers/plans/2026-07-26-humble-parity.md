# Bringing Humble to parity with Jazzy

## Why

Everything built since the branches split lives only on `jazzy`: pose setting,
the RViz marker servo, gravity compensation, and the whole Real2Sim
identification pipeline. `humble` is still at `77f3f63`, the original core
commit.

This is not "port a few features". Measured across the shared modules:

| module | humble | jazzy |
| --- | ---: | ---: |
| `cli.py` | 137 | 1991 |
| `identification.py` | 138 | 884 |
| `artifacts.py` | 63 | 307 |
| `calibration.py` | 115 | 264 |
| `safety.py` | 51 | 224 |
| `track.py` | 66 | 179 |
| `profile.py` | 135 | 230 |
| `interface.py` | 38 | 91 |

and three modules are absent entirely: `kinematics.py`, `ros_adapter.py`,
`srdf.py`. On the workspace side `humble` has `ros_ws/build.sh` and nothing
else — no bringup, no effort-controller loader, no smoke harness.

## The thing that makes this tractable

The standing constraint that **`import robot_control` must not require `rclpy`**
turns out to have already done most of this work. Everything except
`ros_adapter.py` is pure numpy and knows nothing about ROS, so it is
distro-neutral by construction rather than by intention.

The port is therefore two very different jobs, and keeping them apart is the
whole design:

1. **The neutral core** — eight modules and their tests, which should end up
   *byte-identical* on both branches. Anything else means every future fix has
   to be made twice and will eventually be made twice differently.
2. **The distro surface** — `ros_adapter.py`, `ros_ws/*.sh`, the vendor
   snapshot, and a handful of literals. Small, but it is where the real work is.

## What is actually distro-specific

Audited rather than assumed.

### 1. `profile.ros["jazzy"]` is written out six times

`cli.py` hardcodes the distro key at lines 446, 928, 1305, 1381, 1480 and 1896.
The profile has carried a `humble` endpoint beside the `jazzy` one since the
beginning; nothing reads it. `.rosdistro` already declares which branch this is.

### 2. The trajectory controller's state topic was renamed

`tracking_error` subscribes to `/{controller}/controller_state`. That is the
Iron-and-later name; Humble's `joint_trajectory_controller` publishes `~/state`.

This is not cosmetic — that topic is the only thing `pose gravity` measures. On
Humble the sweep would time out with "no .../controller_state within 10 s" and
every gravity scale would be unmeasurable. **Verify before porting**:

```bash
ros2 topic list | grep -E "right_joint_trajectory_controller/(state|controller_state)"
```

The message's own fields are safe either way: `joint_names` and
`error.positions` exist in both versions.

### 3. `ParallelGripperCommand` may not exist on Humble

The adapter imports `control_msgs.action.ParallelGripperCommand`, and the
profile drives `openarm_left_gripper` through it. Jazzy's `control_msgs` ships
it; Humble's is a major version behind and is expected to have `GripperCommand`
only, with `gripper_controllers/GripperActionController` in place of
`parallel_gripper_action_controller`.

Worse than absent: the import sits inside one `try` whose `except ImportError`
says *"the ROS adapter needs rclpy and the MoveIt message packages; source a ROS
2 Jazzy workspace"*. On Humble with rclpy perfectly well installed, one missing
action type would produce that message and send the operator to fix the wrong
thing. **Verify**:

```bash
ros2 interface list | grep -i gripper
```

### 4. The Gazebo plugin patch runs the other way

`vendor_metadata/tesollo/patches/0001-dg5f-gz-ros2-control-jazzy.patch` rewrites
three DG5F launch files from `ign_ros2_control` to `gz_ros2_control`. Humble
predates Gazebo Harmonic, so it needs `ign_ros2_control` (Fortress) or
`gazebo_ros2_control` (Classic) — the patch does not transfer, it inverts.

### 5. The vendor sources are different code, not the same code

| component | humble | jazzy |
| --- | --- | --- |
| `openarm_ros2` | `main` @ `4e837e1` | `jazzy` @ `8087bbc` |
| `delto_m_ros2` | `humble` @ `a683359` | `jazzy-dev` @ `3926c2e` |
| `openarm_can` | `main` @ `c32ecd3` | same |
| `openarm_description` | `main` @ `c8696eb` | same |

Two of the four are different upstream branches. Patches and their
`post_patch_sha256` entries have to be regenerated against the Humble archives,
never copied — a copied hash would assert something that was never checked.

This also means findings do not automatically carry: the hard-coded
`DEFAULT_KP = {20,20,20,20,5,5,5}` in `v10_simple_hardware.cpp` that made the
gravity scale a measured quantity is a fact about the *jazzy* branch of
`openarm_ros2`. It has to be re-read on `main`.

### 6. Python 3.10 against 3.12

Ubuntu 22.04 ships Python 3.10. Checked: no `match` statements, no 3.11+
standard library, and `from __future__ import annotations` in every module, so
`X | None` in annotations is fine. Two things to confirm rather than assume:

- `pyproject.toml` asks for `numpy>=1.24`; Humble's system numpy is older, so
  the venv story may differ from Jazzy's.
- ROS Humble's Python is 3.10; `rclpy` must come from the system interpreter,
  which is why the Jazzy README warns against a plain venv. Same warning, other
  version.

### 7. `.rosdistro` and the layout test

`tests/test_repository_layout.py` asserts `.rosdistro == "jazzy"` and that
`ros_ws/humble/` does not exist. Humble carries its own mirror of that test.
Both are right; neither should be ported verbatim.

## Global constraints

Carried forward:

- Fake hardware is the default; reaching hardware needs an explicit flag, and
  nothing publishes without `--execute`.
- Profile limits are authoritative over URDF limits.
- `import robot_control` must not require `rclpy`.
- Vendor tree changes need a declared patch and a `post_patch_sha256` update.
- Artifacts stay schema-versioned, checksummed, and tied to a profile and asset
  manifest hash.

Amended for this work:

- **The "work only on `jazzy`" constraint is lifted for the `humble` branch, and
  only for it.** The two long-lived branches stay separate; nothing here creates
  a third.

New:

- **The neutral core is identical on both branches, not merely equivalent.**
  Divergence there is the failure mode this plan exists to prevent, and it is
  invisible until someone fixes a bug on one branch only.

## Design

### Confine the distro to a named surface

Rather than a distro flag threaded through the code, the distro appears in
exactly three places:

1. `.rosdistro`, which already exists and already differs.
2. `profile.ros[distro]`, which already exists and is already populated for
   both.
3. `ros_adapter.py`, where the message types and topic names live.

Everything else reads the distro from `.rosdistro` and never names one.

That last part is testable, and the test is what keeps it true:

```python
def test_no_module_names_a_ros_distro():
    """The distro belongs in .rosdistro and the adapter, nowhere else."""
```

with `ros_adapter.py` the single declared exception. It is the same shape as
`test_core_package_imports_without_rclpy`, which has already been earning its
keep — that test is why the neutral core is portable at all.

### Merge, do not copy

The neutral modules should arrive on `humble` by merge, so that git records them
as the same content and a future fix on either branch merges cleanly. Copying
would produce eight files that look identical and share no history, and the
first conflict would have to be resolved by hand with no base to resolve
against.

`ros_adapter.py`, `ros_ws/*.sh` and `vendor_metadata/` are expected to conflict
and are expected to be resolved in Humble's favour.

### Rejected alternatives

- **A shared package both branches depend on.** Clean in the abstract, and it
  turns one repository into two with a version constraint between them. The
  neutral core changes on nearly every task in this project, so the version
  constraint would be edited on nearly every task.
- **A single branch with a distro switch.** The vendor trees are different
  upstream branches with different patches — the divergence is in vendored C++
  and launch files, not in anything a runtime switch can reach.
- **Copying the core and letting the branches drift.** Every fix made twice, and
  the second one eventually forgotten. The gravity-scale finding and the
  velocity-limit defect in the excitation would both have needed porting by
  hand.
- **Porting the adapter first.** It is the part that cannot be tested without a
  Humble machine, so it would block everything behind it. The neutral core can
  be verified anywhere.

## Tasks

### Task 1 — read the distro instead of naming it

`profile.ros[distro]` with the distro from `.rosdistro`, replacing six literals
in `cli.py`. A test asserting no module under `src/` names a ROS distro, with
`ros_adapter.py` declared as the exception.

**Done when** the literals are gone, the test names any that come back, and the
Jazzy suite passes unchanged — this task changes no behaviour on Jazzy at all,
which is what makes it safe to do first.

### Task 2 — name the distro-specific ROS surface

Pull the message types, action types and topic names the adapter depends on into
one declared block, so the Humble variant is a small readable diff rather than
edits scattered through 900 lines. Replace the misleading `ImportError` message
with one that names the interface that was actually missing.

**Done when** a missing `ParallelGripperCommand` reports itself by name rather
than as "source a ROS 2 Jazzy workspace", and the block lists every ROS
interface the adapter needs in one place.

### Task 3 — bring the neutral core to Humble

Merge `profile`, `interface`, `safety`, `track`, `identification`, `artifacts`,
`calibration` and `kinematics` with their tests. No behaviour change, no
adapter, no scripts.

**Done when** the merged modules are byte-identical across the two branches, the
suite passes on Humble's Python 3.10, and `robotctl r2s` runs offline there —
`preflight`, `identify`, `fit`, `bundle`, `validate` and `export` need no ROS at
all, so the entire Real2Sim pipeline except collection is verifiable before any
adapter exists.

### Task 4 — the adapter and the workspace scripts

`ros_adapter.py`, `srdf.py`, `pose_bringup.sh`, `load_effort_controllers.sh`,
`install_dependencies_humble.sh`, and the smoke harness, against a real Humble
install.

**Done when** `pose show` reports real joint values, `pose ee --from-marker`
reaches a dragged pose, and `pose gravity --sweep` measures a non-trivial
tracking error — that last one is what proves the state-topic question in §2 was
answered rather than guessed.

### Task 5 — the Humble vendor snapshot

Regenerate the DG5F Gazebo patch against Humble's plugin, re-import the two
components whose upstream branches differ, and recompute every
`post_patch_sha256` against the Humble archives.

**Done when** `verify_vendor_snapshot.py` reports verified for all four
components on Humble, and the DG5F Gazebo smoke test starts with a working
`/clock` — the defect that cost a day on Jazzy, in its Humble form.

### Task 6 — documentation and verification record

Humble's `README.md` gets the operator walkthrough in Korean, `docs/cli.md` the
reference, and `docs/humble-verification.md` what was measured on Humble
hardware. Cross-link the two branches' verification documents so a reader on
either knows the other exists.

**Done when** the documentation tests pass on Humble, and any behaviour that
genuinely differs between the branches is stated as differing rather than
silently documented twice.

## Risks

- **The branches drift anyway.** The largest risk and the least dramatic.
  Mitigated by merging rather than copying and by the distro-name test, but
  neither is proof; the real defence is that Task 1 and Task 2 shrink the
  legitimately-divergent surface to something a person can hold in their head.
- **Nothing here can be tested on this machine.** Only Jazzy is installed. Tasks
  1 to 3 are verifiable anywhere; Tasks 4 and 5 need a Humble machine and should
  not be reported as done from a Jazzy one.
- **The Humble vendor trees are different code.** Findings do not carry.
  `DEFAULT_KP`, the missing acceleration limits that break RViz's Plan &
  Execute, and the `/clock` bridge defect are all facts about specific files on
  specific branches and have to be re-established.
- **The gripper may need a different action entirely.** If Humble has no
  `ParallelGripperCommand`, that is a profile-level difference —
  `action: parallel_gripper_command` against a `GripperCommand` controller — and
  the profile is shared. It may need a per-distro action name, which is the one
  place the neutral core might legitimately have to learn about distros.
- **Isaac Sim is coming.** The simulator work will want the same neutral core.
  Doing this port properly is what makes that a third consumer rather than a
  third copy.
