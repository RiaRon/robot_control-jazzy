# `robotctl` command reference

Every command this branch ships, with its arguments, defaults, exit codes, and
the safety default in force. Output shown below is real, captured on Ubuntu
24.04 with ROS 2 Jazzy against the bimanual stack on fake hardware.

Two rules hold everywhere:

- **Nothing is published without `--execute`.** Every command that can move the
  robot defaults to a dry run that prints what it would send.
- **Profile limits are authoritative.** When the profile
  (`src/robot_control/profiles/openarm_tesollo.yaml`) and a vendor URDF or SRDF
  disagree, the profile wins and the command is refused.

## Setting a pose interactively

The common task: bring the arms up, find a pose by dragging the end-effector
marker in RViz, then commit it.

**1. Bring up the stack.** Fake hardware, nothing touches CAN:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/pose_bringup.sh
```

**2. Drag the marker.** In RViz open the **MotionPlanning** panel and drag the
interactive marker on `openarm_right_hand_tcp`, the tool centre point. This
moves MoveIt's *goal state* only. The robot does not follow the marker;
nothing reaches the controllers until you run a command with `--execute`.

**3. Read the pose back.** In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"

robotctl pose show --group openarm_right_arm
```

> Append to `PYTHONPATH`, never assign it. `PYTHONPATH=src:. robotctl ...`
> replaces the paths the ROS setup files just added, and the adapter then
> reports `rclpy` as missing on a machine where it is installed.

**4. Commit it.** Check the plan first, then send it:

```bash
robotctl pose ee --group openarm_right_arm --relative --xyz 0,0,0.03
# --execute publishes to the controller and the arm moves:
robotctl pose ee --group openarm_right_arm --relative --xyz 0,0,0.03 --execute
```

**5. Go home.**

```bash
# --execute sends the trajectory; the arm moves back to the SRDF home state:
robotctl pose joints --group openarm_right_arm --named home --execute
```

> **RViz's own Plan & Execute buttons do not work on this branch yet.** The
> vendored `joint_limits.yaml` declares `has_acceleration_limits: false` for
> all 17 joints, and Jazzy's default planning pipeline runs
> `AddTimeOptimalParameterization`, which requires acceleration limits. Every
> plan therefore fails inside `move_group` with
> `PlanningResponseAdapter 'AddTimeOptimalParameterization' failed with error
> code FAILURE`, before anything reaches a controller. This is a configuration
> gap, not a hardware one: it fails identically on fake hardware, in Gazebo,
> and on the real robot. Use the marker to *find* a pose and `robotctl pose ee`
> to *reach* it — `robotctl pose ee` calls `/compute_ik` and sends the
> trajectory straight to the controller, so it never enters that pipeline.

## Running against the real robot

Everything above runs on fake hardware. Driving the physical arms takes two
steps the fake path does not need: the CAN buses have to be configured and
brought up first, and the bringup has to be told which buses to open.

### 1. Bring the CAN buses up

One bimanual OpenArm is one bus per arm. List what the machine actually has
before configuring anything:

```bash
ip -br link show type can
```

A single robot on a dual-channel adapter enumerates as `can0` and `can1`, both
reported against the same `parentdev`. Those are the two buses to configure.
Substitute whatever names your own output reports; a rig with more than one
robot attached will show four or more.

The interfaces must be configured in **CAN FD** mode at **1 Mbit/s arbitration
and 5 Mbit/s data**. `openarm_description` renders its `ros2_control` block with
`can_fd:=true` and `demo.launch.py` does not expose an argument to change it, so
a bus brought up in CAN 2.0 mode links but never exchanges a frame with the
motors:

```bash
# Repeat for each bus this run will open. Configuring requires the link down.
for iface in can0 can1; do
  sudo ip link set "$iface" down
  sudo ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on
  sudo ip link set "$iface" up
done
```

Verify before launching anything — `state UP` and `fd on` must both appear:

```bash
ip -details link show can0 | grep -E "state|fd on"
```

The vendored `ros_ws/src/openarm_can/setup/openarm-can-configure-socketcan-4-arms`
does the same thing, but it is written for a four-bus rig: it aborts unless
`can0` through `can3` all exist, so it is no use with a single robot attached.
With four buses present it still needs `-fd` passed explicitly:

```bash
./ros_ws/src/openarm_can/setup/openarm-can-configure-socketcan-4-arms -fd
```

### 2. Launch against those buses

```bash
source /opt/ros/jazzy/setup.bash
# --real opens the named CAN buses and the physical arms will move:
./ros_ws/pose_bringup.sh --real --right-can can0 --left-can can1
```

Both buses must be named; the wrapper refuses to guess, and the names are
positional to the arm, not to the number. Nothing downstream can catch a swap:
`openarm_hardware` addresses both arms with the same motor IDs — `0x01`–`0x07`
sending, `0x11`–`0x17` receiving, `0x08`/`0x18` for the gripper — so a mirrored
pair of buses reports plausible joint values either way, and the first symptom
is the left arm answering a command addressed to the right.

The pose commands themselves are identical to the fake-hardware ones — same
groups, same options, same gate. Nothing in `robotctl` changes between fake and
real hardware; only what is on the other end of the controller does.

### Before the first real run

1. Keep the E-stop within reach and the workspace clear.
2. Read the pose before commanding one. `robotctl pose show` tells you where
   the arm actually is; `--named home` sends every joint to zero, which from an
   unknown pose can be a large move.
3. Check the plan with a dry run first. Every `pose` command prints exactly
   what it would send before you add `--execute`.
4. Prefer small `--relative` steps over absolute `--xyz` targets until you
   trust the frame.

The profile's velocity limits apply to real hardware exactly as they do to fake
hardware, and `--duration` sets how long a move is allowed to take: a short
duration on a large move is refused rather than executed quickly.

## Groups

Commands address an actuator *group* from the profile, not individual joints.

| Group | Joints | Controller | Planning group | Reachable by |
| --- | ---: | --- | --- | --- |
| `openarm_right_arm` | 7 | `right_joint_trajectory_controller` | `right_arm` | joints, named state, EE pose |
| `openarm_left_arm` | 7 | `left_joint_trajectory_controller` | `left_arm` | joints, named state, EE pose |
| `openarm_left_gripper` | 1 | `left_gripper_controller` | `left_gripper` | joints, named state |
| `tesollo_abduction` | 5 | `joint_trajectory_controller` | — | joints |
| `tesollo_curl` | 5 | `joint_trajectory_controller` | — | joints |
| `tesollo_pip` | 5 | `joint_trajectory_controller` | — | joints |
| `tesollo_dip` | 5 | `joint_trajectory_controller` | — | joints |

Groups with no planning group have no IK solver configured, so they have no
end-effector pose and `robotctl pose ee` refuses them.

---

# `robotctl pose`

Read and set robot poses. Operator commands pass through the same canonical
interface and the same safety gate as commands issued by a learned policy, so
pose setting adds no second route to the hardware.

## `robotctl pose show`

Report the current joint values of each group and, for a group with a planning
group, its end-effector pose from `/compute_fk`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | all executable groups | Restrict the report to one group |

Reading state is not publishing, so `show` needs no `--execute`. It does need a
running stack.

```bash
robotctl pose show --group openarm_right_arm
```

```text
openarm_right_arm: controller=right_joint_trajectory_controller planning_group=right_arm
openarm_right_arm: +0.0000 +0.0000 +0.0000 +0.0000 +0.0000 +0.0000 +0.0000
openarm_right_arm: openarm_right_hand_tcp xyz [+0.0000 -0.1535 +0.0819] rpy [-3.1416 -0.0000 +0.0000]
```

The first line is static profile data and is printed even with no robot
running; the command then exits `2` if it cannot read live state.

**Exit codes:** `0` read; `2` no ROS, no running stack, or unknown group.

## `robotctl pose joints`

Set a group directly, from explicit canonical values or from an SRDF named
state.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | *required* | Group to move |
| `--values` | — | Comma-separated canonical values, one per group joint |
| `--named` | — | An SRDF group state, such as `home` or `hands_up` |
| `--duration` | `3.0` | Seconds allowed for the move |
| `--execute` | off | Publish; without it the command only prints |

`--values` and `--named` are mutually exclusive, and one is required.

A dry run resolves **entirely offline**, including the named state, so it
cannot reach the robot even if a stack is running:

```bash
robotctl pose joints --group openarm_left_gripper --values 0.02
```

```text
DRY RUN: would send over 3 s; pass --execute to send
group: openarm_left_gripper -> controller left_gripper_controller (parallel_gripper_command)
  canonical        source joint                 commanded (rad)
  l_hj_gripper_1   openarm_left_finger_joint1           +0.0200
```

The `commanded` column is what goes on the wire, in the robot's own joint names
and with the profile's sign applied.

```bash
# --execute sends the goal to the controller and the joints move:
robotctl pose joints --group openarm_right_arm --named home --execute
```

Named states come from the vendored SRDF, which is why they work offline. They
are vendor data and are **not** exempt from the profile:

```bash
robotctl pose joints --group openarm_left_gripper --named open
```

```text
refused: position limit exceeded at waypoint 0
```

The SRDF opens that gripper to 0.044 rad; the profile stops it at 0.04. The
profile is the contract shared with `sim2real`, so it wins.

**Exit codes:** `0` sent or printed; `2` unknown group, wrong number of values,
non-numeric value, unknown named state, or no adapter; `3` refused by the
safety gate.

## `robotctl pose ee`

Move a group so its end-effector reaches a pose, solving IK through
`/compute_ik`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | *required* | Group to move; must have a planning group |
| `--xyz` | *required* | `x,y,z` in metres |
| `--rpy` | keep current | `roll,pitch,yaw` in radians |
| `--relative` | off | Treat `--xyz` and `--rpy` as an offset from the current pose |
| `--duration` | `3.0` | Seconds allowed for the move |
| `--execute` | off | Publish; without it the command only prints |

`--relative` is the usual form when nudging a setup. IK is a `move_group`
service, so unlike `pose joints` there is **no offline form**: even a dry run
needs a running stack. The dry run solves and prints without sending.

```bash
robotctl pose ee --group openarm_right_arm --relative --xyz 0,0,0.03
```

```text
openarm_right_hand_tcp: [+0.0000 -0.1535 +0.0819] -> [+0.0000 -0.1535 +0.1119] in world
group: openarm_right_arm -> controller right_joint_trajectory_controller (follow_joint_trajectory)
  canonical        source joint                 commanded (rad)
  r_aj_1           openarm_right_joint1                 -0.3692
  r_aj_2           openarm_right_joint2                 +0.0159
  r_aj_3           openarm_right_joint3                 -0.0410
  r_aj_4           openarm_right_joint4                 +0.7463
  r_aj_5           openarm_right_joint5                 +0.0409
  r_aj_6           openarm_right_joint6                 -0.0162
  r_aj_7           openarm_right_joint7                 -0.3764
DRY RUN: solved but not sent; pass --execute to send
```

```bash
# --execute publishes the trajectory and the arm moves to the solved pose:
robotctl pose ee --group openarm_right_arm --relative --xyz 0,0,0.03 --execute
```

IK runs with collision avoidance on. The solution is then checked against the
profile limits as a whole trajectory: if any waypoint is out of bounds the
entire motion is discarded rather than truncated.

**Exit codes:** `0` solved and sent or printed; `2` unknown group, group with
no planning group, malformed `--xyz` or `--rpy`, or no adapter; `3` no IK
solution, or refused by the safety gate.

## `robotctl pose rviz`

Launch the bimanual MoveIt stack with RViz through `ros_ws/pose_bringup.sh`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--real` | off | Drive real hardware over CAN instead of fake hardware |
| `--right-can` | — | CAN interface for the right arm, required with `--real` |
| `--left-can` | — | CAN interface for the left arm, required with `--real` |

```bash
robotctl pose rviz
```

This inverts the vendor default. `demo.launch.py` defaults `use_fake_hardware`
to false and opens `can0` and `can1`; here fake hardware is what you get by
omission. A fake run passes a CAN interface name no device can have, so
anything that did try to open a bus fails loudly instead of finding a real one.

```bash
# --real drives the physical robot; both buses must be named explicitly:
robotctl pose rviz --real --right-can can0 --left-can can1
```

`--real` without both interfaces is refused rather than guessed. The buses must
already be up in CAN FD mode; see [Running against the real
robot](#running-against-the-real-robot).

**Exit codes:** `0` the launch exited cleanly; `2` bringup wrapper missing,
non-Jazzy environment, unbuilt workspace, or `--real` without both buses;
otherwise the launch's own status.

---

# `robotctl r2s`

The Real2Sim identification pipeline: measure the real robot so simulation can
be corrected against it. The stages run in order, each consuming the previous
stage's artifact.

## `robotctl r2s preflight`

Report the loaded profile and confirm that publishing is off.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |

```bash
robotctl r2s preflight
```

```text
profile: openarm_tesollo
asset: openarm_tesollo_sensor_rl
joints: 35
publish_enabled: false
```

**Exit codes:** `0` always, unless the profile or its asset manifest fails to
load.

## `robotctl r2s collect`

Build the excitation trajectory used for identification and report its shape.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--dry-run` | on | Print the plan without publishing |
| `--execute` | off | Publish the excitation; needs a ROS adapter |
| `--amplitude-scale` | `0.3` | Fraction of the per-joint range to excite, in `(0, 1]` |

```bash
robotctl r2s collect
```

```text
DRY RUN: profile=openarm_tesollo amplitude_scale=0.3 samples=600 phases=hold,step,ramp,multisine
```

Amplitudes are 5 % of each joint's range scaled by `--amplitude-scale`, so the
default excites 1.5 % of range.

```bash
# --execute would publish the excitation to the real command topic:
robotctl r2s collect --execute
```

**Exit codes:** `0` planned; `2` `--execute` without a publisher backend;
`SystemExit` for `--amplitude-scale` outside `(0, 1]`.

## `robotctl r2s normalize`

Resample a recorded run onto the profile's command rate and write an HDF5
track.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--input` | *required* | `.npz` with `command_time_ns`, `command`, `measured_time_ns`, `measured`, `joint_names` |
| `--output` | *required* | HDF5 track to write |

```bash
robotctl r2s normalize --input run.npz --output track.h5
```

```text
normalize: track.h5 sha256=6f1c…
```

The printed digest identifies the track and is carried into the fit result.

**Exit codes:** `0` written; `SystemExit` if either path is missing.

## `robotctl r2s fit`

Fit a second-order joint model to a normalized track.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--track` | *required* | HDF5 track from `normalize` |
| `--output` | *required* | JSON estimate to write |
| `--population` | `128` | Candidate population size; must be positive |

```bash
robotctl r2s fit --track track.h5 --output estimate.json
```

The estimate records stiffness, damping, friction, and per-joint residual RMSE,
along with the source track's SHA-256.

**Exit codes:** `0` written; `SystemExit` for a missing path or a
non-positive `--population`.

## `robotctl r2s validate`

Check holdout metrics against the acceptance thresholds.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--bundle` | *required* | Calibration bundle to validate against |
| `--metrics` | *required* | JSON holdout metrics |
| `--output` | *required* | JSON verdict to write |

```bash
robotctl r2s validate --bundle bundle.json --metrics holdout.json --output verdict.json
```

```text
validate: schema v2, status=validated
```

**Exit codes:** `0` validated; `3` the model is inadequate; `SystemExit` for a
missing path.

## `robotctl r2s export`

Export a validated bundle. Refuses anything not already validated.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--bundle` | *required* | Calibration bundle to export |
| `--validation` | *required* | Verdict JSON from `validate` |
| `--output` | *required* | Destination bundle |

```bash
robotctl r2s export --bundle bundle.json --validation verdict.json --output release.json
```

**Exit codes:** `0` exported; `3` the verdict is not `validated`; `SystemExit`
for a missing path.

---

# Exit codes

| Code | Meaning |
| --- | --- |
| `0` | The command succeeded. A dry run that printed its plan succeeded. |
| `2` | The request or the environment cannot support the command: no ROS adapter, no running stack, an unknown group or named state, or a malformed argument. |
| `3` | The command was understood and refused: a safety-gate rejection, a failed IK solve, or an inadequate model. |

Exit `2` means *cannot*; exit `3` means *will not*. A `3` is the system working.

# Troubleshooting

**`unavailable: the ROS adapter needs rclpy and the MoveIt message packages`**
Either no ROS 2 Jazzy workspace is sourced, or `PYTHONPATH` was assigned rather
than appended and the assignment dropped the ROS paths. Use
`export PYTHONPATH="src:.:$PYTHONPATH"`.

**`unavailable: no /joint_states within 10.0 s`**
Nothing is publishing joint states. Start the stack with
`./ros_ws/pose_bringup.sh` and wait for the controllers to activate; check with
`ros2 control list_controllers`.

**`unavailable: /compute_ik is not available; is move_group running?`**
The controllers came up but `move_group` did not. Look for a MoveIt
configuration error in the bringup log.

**`refused: no IK solution for openarm_right_hand_tcp`**
The pose is outside the arm's reach, or the only solutions collide. Try a
smaller `--relative` step, or move to an intermediate pose first.

**`refused: position limit exceeded at waypoint N`**
The target violates the profile's joint limits. This is not overridable from
the command line by design; if the limit itself is wrong, change the profile,
which also changes the contract `sim2real` sees.

**`refused: velocity limit exceeded at waypoint N`**
The move is too large for `--duration`. Raise `--duration` rather than the
profile's velocity limit.

**A validator reports another robot's joints as `extra joints`**
A launch from an earlier run is still publishing `/joint_states`. Run the smoke
tests from a clean process table:

```bash
for pattern in "[m]ove_group" "[r]os2_control_node" "[r]obot_state_publisher" \
               "[b]in/ros2 launch" "[g]z sim" "[p]arameter_bridge"; do
    pkill -f "$pattern" || true
done
```

The bracket around the first character stops the pattern from matching the
`pkill` command line itself, which otherwise kills the shell running it.

**`error: source a ROS 2 Jazzy environment`**
`ROS_DISTRO` is not `jazzy`. This branch is Jazzy-only; Humble lives on a
separate long-lived branch.

**A `--real` bringup starts but no joint states ever arrive**
The buses are configured but not talking. Check, in this order:

```bash
ip -details link show can0 | grep -E "state|fd on"   # state UP and fd on
candump can0                                         # frames while the arm is powered
```

`state STOPPED` means the link was never brought up. `state UP` with no frames
in `candump` means either the arm is unpowered, the bus is the wrong one, or it
was configured in CAN 2.0 mode while the description asks for CAN FD. `state
BUS-OFF` means a bitrate mismatch or missing termination — reconfigure at
1 Mbit/s / 5 Mbit/s FD rather than restarting the launch.

**The wrong arm moves**
`--right-can` and `--left-can` were swapped. The names are positional to the
arm, and nothing downstream can detect the mistake: each bus reports plausible
joint values either way.

**RViz reports `Fail: ABORTED: No motion plan found` or the Plan button does
nothing**
See the note at the top of this document: the vendored MoveIt configuration
declares no acceleration limits, so every plan fails in
`AddTimeOptimalParameterization`. Nothing reaches the robot when this happens.
Use `robotctl pose ee`, which does not go through the planning pipeline.

**The RViz marker and `pose show` disagree about where the end effector is**
They should not: both are `openarm_*_hand_tcp`, the tool centre point. If they
differ, the SRDF has drifted; `pytest tests/test_profile.py` pins the contract
and will say so.
