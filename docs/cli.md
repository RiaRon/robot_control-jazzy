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

## `robotctl pose ready` and `robotctl pose rest`

`ready`의 기본값은 오른팔 deterministic 표준 A′ v2:
`openarm_right_ready_v2 = [0,0.2,0,0.6,0,0,0] rad`이다. 비교용 D는
`--posture openarm_right_ready_v1`로만 명시 선택한다. IK를 사용하지 않고 J4를
먼저 올린 뒤 minimum-jerk joint-space reference를 controller rate로 보내며 속도와
가속도는 각각 0.10 rad/s, 0.10 rad/s² 이하이다. 기본은 완전한 offline dry-run이고
왼팔 ready는 비활성 상태다.

`--execute` 전에 trajectory controller와 effort controller가 모두 active이고 서로
분리된 position/effort interface를 claim하는지 검사한다. 하나라도 불일치하면 첫
position publish 전에 거부한다. 이동과 settle 동안 매 cycle의 현재 관절값으로
runtime URDF 중력 토크(scale 1.0)를 다시 계산해 effort controller에 발행한다.
성공·실패·예외 모두 마지막에 zero effort를 발행한다. settle 실패는 먼 target을
유지하지 않고 마지막 measured pose를 joint limit 안에서 position hold한다.

```bash
robotctl pose ready --group openarm_right_arm
robotctl pose ready --group openarm_right_arm --posture openarm_right_ready_v1

# --execute publishes commands and moves the real right arm; effort controller must be active.
robotctl pose ready --group openarm_right_arm \
  --before-output /data/right-before-ready-v2.json \
  --after-output /data/right-after-ready-v2.json --execute
robotctl pose rest --group openarm_right_arm --execute
```

before/after JSON은 원자 저장된다. 실패와 예외도 가능한 최신 feedback으로 partial
after JSON을 남기며 target, controller reference, feedback, 관절별 error, gravity
scale/torque, settle 시간·종료 이유, safe hold와 zero cleanup 결과를 기록한다.
허용 오차는 관절별 0.020 rad다. GenericSystem은 사실상 0 오차이고 과거 중력보상
실물 baseline worst 0.0063 rad의 3배 이상이면서 기존 IK 0.30 rad hard boundary보다
충분히 작다. 후보 근거는
[`ready-posture-evaluation-2026-08-20.md`](ready-posture-evaluation-2026-08-20.md)에
있다.
## `robotctl pose torque`

Run the direct-torque staircase used to identify stiffness, Coulomb friction,
and bias. `--group` is required; repeat `--joint` to restrict the experiment,
otherwise every joint in the group is measured. `--deflection` sets the target
angular displacement and `--steps` sets the staircase resolution. `--noise`
must describe the encoder's measured noise because the seed torque is doubled
until motion clears five times that value. `--hold-sec`, `--urdf`, and
`--output` control settling, the gravity model, and the signed sweep artifact.
Without `--execute`, the command reviews the experiment and publishes nothing.

```bash
robotctl pose torque --group openarm_right_arm --joint r_aj_2 \
  --deflection 0.05 --steps 7 --noise 0.0002
```

## `robotctl pose show`

Report the current joint values of each group and, for a group with a planning
group, its end-effector pose from `/compute_fk`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | all executable groups | Restrict the report to one group |
| `--output` | — | Atomically write the measured joints and TCP pose as JSON |

Reading state is not publishing, so `show` needs no `--execute`. It does need a
running stack.

```bash
robotctl pose show --group openarm_right_arm
```

To keep a small pose snapshot for transfer to another computer, add an output
path:

```bash
robotctl pose show --group openarm_right_arm --output right-pose.json
```

The JSON records the profile and every selected group, including canonical
joint names and positions in radians. Groups with a MoveIt planning group also
record the TCP frame and tip link, XYZ in metres, XYZW quaternion, and RPY in
radians. The normal screen report is unchanged. The file is replaced
atomically only after every requested ROS read succeeds, so a failed read does
not leave a partial snapshot or overwrite an earlier one.

```text
openarm_right_arm: controller=right_joint_trajectory_controller planning_group=right_arm
openarm_right_arm: +0.0000 +0.0000 +0.0000 +0.0000 +0.0000 +0.0000 +0.0000
openarm_right_arm: openarm_right_hand_tcp xyz [+0.0000 -0.1535 +0.0819] rpy [-3.1416 -0.0000 +0.0000]
```

The first line is static profile data and is printed even with no robot
running; the command then exits `2` if it cannot read live state.

**Exit codes:** `0` read (and, when requested, written); `2` no ROS, no running
stack, unknown group, or output error.

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
| `--xyz` | *one of these two* | `x,y,z` in metres |
| `--from-marker` | *one of these two* | Take the target from the RViz marker you dragged |
| `--rpy` | keep current | `roll,pitch,yaw` in radians |
| `--relative` | off | Treat `--xyz` and `--rpy` as an offset from the current pose |
| `--duration` | `3.0` | Seconds allowed for the move |
| `--execute` | off | Publish; without it the command only prints |
| `--settle` | off | Re-command until the residual falls below `--tolerance` |
| `--tolerance` | `0.005` | Metres of residual `--settle` aims for |

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

### Reaching the pose you dragged in RViz

`--from-marker` reads the goal marker straight out of RViz, which is what makes
dragging usable given that RViz's own Execute button cannot drive this robot.
Drag the marker to where you want the tool centre point, then:

```bash
robotctl pose ee --group openarm_right_arm --from-marker
# --execute publishes the trajectory and the arm moves to the dragged pose:
robotctl pose ee --group openarm_right_arm --from-marker --execute
```

The pose comes from `get_interactive_markers` on the MotionPlanning display's
own marker server, asking for the marker MoveIt names after this group's tip
link — `EE:goal_openarm_right_hand_tcp`. Querying the server rather than
listening to the marker's `feedback` topic matters: feedback is published only
while a drag is in progress, so a listener would be racing the operator's
mouse, while the server holds the pose after the drag ends.

The marker is an absolute pose in `world`, so `--from-marker` refuses `--rpy`
and `--relative`; modifying it would move the arm somewhere that was never on
screen.

RViz publishes a marker only for the planning group the panel is currently set
to. Asking for a group it is not showing reports which marker was missing:

```text
unavailable: RViz is running but holds no marker named
'EE:goal_openarm_right_hand_tcp'; set the MotionPlanning panel's planning group
to 'right_arm' so it publishes one
```

### The arm stops short, and `--settle` closes the gap

Every `--execute` prints how far the tool centre point ended up from where it
was sent:

```text
EXECUTED: openarm_right_arm over 3 s
residual: 58.4 mm from the commanded pose
```

That residual is not an IK error — the solution's own FK lands on the target to
within a rounding digit. It is the arms holding position through the DM motors'
impedance control with no gravity feedforward. The controller commands position
only, and the motor's torque is `kp * (commanded - actual)`, so a joint can only
hold against gravity *while it sits short of its command*. The steady-state
error is therefore roughly the holding torque divided by `kp`, which is why the
shoulder and elbow miss by ~0.07 rad while the low-load wrist joints miss by
~0.004.

Re-sending the same solution changes nothing: it reproduces the same shortfall
exactly. `--settle` instead adds each pass's measured shortfall to the command,
so the arm is asked to go past the target by what it last missed:

```bash
# --execute publishes; --settle keeps correcting until it lands:
robotctl pose ee --group openarm_right_arm --from-marker --execute --settle
```

```text
EXECUTED: openarm_right_arm over 3 s
settle 1: 58.4 -> 6.1 mm
settle 2: 6.1 -> 1.4 mm
settled: 1.4 mm after 2 corrections
```

Each pass is authorised by a fresh safety gate, so a wound-up command is checked
against the profile's position limits rather than trusted for having been safe
once. The loop gives up after four corrections, and stops early if a pass fails
to improve the residual by at least a tenth — an arm against a hard stop or
holding a load will not converge, and further passes would only wind the command
further past a pose it cannot reach.

`--settle` needs `--execute`: a dry run sends nothing to fall short of.

**Exit codes:** `0` solved and sent or printed; `2` unknown group, group with
no planning group, malformed `--xyz` or `--rpy`, `--from-marker` combined with
`--rpy` or `--relative`, `--settle` without `--execute`, no adapter, or no
marker to read; `3` no IK solution, or refused by the safety gate.

## `robotctl pose gravity`

Publish gravity feedforward torque, and measure the scale that works.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | *required* | Group to compensate; must declare an `effort_controller` |
| `--scale` | `1.0` | Fraction of the modelled torque: one value, or one per joint |
| `--sweep` | — | Comma-separated scales to measure in turn |
| `--sweep-joint` | all joints | Canonical joint whose scale `--sweep` varies, holding the rest at `--scale` |
| `--output` | — | Write what the run measured, for `r2s identify` to fit |
| `--hold-sec` | `2.0` | Seconds to publish at each scale before measuring |
| `--execute` | off | Publish; without it the torque is only computed and printed |

The effort controllers are not part of the vendor bringup, so load them first:

```bash
./ros_ws/pose_bringup.sh --real --right-can can0 --left-can can1   # terminal 1
./ros_ws/load_effort_controllers.sh                                # terminal 2
```

They are additive — the trajectory controllers keep the position interfaces, and
`ros2 control list_hardware_components` shows `position` and `effort` both
claimed on every arm joint.

### Why a scale, and why it is measured

The torque comes from the URDF's masses and centres of mass, and it works against
gains that are **hard-coded in the vendor hardware**, not configured:
`control_gains.yaml` declares `kp` 70/70/70/60/10/10/10, but
`v10_simple_hardware.cpp` reads only `can_interface`, `arm_prefix`, `hand`, and
`can_fd`, and `write()` applies `DEFAULT_KP` = 20/20/20/20/5/5/5. Editing that
YAML changes nothing.

So the right scale is a measured quantity. `--sweep` measures it: hold at each
scale, read the trajectory controller's own `error.positions` — the droop
directly, with no IK or FK in the way — and print what happened.

```bash
# --execute publishes torque at each scale in turn, and the arm shifts as the
# compensation changes. Keep the E-stop within reach.
robotctl pose gravity --group openarm_right_arm --execute --sweep 0,0.25,0.5,0.75,1.0
```

```text
  scale      r_aj_1    r_aj_2    r_aj_3    r_aj_4    r_aj_5    r_aj_6    r_aj_7
   0.00     +0.0694   -0.0225   -0.0137   +0.1790   +0.0112   +0.0193   -0.0705
   ...
best measured scale: 0.75 (worst joint +0.0121 rad)
```

### Refining one joint at a time

A single number can only reach a compromise, because each joint's optimum is
different: measured on the real right arm, extrapolating each joint's error to
zero puts `r_aj_2` at about 1.14, `r_aj_1` at 1.23 and `r_aj_3` at 1.31. The
modelled torque's *distribution* is off, not only its magnitude.

So find the global optimum first, then refine each joint against it.
`--sweep-joint` varies one joint's scale and holds the rest at `--scale`:

```bash
# --execute publishes torque at each scale; only r_aj_2's share changes.
robotctl pose gravity --group openarm_right_arm --execute --scale 1.1 --sweep-joint r_aj_2 --sweep 1.05,1.1,1.15,1.2
```

The report scores that joint's own error rather than the worst across the arm —
a global worst would be dominated by joints this sweep never touched — and
prints the full vector to carry into the next round:

```text
best measured scale for r_aj_2: 1.15 (that joint +0.0031 rad)
  refine the next joint the same way, then hold them all at once:
  --scale 1.1,1.15,1.1,1.1,1.1,1.1,1.1
```

Then hold there:

```bash
# --execute publishes torque; the arm holds itself up until you stop the command:
robotctl pose gravity --group openarm_right_arm --scale 1.1,1.15,1.1,1.1,1.1,1.1,1.1 --execute
```

`pose follow --gravity` takes the same one-or-per-joint form, so a tuned vector
carries straight over.

Scales above `1.5` are refused. Over-compensating does not mispose the arm, it
drives it away from the pose it was holding. The torque is also checked against
the profile's per-joint `effort` bound and refused if it exceeds it — unlike a
servo step, torque is clamped nowhere, because a force reduced to "as much as
allowed" is still a force in the wrong amount.

Whatever happens, the command publishes zeros before it exits. Torque left
applied after the process is gone would keep pushing.

### Keeping the measurement: `--output`

A sweep is already an identification experiment — a known torque published, a
standing error measured — and `--output` keeps it instead of leaving it in the
terminal:

```bash
# --execute publishes torque at each scale, and the run is recorded to disk:
robotctl pose gravity --group openarm_right_arm --execute --sweep 0,0.5,1.0 --output sweeps/pose0.json
```

Each round in the file carries **its own** pose and its own modelled torque, not
one for the whole sweep. Compensation moves the arm, so the load a joint holds at
scale 1.0 is not the load it held at 0.0.

The file records the profile, the asset id and the asset manifest hash, and
`r2s identify` refuses it against anything else. `--output` needs `--execute`: a
dry run publishes nothing, so there would be nothing measured to record.

**Exit codes:** `0` published or printed; `2` unknown group, group with no
`effort_controller` or no `tip_link`, a scale outside range, `--output` without
`--execute`, or no adapter; `3` torque over the profile's effort limit.

## `robotctl pose follow`

Track the dragged RViz marker continuously, instead of committing one pose at a
time.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | *required* | Group to move; must have a planning group |
| `--gravity` | off | Gravity feedforward scale: one value, or one per joint |
| `--seconds` | `60.0` | How long to follow before stopping on its own |
| `--kp` | `2.0` | Measured-joint outer feedback gain |
| `--ki` | `1.0` | Accepted for compatibility; command accumulation already supplies integral action |
| `--tolerance` | `0.002` | TCP position deadband in metres |
| `--orientation-tolerance` | `0.035` | TCP orientation deadband in radians, approximately 2 degrees |
| `--max-tcp-speed` | `0.05` | Maximum TCP linear speed in metres per second |
| `--max-tcp-angular-speed` | `0.20` | Maximum TCP angular speed in radians per second |
| `--max-ik-step` | `0.02` | Maximum Cartesian distance from the measured TCP to each intermediate IK target |
| `--max-ik-angular-step` | `0.10` | Maximum angular distance from the measured TCP to each intermediate IK target |
| `--startup-settle-sec` | `2.0` | Time to apply gravity compensation before accepting startup alignment |
| `--max-start-distance` | `0.10` | Maximum allowed position distance between the actual TCP and RViz marker during startup alignment |
| `--max-start-angle` | `0.35` | Maximum allowed orientation difference during startup alignment, approximately 20 degrees |
| `--output` | off | Write a JSON trace separating live marker, accepted IK request, IK target, active command, and measured state |
| `--diagnostic-profile` | off | Deterministic `translation`, `rotation`, or `translation-rotation` round trip after startup alignment |
| `--diagnostic-distance` | `0.010` | Translation distance; hard maximum 0.030 m |
| `--diagnostic-angle` | 5 degrees | Rotation angle in radians; hard maximum 10 degrees |
| `--diagnostic-linear-speed` | `0.005` | Target translation speed; hard maximum 0.020 m/s and cannot exceed `--max-tcp-speed` |
| `--diagnostic-angular-speed` | `0.05` | Target rotation speed; hard maximum 0.10 rad/s and cannot exceed `--max-tcp-angular-speed` |
| `--diagnostic-hold-sec` | `3.0` | Hold time at the displaced target and at the origin; hard maximum 10 s |
| `--diagnostic-repetitions` | `1` | Round trips; hard maximum 3 |
| `--diagnostic-translation-axis` | `x` | World-frame translation axis: x, y, or z |
| `--diagnostic-rotation-axis` | `z` | Startup-TCP local rotation axis: x, y, or z |
| `--ik-jump-threshold` | `0.10` | Record an IK target transition event at this joint delta; diagnostic only, never blocks a command |
| `--execute` | off | Publish; without it the command only reports what it would do |

At startup, `pose follow` first applies gravity compensation and moves the
measured TCP pose towards the RViz marker. Both position and orientation are
aligned. The position target is limited by `--max-ik-step`, while the
orientation follows the shortest quaternion path and is limited by
`--max-ik-angular-step`. Do not drag the marker until the command prints
`startup alignment complete`.

Alignment is accepted only after `--startup-settle-sec` has elapsed, the TCP
position is within `--tolerance`, and its orientation is within
`--orientation-tolerance`. If the initial position distance exceeds
`--max-start-distance`, or the initial orientation difference exceeds
`--max-start-angle`, the command refuses to move and asks the operator to move
the RViz marker to Current before retrying.

After alignment, the actual TCP target matches both the marker's absolute
position and orientation. During following, the command stream is limited by
`--max-tcp-speed` and `--max-tcp-angular-speed`. Translation and rotation share
the stricter scaling factor so that the seven-joint motion keeps the IK path
while satisfying both limits.

```bash
# --execute streams position commands and the arm moves while you drag:
robotctl pose follow \
  --group openarm_right_arm \
  --execute \
  --gravity 0.75 \
  --output /tmp/right-follow.json
```

### Ready posture precondition

A deterministic diagnostic reads the current seven joints before marker setup,
gravity publication, IK, or trajectory publication. It refuses unless every
joint is within 0.020 rad of `openarm_right_ready_v2`. `pose ready` and
`pose follow` remain separate commands; follow never moves to ready
automatically. Manual marker follow is unchanged. Full and pre-start-refused
JSON record the ready name, target, actual start, per-joint error, and pass flag
under `settings.ready_posture` and `result.ready_posture`.

### Deterministic diagnostic profiles

The optional diagnostic target replaces live marker updates only after normal
startup alignment. Each motion ramps away from the startup marker, holds,
returns on the same path, and holds at the origin. The combined profile runs
the translation round trip followed by the rotation round trip. Repetitions do
not accumulate position or angle.

Without `--execute`, the command prints the design and publishes nothing:

```bash
robotctl pose follow \
  --group openarm_right_arm \
  --diagnostic-profile translation \
  --diagnostic-translation-axis x
```

A measured fake or real run additionally needs `--execute` and can write JSON:

```bash
# --execute publishes the profile and moves the selected arm.
robotctl pose follow \
  --group openarm_right_arm \
  --diagnostic-profile translation-rotation \
  --diagnostic-distance 0.01 \
  --diagnostic-angle 0.0872665 \
  --diagnostic-linear-speed 0.005 \
  --diagnostic-angular-speed 0.05 \
  --diagnostic-hold-sec 3 \
  --diagnostic-repetitions 1 \
  --output /tmp/right-follow-profile.json \
  --execute
```

The profile has its own conservative hard bounds in addition to the unchanged
pose-follow Cartesian and joint safety gates. A profile automatically ends
after its final origin hold. Do not drag the marker during a profile.

```text
following openarm_right_hand_tcp at 100 Hz for 60 s, gravity scale 0.75
startup alignment: do not drag until the actual TCP reaches the RViz marker
startup alignment complete; drag the marker in RViz
followed 4193 samples; the arm holds its last commanded pose
  actual control rate: 76.8 Hz; joint-state wait 12.9 ms/sample
  IK requests 91: 72 succeeded, 3 failed, 16 superseded
  tool centre point trailed the marker by 8.4 mm on average, 61.2 mm at worst
  last TCP position error: 1.7 mm
  within 2.0 mm on 1132 of 4193 samples (27.0%)
  live marker to measured TCP: 9.1 mm on average, 63.0 mm at worst, 1.7 mm last
  mean position lag decomposition (norms are non-additive):
    marker_update_staleness: 0.7 mm
    accepted_marker_to_ik_target: 1.8 mm
    ik_target_to_command: 5.2 mm
    command_to_measured: 2.1 mm
  TCP orientation trailed the marker by 3.8 deg on average, 12.4 deg at worst
  last TCP orientation error: 1.2 deg
  within 2.0 deg on 1540 of 4193 samples (36.7%)
  Cartesian speed limit on 271 of 4193 samples
  Cartesian angular speed limit on 418 of 4193 samples
  last maximum joint error: 0.0041 rad
  velocity limit clamped on 271 of 4193 samples
```

### How it differs from `pose ee --from-marker`

`pose ee` reads the marker once and sends one trajectory. `pose follow` reads the
marker's `feedback` stream — the topic that publishes throughout a drag — and
commands at the controller rate.

At startup, `pose follow` treats the marker as an absolute pose in `world`.
It refuses a marker farther than `--max-start-distance` or
`--max-start-angle`; otherwise it approaches the marker through the same
bounded IK and command path used during following. Move the marker to
**Current** before starting instead of relying on an old RViz goal.

When the marker target changes, a background worker calls MoveIt
`/compute_ik`. The first request uses the measured joints; later requests use
the previous accepted target as every candidate's seed and continuity reference.
It keeps only the newest pending request, selects the nearest bounded candidate,
and atomically latches that solution. The control loop therefore keeps streaming
the previous valid target while MoveIt is busy, rather than blocking on a
service round trip.

The fast loop compares that latched target with `/joint_states`. While measured
joints trail it, the outer feedback advances the position command through the
motor's impedance droop. The marker's orientation is followed along the shortest
quaternion path, with both the intermediate IK step and commanded angular speed
bounded.

Every sample goes through `CommandGate.follow`, which **clamps rather than
refuses**. Dragging faster than the arm can move is normal operation, so the
step is limited to the profile's velocity bound and the count of clamped samples
is reported at the end. Position limits clamp the same way: the arm stops at the
limit and keeps tracking rather than ending the session mid-drag.

The outer feedback is accumulated from the **previous command**, using error
against the measured joints. These motors hold position by sitting behind their
command, so merely sending the IK joint target reproduces a standing error.
Continuing to advance the command while feedback trails the target closes that
error; `max_lead` still bounds the compensation if an obstacle blocks the arm.

A third bound follows from that: `max_lead` limits how far the command may run
ahead of the measured position, so a joint held still by an obstacle cannot wind
up command, and with it torque, without limit. It is set from the profile's
velocity limit over `LEAD_SEC`, and `lead limit` in the clamp report names it.

### The arm's posture will not match RViz's ghost

Following puts the tool centre point on the marker, and leaves the elbow
somewhere MoveIt's goal-state ghost does not show. That is redundancy, not error.

The arm has seven joints and a pose has six degrees of freedom, so a whole
one-dimensional family of joint configurations reaches any given tool centre
point — the elbow swings around a cone while the hand stays put. Which member of
that family you get depends on who solved the inverse kinematics:

| | Solver | Behaviour |
| --- | --- | --- |
| RViz ghost | MoveIt's KDL solver | Whatever it converges to from its seed; the elbow can land differently each solve |
| `pose follow` | MoveIt `/compute_ik`, measured-joint seed | Latches the newest valid solution and feedback-tracks it |
| `pose ee --from-marker` | `/compute_ik`, MoveIt's solver | Matches the ghost, because it is the same solver |

`pose follow` seeds each request from the measured joints, so MoveIt normally
selects a nearby branch. The ghost can still differ because RViz solved at a
different time or with a different seed.

### Reading the report

The clamp counts describe the *command*, not the arm. The original
`trailed the marker by` line is retained for baseline comparison, but it uses
the marker snapshot associated with the currently accepted IK result. During a
fast drag that snapshot can be older than the live RViz marker. The new live
marker line and lag decomposition make that delay explicit.

| Line | What it means |
| --- | --- |
| `trailed the marker by` | Baseline-compatible distance from the accepted IK request's marker snapshot to measured TCP |
| `live marker to measured TCP` | Current RViz marker to measured TCP; use this as total moving-target error |
| `marker_update_staleness` | Distance the live marker moved beyond the marker snapshot used by the accepted IK result |
| `accepted_marker_to_ik_target` | Backlog created by the bounded intermediate IK target |
| `ik_target_to_command` | Outer-loop and Cartesian-cap backlog between the IK target and active command |
| `command_to_measured` | Physical lag from the command active at state-sample time to measured TCP |
| `last TCP position error` | Distance to the marker at shutdown |
| `within 2.0 mm` | Samples inside the configured deadband |
| `TCP orientation trailed the marker by` | Average and worst orientation error between the RViz marker and measured TCP |
| `last TCP orientation error` | Orientation error to the RViz marker at shutdown |
| `within 2.0 deg` | Samples inside the configured orientation deadband |
| `actual control rate` | Achieved loop frequency and average time waiting for fresh joint feedback |
| `IK requests` | Submitted, successful, failed, and superseded MoveIt solves |
| `Cartesian speed limit` | Samples whose TCP request was capped at `--max-tcp-speed` |
| `Cartesian angular speed limit` | Samples whose TCP rotation request was capped at `--max-tcp-angular-speed` |
| `last maximum joint error` | Largest absolute joint error against the latched IK target at shutdown |
| `velocity limit` | Normal. The marker was dragged faster than the profile allows the arm to move |
| `lead limit` | The arm fell more than `LEAD_SEC` of travel behind its command. Frequent hits mean the arm cannot keep up — try `--gravity`, or drag more slowly |
| `position limit` | The drag asked for a joint past the profile's bound. The arm stops there and keeps tracking the rest |

`--output` needs `--execute`, because a dry run contains no measured samples.
The JSON stores the same summaries plus a 100 Hz trace with TCP poses, joint
positions, IK sequence and per-sample limit flags. It is intended for temporary
experiment storage such as `/tmp/right-follow.json`; do not commit large run
files to Git.

Additive schema-v1 fields record:

- `result.startup_alignment`: completion flag and elapsed completion time.
- `result.ik.events`: request, solve-start, complete and accepted elapsed
  timestamps, outcomes and latencies for every sequence.
- `result.position_error_signed_projection_m`: each position layer projected
  onto the final live-marker error direction. The four downstream projections
  sum to `live_marker_to_measured`; negative values cancel error.
- `result.ik_target_jumps`: per-joint worst single-step target delta and
  threshold-crossing events. These events never change control.
- `settings.diagnostic_profile` and each trace sample's
  `diagnostic_profile`: the reproducible profile and active phase.

### Stopping, and what the arm does then

Following ends when `--seconds` runs out, or on Ctrl-C. Either way the last
thing that happens is that commanding stops and any feedforward torque is set
back to zero. The trajectory controller holds its last commanded position, which
may lead the measured joints by the bounded `max_lead`. The arm can therefore
settle slightly after shutdown, especially when gravity feedforward is removed;
it does not go limp because position control remains active.

It ends on its own by design. A servo loop left running is a robot that moves
when somebody brushes the marker hours later.

### The all-zeros home pose is singular

At exactly `q = 0` the arm's Jacobian has rank 5, and its entire z row is zero.
MoveIt can fail to find some directions from that seed. Dragging the marker
up produces no motion, correctly — the damped inverse asks for the bounded step,
which is nothing.

```text
q = 0     singular values: [1.8674 1.5266 1.4142 0.286 0.2856 0.0000]   rank 5
elbow 0.6 singular values: [1.8601 1.5187 1.4142 0.275 0.2743 0.0524]   rank 6
```

Bend the arm before following. Anything off the exact home pose is fine:

```bash
# --execute sends the trajectory, bending the elbow off the singularity first:
robotctl pose joints --group openarm_right_arm --values 0,0,0,0.6,0,0,0 --execute
```

`--gravity` is worth using here in a way it is not for a single pose:
`pose ee --settle` corrects where a move *ends*, which a moving arm never does.
Tune the scale first with `pose gravity --sweep`.

**Exit codes:** `0` followed and stopped; `2` unknown group, group with no
planning group, a gravity scale outside range, non-positive `--seconds`, or no
adapter; `3` refused by the safety gate.

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

Publish an identification excitation and record what the arm did.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--group` | *required* | Group to excite; must have a trajectory controller |
| `--output` | *required with `--execute`* | `.npz` recording to write |
| `--repetitions` | `1` | `3` runs the same excitation three times and writes a manifest |
| `--dry-run` | on | Build and authorize the track without publishing |
| `--execute` | off | Publish it and record the response |
| `--amplitude-scale` | `0.3` | Fraction of the per-joint range to excite, in `(0, 1]` |

```bash
robotctl r2s collect --group openarm_right_arm
```

```text
openarm_right_arm: amplitude_scale=0.3 samples=611 (6.1 s at 100 Hz) phases=hold,bridge,step,ramp,multisine
DRY RUN: nothing was published; pass --execute to collect
```

Amplitudes are 5 % of each joint's range scaled by `--amplitude-scale`, so the
default excites 1.5 % of range.

```bash
# --execute publishes position commands at 100 Hz and the arm moves through the
# whole excitation. Keep the E-stop within reach.
robotctl r2s collect --group openarm_right_arm --output run.npz --execute
```

```text
published 611 samples, recorded 612 (0 did not cover the group)
  largest gap 10.0 ms against a 10.0 ms median (1.0 command periods)
collect: run.npz
```

### The excitation starts where the arm is

`neutral` is the arm's **current measured pose**, not the midpoint of its range.
The arms' limits are symmetric, so the midpoint is the all-zeros pose — a large
unplanned move before the excitation even begins, and the pose where the arm is
straight out and most loaded.

That is also why a dry run needs the robot. Building the review track around the
midpoint while `--execute` used the current pose would mean reviewing a track
that never runs.

The whole track is authorized before the first sample is published — every sample
against the position limits, every step against the velocity limits. A run that
stopped partway would leave the arm mid-excitation at a velocity nobody chose.

### `bridge` in the phase list

The phases are shapes, and the joins between them are discontinuities. Measured
against the real profile, the join from the ramp into the multisine asks for
**seven times** the arms' 2.0 rad/s at 100 Hz. Published as position commands the
gate refuses the whole track, and shrinking the amplitude until every
discontinuity fits in one sample would shrink it by that same factor of seven.

So the joins are bridged at the velocity limit instead: 11 extra samples at the
default scale, and the amplitude is kept.

A multisine too fast to slew is refused rather than bridged, and the refusal says
by how much to scale. Its peak slew is frequency times amplitude, so extra time
changes neither — on this profile `--amplitude-scale 0.6` is refused for that
reason.

### What the recording holds

Two streams, each with its own clock, never resampled and never paired:

| Key | Meaning |
| --- | --- |
| `command_time_ns`, `command` | Stamped from the node clock when published |
| `measured_time_ns`, `measured` | One row per `/joint_states` message, stamped with its `header.stamp` |
| `joint_names` | Canonical, the group's, in the group's order |
| `profile`, `asset_id`, `manifest_sha256` | What it was recorded on |

**Not paired into one row.** A loop that wrote each command beside the state it
read in the same cycle would be asserting that the state responds to that
command. It does not — it responds to one from several cycles back, and that lag
is `delay_sec`, a parameter being measured. Pairing at record time bakes in zero
and destroys it. `r2s normalize` puts both on a common grid afterwards, which
keeps the alignment a decision that can still be revised.

The reported gap is what says whether the recording can be trusted:
`/joint_states` is subscribed best-effort, so messages can be dropped, and
`normalize` interpolates across a gap without knowing it was one.

### Three repetitions, and the one that is held out

```bash
# --execute publishes the excitation three times and the arm moves through all
# of it, returning to the start between runs. Keep the E-stop within reach.
robotctl r2s collect --group openarm_right_arm --output run.npz --repetitions 3 --execute
```

Writes `run0.npz`, `run1.npz`, `run2.npz` and a manifest `run.json`:

```json
{
  "group": "openarm_right_arm",
  "runs": [{"path": "run0.npz"}, {"path": "run1.npz"}, {"path": "run2.npz"}],
  "fit_runs": [0, 1],
  "holdout_runs": [2]
}
```

Exactly three, or one. `split_repetitions` has always required three — two to fit
and one held out — and the v2 bundle's `source` block has always carried
`fit_runs` and `holdout_runs`. Two repetitions would leave one of each, and a
model fitted on a single run has nothing to be validated against, so `2` is
refused.

The arm returns to the starting pose between runs. The excitation ends wherever
its last phase left it, not at neutral, so without that the second run would be a
different experiment.

The manifest is written only after every recording is on disk. One naming a file
that was never written would be worse than none.

**Exit codes:** `0` published or planned; `2` no `--group`, `--execute` without
`--output`, a gripper group, an amplitude too fast to slew, or no adapter;
`3` the excitation leaves the profile's envelope; `SystemExit` for
`--amplitude-scale` outside `(0, 1]`.

## `robotctl r2s normalize`

Resample a recorded run onto the profile's command rate and write an HDF5
track.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--input` | *required* | `.npz` with `command_time_ns`, `command`, `measured_time_ns`, `measured`, `joint_names` |
| `--output` | *required* | HDF5 track to write |
| `--max-gap-periods` | `20` | Command periods a stream may skip before the hole counts as missing data |

```bash
robotctl r2s normalize --input run.npz --output track.h5
```

```text
normalize: track.h5 sha256=6f1c…
```

The printed digest identifies the track and is carried into the fit result.

### A gap is a hole, not a straight line

`/joint_states` is subscribed best-effort, so messages drop, and interpolating
between whatever samples arrived turns a dropped run into a smooth curve through
data nobody measured. A fit cannot tell that curve from a measurement, so a gap
longer than `--max-gap-periods` is refused:

```text
error: the measured stream has a gap of 410.0 ms at 0.49 s, over the 200.0 ms
allowed (20 command periods). Interpolating across it would draw a smooth line
through data nobody measured.
```

The gap is judged against the **command** period, not the stream's own median. A
recording that is uniformly twenty-five times too slow has a perfectly consistent
median and still cannot support a fit at this rate, and a median-relative check
would pass it.

Both streams are checked: our own publisher can stall, and that leaves the same
hole.

Writing HDF5 needs the optional extra (`pip install robot-control[hdf5]`); not
having it exits `2` with that message rather than raising.

**Exit codes:** `0` written; `2` an unreadable recording, streams that do not
overlap, a joint the command never moved, or no HDF5 support; `SystemExit` if
either path is missing.

## `robotctl r2s fit`

Fit a second-order joint model to a normalized track.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--track` | *one of these two* | HDF5 track from `normalize` |
| `--manifest` | *one of these two* | Run manifest; fits across the runs it names |
| `--output` | *required* | JSON estimate to write |
| `--population` | `128` | Candidate population size; must be positive |
| `--static` | — | Stiffness set from `r2s identify`; adds the gravity term and turns the ratios into physical parameters |
| `--urdf` | *required with `--static`* | Robot description, to compute the modelled torque along the track |
| `--accept-inertia-gap` | off | Write a fit whose measured `kp/k` inertia disagrees with the URDF-dependent `1/g` cross-check; the gap remains recorded |

```bash
robotctl r2s fit --track track.h5 --output estimate.json
```

The estimate records stiffness, damping, friction, and per-joint residual RMSE,
along with the source track's SHA-256. Those three are **ratios**: the fit is

```
qdd = k (q_cmd - q) - d qd - f sign(qd)
```

so what comes out is `k = kp/J`, `d = b/J`, `f = tau_f/J` — every parameter
divided by an inertia the track cannot separate from them.

### Fitting across the runs a manifest names

```bash
robotctl r2s fit --manifest run.json --output estimate.json
```

The two fit runs are stacked as rows of **one** regression rather than fitted
separately and averaged. They are repeats of one experiment, so the parameters
are shared and every sample is evidence about the same numbers; averaging
per-run fits would weight a short run as heavily as a long one.

They are not concatenated into a single track either. The join between two runs
is not motion — the arm was driven back to the start in between — and
differentiating across it would invent an acceleration that never happened.

A manifest that fits on a run it also holds out is refused. Validating against a
run the model was fitted on validates nothing.

The estimate records `fit_runs` and `holdout_runs`, which `r2s bundle` carries
into the bundle's `source` block: the fit is what knows which runs it used.

### With `--static`: the gravity term, and physical units

Without a gravity term the regression has nowhere to put a standing load but the
stiffness and damping columns. On a real arm that is not a small error: on a
seven-joint arm under its own weight it can drive a fitted parameter out of range
entirely, so the fit refuses rather than reporting anything. A lighter load is
worse, because it gives a plausible wrong number instead.

`--static` supplies both missing pieces. The modelled torque, corrected by the
`alpha` that `r2s identify` measured, enters as a fourth column:

```
qdd = k (q_cmd - q) - d qd - f sign(qd) - g tau_g(q)
```

Its coefficient `g` is `1/J`, the inertia on its own. Dump the URDF from the
running stack:

```bash
ros2 param get --hide-type /robot_state_publisher robot_description > robot.urdf
robotctl r2s fit --track track.h5 --output estimate.json --static static.json --urdf robot.urdf
```

```text
  joint            J (kg.m2)   b (N.m.s)   tau_f (N.m)   kp (N.m/rad)   J from gravity   gap
  r_aj_1             0.35000      1.5000        0.4000          20.00          0.35000  0.0%
  ...
```

`J` in the first column is `kp/k`: the static fit's stiffness, which has no
inertia in it, over the dynamic fit's, which is that same stiffness divided by
one. `J from gravity` is `1/g`, from a different column of a different
model. A disagreement is refused by default. Use `--accept-inertia-gap` only
when the asset masses are known to be approximate and the recorded gap has
been reviewed; this does not erase or rewrite the cross-check.
experiment. **They agreeing is evidence, not arithmetic** — it is the one check
that catches a static estimate measured on another robot, or a URDF that is not
the arm the track came from. A gap above 25% is refused and nothing is written.

The output then also carries `inertia_kg_m2`, `damping_nm_s_per_rad`,
`friction_nm` and `stiffness_nm_per_rad` — the set a simulator needs to behave
like this arm, measured rather than taken from the URDF.

**Exit codes:** `0` written; `2` neither `--track` nor `--manifest`, `--static`
without `--urdf`, a static estimate or manifest from another profile or asset, a
track covering other joints, a manifest that fits on its own holdout, a URDF the
chain cannot be built from, or dynamics the fit cannot identify; `3` the two
inertias disagree; `SystemExit` for a missing `--output` or a non-positive
`--population`.

## `robotctl r2s identify`

Fit joint stiffness, stiction and a torque-model correction from gravity sweeps
measured at several poses.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--sweep` | — | A file from `pose gravity --output`; pass it once per pose. At least two in total, counting collected ones |
| `--output` | *required* | JSON stiffness set to write |
| `--noise` | `0.0004` | Radians below which a joint counts as not having moved |
| `--collect` | off | Design a pose set and sweep at each pose, rather than only fitting files that exist |
| `--group` | *required with `--collect`* | Group to collect on; must declare an `effort_controller` |
| `--sweep-dir` | *required with `--collect --execute`* | Directory to write one sweep file per collected pose |
| `--poses` | `4` | Poses to design; at least 2 |
| `--scales` | `0,0.5,1.0` | Comma-separated gravity scales to hold at every pose |
| `--reach` | `0.5` | Fraction of each joint's range the poses may use, about its middle |
| `--seed` | `0` | Pose-set seed; the same seed designs the same poses |
| `--duration` | `3.0` | Seconds to take moving between poses |
| `--hold-sec` | `2.0` | Seconds to publish at each scale before measuring |
| `--execute` | off | Move the arm through the designed poses |

```bash
robotctl r2s identify --sweep sweeps/pose0.json --sweep sweeps/pose1.json --sweep sweeps/pose2.json --output static.json
```

```text
identify: 3 poses, 12 rounds at most per joint
  joint            kp (N.m/rad)   alpha   offset (rad)  residual (rad)   cond  rounds  frozen
  r_aj_1                  7.52   1.083       +0.00210         0.00021    4.8      12       0
  ...
```

### Why several poses, and what each column means

At equilibrium the impedance controller balances the load with a standing
position error, so with a fraction `s` of the modelled torque fed forward:

```
error = (alpha - s) * tau_model / kp + c
```

`kp` is the stiffness the hardware applies, `alpha` is the factor the modelled
torque was wrong by — masses the URDF does not know about, cabling, anything
bolted on — and `c` is the scale-independent offset, which is where stiction
lives.

At **one** pose `tau_model` is a constant column, indistinguishable from the
offset, so `kp`, the friction and the model's own error are one equation in three
unknowns. That is why fitting the first real sweeps joint by joint gave `kp`
values of 7.5, 15.4 and 28.4 against the vendor header's 20 — scattered in both
directions, the signature of an under-determined fit rather than a surprising
robot. Several poses vary `tau_model` while the friction stays put, and the pair
separates.

`cond` is the conditioning of that joint's regression. Two poses whose modelled
torque differs by 10% sit near 50; nearly the same pose twice sits in the
hundreds. Above 200 the joint is reported as not identified rather than given a
number.

`frozen` counts rounds dropped because the torque changed and the joint did not
move — it was inside its stiction band, held by friction rather than by the
position error, and those samples pull the fitted stiffness towards infinity
rather than merely adding noise. Both members of such a pair go: the first is
where the joint stopped, and nothing says whether that was its equilibrium or the
edge of the band. Measured on the real right arm, `r_aj_4` sat at exactly
`+0.0075` rad across six consecutive scales.

### Collecting the poses instead of driving them by hand

`--collect` designs a pose set, moves the arm to each pose, sweeps there, and
writes one file per pose. Review it first — without `--execute` nothing moves:

```bash
robotctl r2s identify --collect --group openarm_right_arm --poses 4 --output static.json
```

```text
openarm_right_arm: 4 poses, scales 0,0.5,1
            r_aj_1   r_aj_2   r_aj_3   r_aj_4   r_aj_5   r_aj_6   r_aj_7
  pose 0   +0.000   +0.000   +0.000   +0.000   +0.000   +0.000   +0.000
  pose 1   +1.555   +0.898   -0.125   +0.515   -0.008   +0.059   +0.897
  ...
  cond        2.8      2.8      2.8      2.8      2.8      2.8      2.8
  worst conditioned: r_aj_1 at 2.8
DRY RUN: nothing moved and nothing was written.
```

> **The dry run is the review, not a formality.** Nothing here checks the arm
> against itself, the table, or anything on it. The profile bounds each joint
> separately; it says nothing about whether the arm in that posture is where the
> arm can be. Look at each pose in RViz before committing to it.

The pose set is deterministic in `--seed`, so `--execute` with the same seed
visits the poses you just reviewed:

```bash
# --execute moves the arm to each designed pose in turn and publishes torque
# there. Keep the E-stop within reach.
robotctl r2s identify --collect --group openarm_right_arm --poses 4 --seed 0 \
  --sweep-dir sweeps --output static.json --execute
```

The whole itinerary is authorized before the first move — every pose against the
profile's position limits, and every leg against its velocity limits at
`--duration`. A run that stopped partway because the fifth pose was out of range
would leave the arm somewhere nobody chose, and refusing up front costs nothing.
The conditioning is checked the same way, before moving rather than after
fitting.

Torque is released at every pose, not only at the end.

`--reach` keeps the poses off the hard stops. A pose against a stop cannot droop,
and a joint that cannot droop looks exactly like one held by stiction — the fit
would read the constraint as friction.

`--sweep` composes with `--collect`, so a joint frozen at every designed pose can
be fixed by adding one pose by hand rather than recollecting everything:

```bash
# Both --execute lines publish torque and the second also moves the arm; pose
# the joint you want to free by hand first, then measure there.
robotctl pose gravity --group openarm_right_arm --execute --sweep 0,0.5,1.0 --output sweeps/byhand.json
robotctl r2s identify --collect --group openarm_right_arm --sweep sweeps/byhand.json --sweep-dir sweeps2 --output static.json --execute
```

### Why it refuses rather than reporting a partial answer

Nothing written unless every joint identified. Least squares always returns
something, and a stiffness for a joint whose load never varied is that something;
`r2s fit` turns it into an inertia, and after that nothing could tell it from a
measured one. The report still prints, naming which joints failed and why, which
is what says whether to add a pose or to free a stuck joint.

**Exit codes:** `0` written; `2` no `--output`, a sweep from another profile or
asset, a tampered file, or sweeps covering different groups; `3` fewer than two
`--sweep` files, or at least one joint could not be identified.

## `robotctl r2s bundle`

Merge identified parameters into a schema v2 calibration bundle.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--base` | *required* | Schema v2 bundle to merge into |
| `--fit` | *required* | Output of `r2s fit --static`; pass it once per group |
| `--output` | *required* | Bundle to write |

```bash
robotctl r2s bundle --base bundle.json --fit right.json --fit left.json --output identified.json
```

Each group gains an `identified` block beside its `nominal` one. They are
different kinds of number: `nominal` is one guess per group, `identified` is one
measured value per joint, so it carries the sweeps and the track it came from and
the cross-check it survived.

A fit produced without `--static` is refused. Its stiffness, damping and friction
are ratios to an inertia rather than parameters, and nothing downstream would be
able to tell.

### `fitted_against`, and why it is not redundant

The block records the profile, asset and manifest hash it was **measured on**,
beside the header saying what the bundle **claims to be**. That looks like the
same information twice, and it is exactly the check a checksum cannot do: the
checksum is recomputed on write, so it signs a block pasted from another robot's
bundle just as happily. `load_bundle` refuses a mismatch, so `validate` and
`export` both refuse it.

Also refused: a block whose joint names are not the group's, an array that is not
one finite value per joint, a non-positive inertia or stiffness, a negative
damping or friction, and a block with no provenance — a number with no source is
not a measurement.

**Exit codes:** `0` written; `2` a missing path, a base bundle that cannot be
read, a base that is not schema v2, a fit without `--static`, or a fit for a
group the bundle has no entry for; `3` a parameter the bundle refuses to carry.

## `robotctl r2s validate`

Check holdout metrics against the acceptance thresholds.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--bundle` | *required* | Calibration bundle to validate against |
| `--output` | *required* | JSON verdict to write |
| `--manifest` | — | Run manifest from `collect --repetitions 3`; scores the held-out run |
| `--fit` | *required with `--manifest`* | Fit estimate to score |
| `--urdf` | *required if the fit carries a gravity term* | Robot description |
| `--metrics` | — | JSON holdout metrics computed elsewhere |

```bash
robotctl r2s validate --bundle bundle.json --manifest run.json --fit estimate.json --output verdict.json
```

```text
validate: schema v2, status=validated
  identified parameters: openarm_left_arm, openarm_right_arm
  command_period_sec: 0.01
  delay_residual_sec: 0
  improvement_fraction: 0.9997
  openarm_rmse_rad: 8.9e-05
  tesollo_rmse_rad: 0
```

### What is actually measured

The model is simulated **open loop** along the held-out run's commands and
compared with what the arm did. Open loop on purpose: feeding the measurement
back each step would score how well the model interpolates between samples it was
already given, which every model does well. A simulator has to run without them.

`improvement_fraction` is against the model-free assumption that **the arm
reached its command** — what somebody with no identification at all would
believe. That is what a model has to beat to be worth carrying.

`delay_residual_sec` is how far the prediction would have to slide to line up
best. A model that captured the loop's delay needs no sliding; one that did not
shows it here rather than spreading it through the position error.

Which of `openarm_rmse_rad` and `tesollo_rmse_rad` a run fills comes from the
group's own name against the profile's declared components — the two have
different thresholds, so putting a run's error under the wrong one would compare
it against the wrong bound. The other is left at `0`, which cannot fail: an arm
run says nothing about the hand, and inventing a number for it would be worse
than admitting that.

`--metrics` remains for a verdict computed elsewhere.

Loading the bundle is itself part of the verdict, so a bundle that cannot be read
— a bad checksum, the wrong asset, or identified parameters fitted against
another robot — exits `2` with the reason rather than raising.

**Exit codes:** `0` validated; `2` the bundle could not be read, neither
`--metrics` nor `--manifest` was given, `--manifest` without `--fit`, or a
manifest from another profile or asset; `3` the model is inadequate;
`SystemExit` for a missing path.

## `robotctl r2s export`

Export a validated bundle. Refuses anything not already validated.

| Argument | Default | Meaning |
| --- | --- | --- |
| `--profile` | `openarm_tesollo` | Built-in profile to load |
| `--bundle` | *required* | Calibration bundle to export |
| `--validation` | *required* | Verdict JSON from `validate` |
| `--output` | *required* | Destination bundle |
| `--hdgp` | — | Also write the schema-v1 actuator calibration consumed by the HDGP training environment |
| `--hdgp-max-spread` | `0.25` | Maximum `(max-min)/mean` joint disagreement allowed when collapsing a group to one HDGP scalar |

```bash
robotctl r2s export --bundle bundle.json --validation verdict.json --output release.json
```

To hand the same validated measurement to HDGP, request the additional output
explicitly. A group whose joints disagree beyond `--hdgp-max-spread` is refused
rather than averaged into a scalar that describes none of them.

```bash
robotctl r2s export --bundle bundle.json --validation verdict.json \
  --output release.json --hdgp real2sim_actuator.json \
  --hdgp-max-spread 0.25
```

The bundle is copied byte for byte, so anything `load_bundle` accepted travels
with it — including a group's `identified` block and its provenance. The bundle is
re-read before the copy, so the same refusals that stop `validate` stop an export.

**Exit codes:** `0` exported; `2` the bundle could not be read; `3` the verdict is
not `validated`; `SystemExit` for a missing path.

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
