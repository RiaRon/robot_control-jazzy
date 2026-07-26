# Gravity compensation and marker servoing

## Why

`robotctl pose ee --from-marker --execute --settle` reaches a dragged pose, but
it gets there by iterating: command, let the arm settle, measure the shortfall,
command past the target by that much. Measured on the real right arm, the first
pass lands 58–85 mm out, and two corrections bring it under 2 mm.

That works because the shortfall is a *steady-state* error. The trajectory
controller claims the position interface only, `OpenArm_v10HW::write` therefore
sends `MITParam{kp, kd, q_cmd, 0, 0}`, and a joint can only produce holding
torque while it sits short of its command:

```
tau = kp * (q_cmd - q) + kd * (0 - dq) + tau_ff,   tau_ff = 0
=> q settles where kp * error balances the gravity load
=> error ~ tau_gravity / kp
```

Measured, at the elbow: 0.179 rad against `kp` 60, or about 10.7 N·m of holding
torque. The pattern follows load, not step size — shoulder and elbow miss by
0.07–0.18 rad, the unloaded wrist joints by 0.004.

Two consequences set the scope of this plan:

1. **Settling is slow.** Each pass costs a full `--duration`, so reaching a pose
   takes three moves instead of one.
2. **Settling cannot help a moving arm.** The error is present throughout the
   motion, so an arm asked to follow a dragged marker in real time would trail
   it by that same 10 degrees at the elbow the whole way. Real-time following
   makes gravity compensation a precondition, not an improvement.

## What already exists

Verified on the running stack, not assumed:

| Piece | State |
| --- | --- |
| `effort` command interface | `[available] [unclaimed]` on all 14 arm joints |
| `OpenArm_v10HW::write` | passes `tau_commands_` straight through as MIT `tau` |
| Streaming controller | `*_forward_position_controller` (`ForwardCommandController`) is configured, 100 Hz |
| `controller_manager` rate | 100 Hz, matching the profile's `command_rate_hz` |
| Marker target stream | `.../robot_interaction_interactive_marker_topic/feedback` publishes throughout a drag |
| Streaming safety | `CommandGate.authorize` already takes one command at a time with a watchdog |
| Dynamics library | `PyKDL` 1.5.1 and `urdf_parser_py` present; no `pinocchio`, no `kdl_parser_py` |

So no vendor C++ change is needed to compensate gravity: something has to claim
the effort interfaces and publish a torque, and the hardware already forwards it.

## Global constraints

Carried forward from `2026-07-26-pose-setting-and-ros-adapter.md`, still binding:

- Work only on the long-lived `jazzy` branch.
- Fake hardware is the default everywhere; reaching hardware needs an explicit
  flag.
- Profile limits are authoritative over URDF limits.
- `import robot_control` must not require `rclpy`.
- Every vendor tree change needs a declared patch and a `post_patch_sha256`
  update.

Two constraints change, deliberately, and this plan is the record of that:

- **`moveit_servo` stays out, but continuous command streaming comes in.** The
  earlier plan said plan-then-execute only. Following a dragged marker is
  streaming by definition. What stays out is `moveit_servo` itself: the loop is
  ours, so every sample passes our gate.
- **Effort becomes a commanded quantity.** Until now only position was ever
  sent. A wrong torque does not merely mispose the arm, it accelerates it, so
  the gate has to bound effort the way it bounds position and velocity, and the
  compensation has to be introduced behind a scale that starts at zero.

## Design

### Gravity torque without a dynamics package

Gravity torque needs only masses and centre-of-mass positions, not inertia
tensors:

```
tau_j = -sum_{i >= j} m_i * g . (z_j x (p_com_i - p_j))
```

for revolute joints, with `g = (0, 0, -9.81)`, `z_j` and `p_j` the joint's axis
and origin in the world frame, and `p_com_i` link `i`'s centre of mass. Every
term comes from forward kinematics over the URDF.

That argues for implementing FK in numpy rather than reaching for KDL: the same
code yields the Jacobian the servo loop needs for differential IK, it has no ROS
dependency so it stays testable offline, and it can be validated against
`/compute_fk` on the live robot rather than trusted.

Rejected alternatives: `pinocchio` is not packaged for this platform;
`kdl_parser_py` is absent, so PyKDL would still need the URDF walked by hand,
and PyKDL gives no Jacobian we could reuse for IK without a second conversion.

### Owning the controller configuration

`demo.launch.py` builds the controller parameters path as
`<share of runtime_config_package>/config/v10_controllers/<controllers_file>`,
and both halves are launch arguments. Adding an effort controller therefore does
**not** need a patch against `openarm_bringup`: a small package of ours under
`ros_ws/src` supplying that same layout, selected with
`runtime_config_package:=...`, keeps the vendor snapshot untouched and gives the
servo node a home.

### Refuse versus clamp

`CommandGate` refuses rather than massages, and that is right for a discrete
command: exit 3 means "understood and declined". A servo loop cannot work that
way — an operator dragging a marker faster than the arm can move would abort the
session on the first quick flick.

So streaming gets a separate entry point with explicitly different semantics:
`follow()` steps toward the target at no more than the velocity limit, clamps
into the position limits, and *reports* what it limited, rather than raising.
`authorize()` and `authorize_trajectory()` keep refusing. One gate, two
documented contracts, so a reader is never left guessing which one applies.

## Tasks

### Task 1 — `kinematics.py`: FK, Jacobian, gravity torque from the URDF

Pure numpy, no `rclpy`. Parse the URDF chain for a group's joints, and expose
forward kinematics for any link, the geometric Jacobian at the tip, and gravity
torque per joint.

Tests: a one-link pendulum whose gravity torque is `m*g*L*sin(theta)` in closed
form; a two-link arm; the Jacobian checked against a finite difference of FK;
and FK checked against `/compute_fk` on the live stack in the smoke test.

**Done when** gravity torque and FK are correct on analytic fixtures, and live
FK agrees with `/compute_fk` to under a millimetre.

### Task 2 — effort limits and `follow()` in the gate

Add effort bounds from the profile's `effort` field, and `follow(target,
elapsed)` returning a clamped command plus a note naming what limited it.
`authorize*` behaviour unchanged.

**Done when** a fast target is clamped rather than refused, a target outside the
position limits is clamped at the limit, an effort over the profile bound is
refused, and every existing safety test still passes.

### Task 3 — our controller package and the effort controller

A minimal ament package under `ros_ws/src` carrying
`config/v10_controllers/*.yaml` that adds `*_forward_effort_controller` beside
the vendor controllers, and a `pose_bringup.sh` flag selecting it.

**Done when** the stack comes up with position and effort both claimed, on fake
hardware and then real, and `ros2 control list_hardware_components` shows the
effort interfaces claimed.

### Task 4 — gravity compensation

Publish gravity torque at the controller rate, behind a `--gravity SCALE` that
defaults to `0.0`. Report the residual with compensation on so the improvement
is measured, not asserted.

**Done when** the standing residual at a loaded pose falls from tens of
millimetres to single digits with `--settle` off, at a scale recorded in the
verification document.

### Task 5 — `robotctl pose follow`: marker servoing

Subscribe to the marker feedback stream, solve differential IK with the Jacobian
from Task 1, gate each sample with `follow()`, and publish to the forward
position controller at the controller rate. Hold the last authorised command
when the drag stops.

**Done when** dragging the marker moves the arm continuously, releasing it
leaves the arm still, and Ctrl-C leaves the arm holding rather than limp.

### Task 6 — documentation

README (Korean, operator order) and `docs/cli.md` (per-command reference), plus
the measured before-and-after residuals in `docs/jazzy-verification.md`.

## Risks

- **A wrong gravity model accelerates the arm.** Hence the scale defaulting to
  zero, the effort bound in the gate, and validating FK against `/compute_fk`
  before any torque is sent.
- **URDF masses may not match the built robot.** Compensation is only as good as
  the model; the scale is the operator's adjustment for that, and the residual
  is always reported so a bad model is visible rather than silent.
- **Streaming has no watchdog story yet.** A position controller holds its last
  command if the publisher dies, which is the safe failure, but the servo loop
  must not treat "no new target" as "go to zero".
