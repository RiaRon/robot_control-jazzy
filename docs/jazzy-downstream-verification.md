# Jazzy Downstream Verification

Date: 2026-08-01

## Lineage

- Humble mainline baseline: `b5dee539014cd6779c27d07f0808f8be658a0710`
- Valid Jazzy surface baseline: `2433a305d7564b91883f1648a073baef1e82e1cc`
- Integration branch: `integration/humble-mainline-jazzy-downstream`
- Humble remains an ancestor; neither source checkout was reset, cleaned, or
  committed during integration.

The downstream branch restores the Jazzy ROS/MoveIt/RViz/Gazebo surface and
keeps Humble's current tuning, direct-torque identification, canonical asset
URDF, calibration, and Real2Sim core. It also carries the preserved local
IK-follow and Cartesian servo work.

## Offline and static verification

With the repository beside `/home/user/rl_ws/hdgp@b32620c`:

```text
PYTHONPATH=src:. pytest -q
600 passed, 1 skipped in 49.12s
```

The single skip is the suite's environment-dependent ROS import isolation test.

Focused Jazzy distribution contracts:

```text
69 passed in 0.40s
```

The set covers distro selection, Jazzy build/dependency wrappers, DG5F Jazzy
contracts, pose bringup, MoveIt/TCP configuration, vendor snapshots, and CLI
documentation. `bash -n ros_ws/*.sh` also passed.

GUI/servo and safety-focused verification:

```text
170 passed, 1 skipped in 7.43s
```

It covers one-shot marker targets, continuous latest-only IK, Cartesian speed
limits, joint/velocity/lead clamps, missing markers, unreachable targets,
orientation hold, droop recovery, stop behavior, and the rule that dry-run
publishes nothing.

## CLI dry-run evidence

`robotctl r2s preflight` completed offline and reported:

```text
profile: openarm_tesollo
asset: openarm_tesollo_sensor_rl
joints: 35
publish_enabled: false
```

`robotctl pose ready` completed without ROS and printed two `DRY RUN` arm plans,
each stating that `--execute` is required to send.

`robotctl r2s collect --dry-run` imports ROS and reads `/joint_states` to build
an itinerary, but never publishes. Without a running fake stack it exited with
the expected unavailable message after the state timeout. Its no-publication
contract is covered by recording-backend tests.

## Jazzy build

After sourcing `/opt/ros/jazzy/setup.bash`:

```text
Summary: 11 packages finished [18.1s]
```

`delto_hardware`, `openarm_can`, and `openarm_hardware` emitted compiler
warnings only; the build returned zero.

## Sandbox-limited fake-hardware smoke

`ros_ws/smoke_openarm_fake.sh` was attempted with a temporary ROS log directory
and localhost-only discovery. It was bounded and cleaned up correctly, but the
managed environment prevented a valid ROS graph:

- `getifaddrs` and DDS UDP socket creation returned `Operation not permitted`;
- controller spawners could not write `$HOME/.ros/locks`;
- RViz could not connect to a display;
- the validator timed out waiting for controller/state conditions.

This is recorded as an environment omission, not a passing smoke test. No CAN
interface was opened, and no real trajectory or effort command was published.

## Deferred physical and unrestricted-host checks

On an unrestricted Jazzy host, repeat:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/build.sh
./ros_ws/smoke_openarm_fake.sh
./ros_ws/pose_bringup.sh
robotctl r2s collect --group openarm_right_arm --dry-run
```

Only after fake-hardware verification should physical checks begin. Those checks
must cover CAN discovery, controller activation, E-stop readiness, low-speed
one-shot marker motion, continuous-follow error, gravity torque limits,
excitation bounds, artifact capture, and safe shutdown. `--real` and
`--execute` remain explicit gates.
