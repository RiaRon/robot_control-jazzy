# robot_control

This directory owns canonical lower-level robot contracts and the Real2Sim
artifact pipeline. `sim2real` remains responsible for policy execution and
task orchestration; `hdgp` remains responsible for robot assets and RL.

This Git branch targets Ubuntu 22.04 and ROS 2 Humble only. It contains
validated OpenArm and Tesollo Humble driver snapshots directly under
`ros_ws/src`. Jazzy is maintained on a separate long-lived branch; do not
merge the two distribution branches wholesale.

The first complete profile is `openarm_tesollo`. RH56F1 and the simple gripper
currently provide static component contracts only.

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -e '.[test,hdf5]'
robotctl r2s preflight
robotctl r2s collect --dry-run
```

Install ROS dependencies and build the imported drivers with:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros_ws/src --ignore-src -r -y
./ros_ws/build.sh
source ros_ws/install/setup.bash
```

The wrapper rejects non-Humble environments and keeps all generated products
inside `ros_ws/{build,install,log}`.

No command is published unless `--execute` is explicit. A ROS adapter must
also be installed for execution; the core CLI deliberately fails without one.

Calibration JSON v1 is read-only compatibility input. All newly exported
bundles are schema v2, checksum protected, and tied to a profile and asset
manifest hash.

## Handing a calibration to hdgp

`hdgp` reads schema v1 and one scalar per actuator group, so `r2s export` can
write that form beside the bundle:

```bash
robotctl r2s export --bundle bundle.json --validation verdict.json \
    --output exported.json --hdgp real2sim_actuator.json
OPENARM_REAL2SIM_ACTUATOR_CALIBRATION=$PWD/real2sim_actuator.json ./train.sh ...
```

Two things about that conversion are worth knowing before trusting a run.
`get_actuator_params` answers a group name it does not recognise with the
env's own default and reports nothing, so the group each profile group lands
in is declared as `hdgp_group` in the profile rather than guessed; a measured
group without one is refused. And collapsing a group's joints to one scalar is
refused when they disagree by more than `--hdgp-max-spread` of their mean —
the arm's real gains run kp 70 / 60 / 10, where the average describes no joint
in the group. Groups with no measurement are left out, so the env keeps its own
gain; the export names them.
