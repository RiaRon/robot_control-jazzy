# robot_control

This directory owns canonical lower-level robot contracts and the Real2Sim
artifact pipeline. `sim2real` remains responsible for policy execution and
task orchestration; `hdgp` remains responsible for robot assets and RL.

This Git branch targets Ubuntu 24.04 and ROS 2 Jazzy only. It contains
validated OpenArm and Tesollo Jazzy driver snapshots directly under
`ros_ws/src`. Humble is maintained on a separate long-lived branch; do not
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
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros_ws/src --ignore-src -r -y
./ros_ws/build.sh
source ros_ws/install/setup.bash
```

The wrapper rejects non-Jazzy environments and keeps all generated products
inside `ros_ws/{build,install,log}`.

No command is published unless `--execute` is explicit. A ROS adapter must
also be installed for execution; the core CLI deliberately fails without one.

Calibration JSON v1 is read-only compatibility input. All newly exported
bundles are schema v2, checksum protected, and tied to a profile and asset
manifest hash.
