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

Install the Jazzy dependencies and build the supported OpenArm/DG5F graph with:

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/install_dependencies_jazzy.sh
./ros_ws/build.sh
source ros_ws/install/setup.bash
```

`install_dependencies_jazzy.sh` deliberately prompts through `sudo`; run it
only when you are ready to grant the operator-controlled package changes. The
build wrapper rejects non-Jazzy environments, builds only the supported
OpenArm/DG5F leaves, and keeps all generated products inside
`ros_ws/{build,install,log}`.

No command is published unless `--execute` is explicit. Execution also needs
the ROS adapter, which requires `rclpy`; without it the CLI fails with a named
error rather than crashing on import.

## Setting a pose

Bring the arms up on fake hardware, drag the end-effector marker in RViz to
find a pose, then commit it. Every command is listed in [docs/cli.md](docs/cli.md).

```bash
source /opt/ros/jazzy/setup.bash
./ros_ws/pose_bringup.sh                     # fake hardware; nothing touches CAN
```

In RViz, open the **MotionPlanning** panel and drag the interactive marker on
`openarm_right_hand`. That moves the goal state only; the robot does not
follow it until a command is executed. Read the pose back and commit it from a
second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH="src:.:$PYTHONPATH"       # append; assigning hides rclpy

robotctl pose show  --group openarm_right_arm
robotctl pose ee    --group openarm_right_arm --relative --xyz 0,0,0.03
robotctl pose ee    --group openarm_right_arm --relative --xyz 0,0,0.03 --execute
robotctl pose joints --group openarm_right_arm --named home --execute
```

`pose_bringup.sh` inverts the vendor default: `demo.launch.py` defaults
`use_fake_hardware` to false and opens `can0` and `can1`, while the wrapper
uses fake hardware unless given `--real` together with `--right-can` and
`--left-can`.

Calibration JSON v1 is read-only compatibility input. All newly exported
bundles are schema v2, checksum protected, and tied to a profile and asset
manifest hash.
