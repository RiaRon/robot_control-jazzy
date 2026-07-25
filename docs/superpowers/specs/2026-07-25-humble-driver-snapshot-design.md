# Humble Driver Snapshot Integration Design

## Objective

Make `robot_control` the owner of the validated ROS 2 Humble OpenArm and
Tesollo driver sources. The existing repositories remain untouched as rollback
references. Jazzy will be maintained later on a separate long-lived branch.

## Source Revisions

- OpenArm: `https://github.com/enactic/openarm_ros2.git`
  at `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`
- Tesollo: `https://github.com/tesollodelto/delto_m_ros2.git`
  Humble branch at `a68335919ee490d5293581574acc7aff12fe969d`

The imported trees exclude source-repository Git metadata, upstream CI
configuration, caches, and build products. Source licenses and copyright
headers remain intact.

## Repository and Branch Model

Initialize `robot_control` as a Git repository on the long-lived `humble`
branch. The branch contains Humble driver sources only. A future `jazzy`
branch will replace the driver snapshots and ROS-specific configuration with
validated Jazzy revisions while retaining compatible core changes through
selective cherry-picks.

Humble and Jazzy branches must not be merged wholesale.

## Layout

```text
robot_control/
├── components/
├── docs/
├── ros_ws/
│   ├── src/
│   │   ├── openarm_ros2/
│   │   └── delto_m_ros2/
│   ├── build.sh
│   └── README.md
├── src/robot_control/
├── tests/
└── vendor_metadata/
    ├── openarm/UPSTREAM.md
    └── tesollo/UPSTREAM.md
```

The branch removes the previous `ros_ws/humble` and `ros_ws/jazzy` dual
overlay layout. `ros_ws/build.sh` rejects any environment whose
`ROS_DISTRO` is not `humble` and uses branch-local build, install, and log
directories.

## Ownership and Provenance

Each `UPSTREAM.md` records the upstream URL, branch, exact commit, import date,
license, excluded files, and local modifications. Snapshot updates are
performed intentionally by replacing the imported tree and reviewing the
resulting diff; they do not silently track an upstream branch.

The top-level `.gitignore` excludes:

- Python bytecode, test caches, virtual environments, and packaging products
- ROS `build`, `install`, and `log` products
- imported-workspace temporary directories
- raw rosbag, HDF5, fit, and exported calibration artifacts

## Integration Boundaries

The imported drivers retain their vendor topics and hardware plugins.
`robot_control` canonical profiles, safety gates, and Real2Sim code remain
outside vendor trees. Adapters translate canonical command/state contracts at
the boundary so future upstream snapshot replacement does not overwrite
project-owned control logic.

No source in `robot_control` may import Python modules through absolute
workspace paths. The HDGP asset manifest reference remains an explicit sibling
repository contract because deployment environments guarantee that `hdgp` is
cloned beside `robot_control`.

## Verification

Completion requires:

1. Imported files match the two source commits after applying the documented
   exclusion policy.
2. Original source repositories remain unchanged.
3. `robot_control` core tests and fake-hardware canonical round-trip pass.
4. Python and shell syntax checks pass.
5. `colcon build` passes in ROS 2 Humble when all system dependencies are
   available.
6. Build output is isolated under `robot_control/ros_ws`.

No real-hardware command is issued as part of this migration.
