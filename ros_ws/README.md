# ROS 2 Jazzy workspace

This Git branch owns only ROS 2 Jazzy sources. Validated OpenArm and Tesollo
driver snapshots live directly under `src/`; no `vcs import` is required.
`supported-packages.txt` defines the supported OpenArm/DG5F leaf packages.
`build.sh` builds those leaves and their dependencies into branch-local
`build`, `install`, and `log` products. Before building, source Jazzy and run
`./ros_ws/install_dependencies_jazzy.sh`; its `sudo` prompts are deliberately
operator-controlled.

Humble is maintained on a separate long-lived Git branch. Never merge the
Jazzy and Humble branches wholesale; transfer distribution-independent core
changes selectively.

Canonical profile topics are rooted at
`/robot_control/openarm_tesollo/{command,state}`. Vendor adapters translate at
that boundary. Hardware publishing remains behind the explicit
`robotctl r2s collect --execute` gate.
