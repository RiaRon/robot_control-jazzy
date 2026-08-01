"""Bounded end-to-end pose-setting smoke check.

Exercises the operator path rather than the library: it measures the
end-effector, runs the real `robotctl pose ee --execute`, and measures again.
A library-only check would pass even if the CLI wiring were broken.

ROS imports stay inside main so importing this module needs no ROS.
"""

from collections.abc import Sequence
import subprocess
import sys
import time


def _read_pose(profile, group: str, timeout_sec: float):
    """Return the group's end-effector pose, waiting for the stack to come up."""
    from robot_control.ros_adapter import AdapterUnavailable, IkFailed, RosAdapter

    deadline = time.monotonic() + timeout_sec
    last_error: Exception | None = None
    while True:
        try:
            with RosAdapter(profile, group) as adapter:
                return adapter.read_pose(timeout_sec=5.0)
        except (AdapterUnavailable, IkFailed) as error:
            last_error = error
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"the stack did not become usable within {timeout_sec} s: "
                    f"{last_error}"
                ) from error
            time.sleep(1.0)


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--group", default="openarm_right_arm")
    parser.add_argument("--offset-z", type=float, default=0.03)
    parser.add_argument("--tolerance", type=float, default=0.005)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--duration", type=float, default=3.0)
    arguments = parser.parse_args(argv)

    if not 0 < abs(arguments.offset_z) <= 0.05:
        parser.exit(1, "pose smoke: the offset must stay within 5 cm\n")

    from robot_control.profile import load_builtin_profile

    profile = load_builtin_profile("openarm_tesollo")
    try:
        before = _read_pose(profile, arguments.group, arguments.timeout)
    except TimeoutError as error:
        parser.exit(1, f"pose smoke: {error}\n")
    print(f"pose smoke: before z={before.position[2]:+.4f}", flush=True)

    command = [
        sys.executable,
        "-m",
        "robot_control.cli",
        "pose",
        "ee",
        "--group",
        arguments.group,
        "--relative",
        "--xyz",
        f"0,0,{arguments.offset_z}",
        "--duration",
        str(arguments.duration),
        "--execute",
    ]
    result = subprocess.run(command, text=True, capture_output=True)
    print(result.stdout, end="", flush=True)
    if result.returncode != 0:
        parser.exit(
            1,
            f"pose smoke: robotctl pose ee exited {result.returncode}\n"
            f"{result.stderr}",
        )

    after = _read_pose(profile, arguments.group, timeout_sec=10.0)
    moved = after.position[2] - before.position[2]
    print(f"pose smoke: after z={after.position[2]:+.4f} moved {moved:+.4f}", flush=True)
    if abs(moved - arguments.offset_z) > arguments.tolerance:
        parser.exit(
            1,
            f"pose smoke: the end effector moved {moved:+.4f} m, not "
            f"{arguments.offset_z:+.4f} m within {arguments.tolerance} m\n",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
