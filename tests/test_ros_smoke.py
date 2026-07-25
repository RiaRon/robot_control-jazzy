import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

from tools.ros_smoke import smoke_plan, validate_joint_state, wait_for


ROOT = Path(__file__).parents[1]
DG5F_JOINTS = (
    "rj_dg_1_1",
    "rj_dg_1_2",
    "rj_dg_1_3",
    "rj_dg_1_4",
    "rj_dg_2_1",
    "rj_dg_2_2",
    "rj_dg_2_3",
    "rj_dg_2_4",
    "rj_dg_3_1",
    "rj_dg_3_2",
    "rj_dg_3_3",
    "rj_dg_3_4",
    "rj_dg_4_1",
    "rj_dg_4_2",
    "rj_dg_4_3",
    "rj_dg_4_4",
    "rj_dg_5_1",
    "rj_dg_5_2",
    "rj_dg_5_3",
    "rj_dg_5_4",
)
DG5F_TARGET = dict.fromkeys(DG5F_JOINTS, 0.05)
SMOKE_SCRIPTS = (
    ROOT / "ros_ws/smoke_openarm_fake.sh",
    ROOT / "ros_ws/smoke_dg5f_fake.sh",
    ROOT / "ros_ws/smoke_dg5f_gazebo.sh",
)
SMOKE_HARNESS = ROOT / "ros_ws/smoke_harness.sh"


def test_wait_for_returns_after_predicate_succeeds():
    attempts = 0

    def succeeds_on_third_attempt() -> bool:
        nonlocal attempts
        attempts += 1
        return attempts == 3

    wait_for(succeeds_on_third_attempt, timeout_s=0.1, interval_s=0)

    assert attempts == 3


def test_wait_for_raises_at_deadline():
    with pytest.raises(TimeoutError):
        wait_for(lambda: False, timeout_s=0, interval_s=0)


def test_validate_joint_state_accepts_exact_finite_target():
    validate_joint_state(
        DG5F_TARGET,
        DG5F_JOINTS,
        [0.05] * 20,
        tolerance=0.01,
    )


def test_validate_joint_state_rejects_missing_joint():
    with pytest.raises(ValueError, match="missing"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS[:-1],
            [0.05] * 19,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_extra_joint():
    with pytest.raises(ValueError, match="extra"):
        validate_joint_state(
            DG5F_TARGET,
            (*DG5F_JOINTS, "unexpected_joint"),
            [0.05] * 21,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_name_position_length_mismatch():
    with pytest.raises(ValueError, match="length"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            [0.05] * 19,
            tolerance=0.01,
        )


@pytest.mark.parametrize("bad_position", [math.nan, math.inf, -math.inf])
def test_validate_joint_state_rejects_non_finite_position(bad_position):
    positions = [0.05] * 20
    positions[7] = bad_position

    with pytest.raises(ValueError, match="non-finite"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            positions,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_position_error_above_tolerance():
    positions = [0.05] * 20
    positions[12] = 0.071

    with pytest.raises(ValueError, match="position error"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            positions,
            tolerance=0.02,
        )


def test_validate_joint_state_rejects_duplicate_joint_name():
    duplicate_names = (*DG5F_JOINTS[:-1], DG5F_JOINTS[0])

    with pytest.raises(ValueError, match="duplicate"):
        validate_joint_state(
            DG5F_TARGET,
            duplicate_names,
            [0.05] * 20,
            tolerance=0.01,
        )


def test_validate_joint_state_rejects_non_finite_target():
    targets = dict(DG5F_TARGET)
    targets[DG5F_JOINTS[0]] = math.nan

    with pytest.raises(ValueError, match="target"):
        validate_joint_state(
            targets,
            DG5F_JOINTS,
            [0.05] * 20,
            tolerance=0.01,
        )


@pytest.mark.parametrize("tolerance", [-0.01, math.nan, math.inf])
def test_validate_joint_state_rejects_invalid_tolerance(tolerance):
    with pytest.raises(ValueError, match="tolerance"):
        validate_joint_state(
            DG5F_TARGET,
            DG5F_JOINTS,
            [0.05] * 20,
            tolerance=tolerance,
        )


@pytest.mark.parametrize("robot", ["openarm", "dg5f"])
def test_smoke_plan_never_commands_more_than_point_zero_five_radians(robot):
    plan = smoke_plan(robot)

    assert plan.command_targets
    assert max(abs(position) for position in plan.command_targets.values()) <= 0.05
    assert plan.command_targets.keys() <= plan.state_targets.keys()


@pytest.mark.parametrize("script", SMOKE_SCRIPTS)
def test_smoke_script_rejects_non_jazzy_before_starting_ros(script):
    result = subprocess.run(
        ["bash", str(script)],
        env=dict(os.environ, ROS_DISTRO="humble"),
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "ROS 2 Jazzy" in result.stderr


@pytest.mark.parametrize(
    ("sent_signal", "expected_returncode"),
    [
        (signal.SIGINT, 130),
        (signal.SIGTERM, 143),
        (signal.SIGHUP, 129),
    ],
)
def test_signal_promptly_terminates_validator_and_launch_process_groups(
    tmp_path,
    sent_signal,
    expected_returncode,
):
    workspace = tmp_path / "ros_ws"
    workspace.mkdir()
    (workspace / "install").mkdir()
    (workspace / "install/setup.bash").write_text("")
    shutil.copy2(ROOT / "ros_ws/smoke_openarm_fake.sh", workspace)
    shutil.copy2(SMOKE_HARNESS, workspace)

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    process_script = (
        "#!/usr/bin/env bash\n"
        "set -eu\n"
        'kind="${1:-unknown}"\n'
        'pid_file_var="${kind^^}_PID_FILE"\n'
        'stopped_file_var="${kind^^}_STOPPED_FILE"\n'
        'pid_file="${!pid_file_var}"\n'
        'stopped_file="${!stopped_file_var}"\n'
        'printf "%s\\n" "$$" > "$pid_file"\n'
        'trap \'printf "stopped\\n" > "$stopped_file"; exit 0\' TERM INT HUP\n'
        "while true; do sleep 0.1; done\n"
    )
    fake_process = fake_bin / "fake-process"
    fake_process.write_text(process_script)
    fake_process.chmod(0o755)

    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        f"exec {fake_process} launch\n"
    )
    fake_ros2.chmod(0o755)
    fake_python = fake_bin / "python3"
    fake_python.write_text(
        "#!/usr/bin/env bash\n"
        f"exec {fake_process} validator\n"
    )
    fake_python.chmod(0o755)

    launch_pid_file = tmp_path / "launch.pid"
    validator_pid_file = tmp_path / "validator.pid"
    launch_stopped = tmp_path / "launch.stopped"
    validator_stopped = tmp_path / "validator.stopped"
    environment = dict(
        os.environ,
        ROS_DISTRO="jazzy",
        PATH=f"{fake_bin}:{os.environ['PATH']}",
        LAUNCH_PID_FILE=str(launch_pid_file),
        VALIDATOR_PID_FILE=str(validator_pid_file),
        LAUNCH_STOPPED_FILE=str(launch_stopped),
        VALIDATOR_STOPPED_FILE=str(validator_stopped),
    )
    process = subprocess.Popen(
        ["bash", str(workspace / "smoke_openarm_fake.sh")],
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )

    def wait_until(predicate, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError("synthetic smoke process did not reach state")
            time.sleep(0.01)

    child_pids = []
    try:
        wait_until(lambda: launch_pid_file.exists() and validator_pid_file.exists())
        child_pids = [
            int(launch_pid_file.read_text()),
            int(validator_pid_file.read_text()),
        ]

        signaled_at = time.monotonic()
        process.send_signal(sent_signal)
        returncode = process.wait(timeout=2)
        elapsed = time.monotonic() - signaled_at

        wait_until(lambda: launch_stopped.exists() and validator_stopped.exists())
        assert returncode == expected_returncode
        assert elapsed < 2
        for child_pid in child_pids:
            with pytest.raises(ProcessLookupError):
                os.kill(child_pid, 0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)
        for child_pid in child_pids:
            try:
                os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
