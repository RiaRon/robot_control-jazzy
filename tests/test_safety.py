import numpy as np
import pytest

from robot_control.safety import CommandGate, SafetyError


def test_publish_is_blocked_without_execute():
    gate = CommandGate(
        execute=False,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([2.0]),
        command_period_sec=0.1,
    )
    with pytest.raises(SafetyError, match="--execute"):
        gate.authorize(np.array([0.0]), now_sec=0.0)


def test_command_gate_enforces_position_and_velocity_and_watchdog():
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([2.0]),
        command_period_sec=0.1,
        watchdog_sec=0.25,
    )
    assert gate.authorize(np.array([0.0]), now_sec=0.0).tolist() == [0.0]
    with pytest.raises(SafetyError, match="velocity"):
        gate.authorize(np.array([0.5]), now_sec=0.1)
    with pytest.raises(SafetyError, match="position"):
        gate.authorize(np.array([1.1]), now_sec=0.2)
    with pytest.raises(SafetyError, match="watchdog"):
        gate.authorize(np.array([0.0]), now_sec=0.3)


def test_estop_returns_last_safe_pose_hold():
    gate = CommandGate(
        execute=True,
        lower=np.array([-1.0]),
        upper=np.array([1.0]),
        velocity=np.array([10.0]),
        command_period_sec=0.1,
    )
    gate.authorize(np.array([0.25]), now_sec=0.0)
    gate.estop()
    assert gate.hold_pose().tolist() == [0.25]
    with pytest.raises(SafetyError, match="E-stop"):
        gate.authorize(np.array([0.0]), now_sec=0.1)
