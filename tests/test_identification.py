import numpy as np
import pytest

from robot_control.identification import (
    FitError,
    build_excitation,
    fit_second_order,
    split_repetitions,
    validate_holdout,
)


def test_excitation_contains_step_ramp_hold_and_multisine():
    time, command, phases = build_excitation(
        neutral=np.array([0.0, 0.2]), amplitude=np.array([0.2, 0.1]), rate_hz=100
    )

    assert command.shape == (len(time), 2)
    assert {"step", "ramp", "hold", "multisine"} <= set(phases)
    assert np.max(np.abs(command[:, 0])) <= 0.2 + 1e-12


def test_three_repetitions_split_two_fit_one_holdout():
    assert split_repetitions(["run-1", "run-2", "run-3"]) == (
        ("run-1", "run-2"),
        ("run-3",),
    )
    with pytest.raises(FitError, match="exactly three"):
        split_repetitions(["run-1", "run-2"])


def test_second_order_fit_recovers_synthetic_stiffness_and_damping():
    rate = 500.0
    dt = 1.0 / rate
    time, command, _ = build_excitation(
        neutral=np.array([0.0]), amplitude=np.array([0.2]), rate_hz=rate
    )
    stiffness, damping = 35.0, 4.0
    measured = np.zeros_like(command)
    velocity = np.zeros(1)
    for index in range(1, len(time)):
        acceleration = stiffness * (command[index - 1] - measured[index - 1]) - damping * velocity
        velocity = velocity + acceleration * dt
        measured[index] = measured[index - 1] + velocity * dt

    estimate = fit_second_order(time, command, measured)

    assert abs(estimate.stiffness[0] - stiffness) / stiffness < 0.1
    assert abs(estimate.damping[0] - damping) / damping < 0.1


def test_holdout_gates_and_model_inadequate_status():
    assert validate_holdout(
        openarm_rmse_rad=0.02,
        tesollo_rmse_rad=0.04,
        delay_residual_sec=0.009,
        command_period_sec=0.01,
        improvement_fraction=0.31,
    ).status == "validated"

    failed = validate_holdout(
        openarm_rmse_rad=0.02,
        tesollo_rmse_rad=0.06,
        delay_residual_sec=0.009,
        command_period_sec=0.01,
        improvement_fraction=0.31,
    )
    assert failed.status == "model_inadequate"
    assert "tesollo_rmse" in failed.failures
