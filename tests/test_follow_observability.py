import numpy as np

from robot_control.follow_observability import (
    ConvergenceObservation,
    HandoffConvergenceGate,
    scalar_statistics,
    window_statistics,
)


def observation(**overrides):
    values = {
        "marker_position_error_m": 0.001,
        "marker_orientation_error_rad": 0.01,
        "ik_to_command_max_error_rad": 0.01,
        "command_to_measured_max_error_rad": 0.01,
        "measured_sample_delta_max_rad": 0.0002,
    }
    values.update(overrides)
    return ConvergenceObservation(**values)


def test_convergence_gate_requires_a_continuous_stable_window():
    gate = HandoffConvergenceGate(
        started_sec=10.0, stable_window_sec=0.5, timeout_sec=2.0
    )
    assert gate.update(10.0, observation()) == (False, False)
    assert gate.update(10.3, observation()) == (False, False)
    assert gate.update(
        10.4, observation(marker_position_error_m=0.006)
    ) == (False, False)
    assert gate.update(10.9, observation()) == (False, False)
    assert gate.update(11.4, observation()) == (True, False)


def test_convergence_gate_times_out_without_starting_profile():
    gate = HandoffConvergenceGate(
        started_sec=1.0, stable_window_sec=0.5, timeout_sec=1.0
    )
    assert gate.update(
        1.5, observation(command_to_measured_max_error_rad=0.061)
    ) == (False, False)
    assert gate.update(
        2.0, observation(command_to_measured_max_error_rad=0.061)
    ) == (False, True)
    assert gate.last.command_to_measured_max_error_rad == 0.061


def _sample(stage, value, timestamp):
    target = np.full(2, value)
    measured = np.zeros(2)
    return {
        "stage": stage,
        "timestamp_sec": timestamp,
        "position_error_m": {"live_marker_to_measured": value},
        "orientation_error_rad": {"live_marker_to_measured": value / 10},
        "joint_positions_rad": {
            "ik_target": target.tolist(), "measured": measured.tolist()
        },
        "limits": {
            "cartesian_speed": stage == "startup_alignment",
            "cartesian_angular_speed": False,
            "joint": {},
        },
    }


def test_profile_only_statistics_never_include_startup_worst():
    trace = [
        _sample("startup_alignment", 0.048466, 0.0),
        _sample("profile_ramp", 0.010, 1.0),
        _sample("profile_hold", 0.016842, 2.0),
    ]
    stats = window_statistics(trace, joint_tolerance_rad=0.05)
    assert stats["comparison_default"] == "profile_only"
    assert stats["windows"]["overall_run"]["tcp_translation_m"]["max"] == 0.048466
    assert stats["windows"]["profile_only"]["tcp_translation_m"]["max"] == 0.016842


def test_scalar_statistics_uses_interpolated_p95_and_final_residual():
    stats = scalar_statistics([1.0, 2.0, 3.0])
    assert stats["p95"] == 2.9
    assert stats["final_residual"] == 3.0


def test_ready_reacquisition_joint_statistics_use_recorded_command_not_fake_ik():
    stage_trace = [
        {
            "stage": "ready_reacquisition",
            "timestamp_sec": 0.1,
            "joint_positions_rad": {
                "ik_target": None,
                "command": [0.2, 0.4],
                "measured": [0.1, 0.3],
            },
        }
    ]
    stats = window_statistics(
        [], joint_tolerance_rad=0.05, stage_trace=stage_trace
    )["windows"]["ready_reacquisition"]
    assert stats["samples"] == 1
    assert stats["tcp_samples"] == 0
    assert stats["joint_error_basis"] == "command_to_measured"
    assert stats["joints"][0]["max_abs_rad"] == 0.1
