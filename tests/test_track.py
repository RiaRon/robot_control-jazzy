import numpy as np
import pytest

from robot_control.track import TrackError, normalize_track


def test_normalize_resamples_command_and_measured_separately():
    result = normalize_track(
        command_time_ns=np.array([0, 20_000_000, 40_000_000]),
        command=np.array([[0.0], [2.0], [4.0]]),
        measured_time_ns=np.array([0, 10_000_000, 30_000_000, 40_000_000]),
        measured=np.array([[0.0], [0.5], [2.5], [4.0]]),
        joint_names=["j1"],
        rate_hz=100,
    )

    assert result.timestamps_ns.tolist() == [
        0,
        10_000_000,
        20_000_000,
        30_000_000,
        40_000_000,
    ]
    assert result.command[:, 0].tolist() == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert result.measured[:, 0].tolist() == [0.0, 0.5, 1.5, 2.5, 4.0]


def test_normalize_rejects_non_monotonic_timestamps():
    with pytest.raises(TrackError, match="strictly increasing"):
        normalize_track(
            np.array([0, 0]),
            np.zeros((2, 1)),
            np.array([0, 1]),
            np.zeros((2, 1)),
            ["j1"],
            100,
        )


def test_normalize_rejects_insufficient_excitation():
    with pytest.raises(TrackError, match="insufficient excitation"):
        normalize_track(
            np.array([0, 10_000_000]),
            np.array([[0.0], [0.00001]]),
            np.array([0, 10_000_000]),
            np.zeros((2, 1)),
            ["j1"],
            100,
            minimum_range_rad=0.001,
        )
