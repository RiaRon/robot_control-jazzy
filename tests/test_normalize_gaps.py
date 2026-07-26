"""A gap in a recording is a hole, not a straight line.

`/joint_states` is subscribed best-effort, so messages drop. `normalize_track`
interpolates between whatever samples it was given without looking at their
spacing, which turns a dropped run into a smooth curve through data nobody
measured — and a fit has no way to tell that curve from a measurement.
"""

import numpy as np
import pytest

from robot_control.track import TrackError, normalize_track


RATE = 100.0
PERIOD_NS = int(1e9 / RATE)


def _streams(measured_stamps):
    """A command at a steady rate, and a measurement on the stamps given."""
    span = max(measured_stamps)
    command_stamps = np.arange(0, span + PERIOD_NS, PERIOD_NS, dtype=np.int64)
    command = np.column_stack(
        [np.sin(command_stamps * 1e-9 * 2), np.cos(command_stamps * 1e-9 * 2)]
    )
    measured_stamps = np.asarray(measured_stamps, dtype=np.int64)
    measured = np.column_stack(
        [np.sin(measured_stamps * 1e-9 * 2), np.cos(measured_stamps * 1e-9 * 2)]
    )
    return command_stamps, command, measured_stamps, measured


def _normalize(measured_stamps, **kwargs):
    ct, c, mt, m = _streams(measured_stamps)
    return normalize_track(ct, c, mt, m, ["a", "b"], RATE, **kwargs)


def test_an_evenly_sampled_recording_normalizes():
    stamps = np.arange(0, 200) * PERIOD_NS

    track = _normalize(stamps)

    assert len(track.timestamps_ns) > 100


def test_ordinary_jitter_passes():
    """Real arrival times wobble; that is not a hole."""
    rng = np.random.default_rng(0)
    stamps = np.arange(0, 200) * PERIOD_NS
    stamps = stamps + rng.integers(-PERIOD_NS // 4, PERIOD_NS // 4, len(stamps))
    stamps = np.sort(stamps)

    track = _normalize(stamps)

    assert len(track.timestamps_ns) > 100


def test_a_dropped_run_is_refused_rather_than_drawn_through():
    stamps = np.concatenate(
        [np.arange(0, 50), np.arange(90, 200)]
    ) * PERIOD_NS

    with pytest.raises(TrackError, match="gap"):
        _normalize(stamps)


def test_the_refusal_says_how_long_the_gap_was_and_where():
    stamps = np.concatenate([np.arange(0, 50), np.arange(90, 200)]) * PERIOD_NS

    with pytest.raises(TrackError) as raised:
        _normalize(stamps)

    message = str(raised.value)
    assert "410" in message, message  # sample 49 to sample 90, at 10 ms each
    assert "0.49" in message, message  # and where it started


def test_the_tolerated_gap_is_configurable():
    """A slower state publisher is a different pipeline, not a broken one."""
    stamps = np.concatenate([np.arange(0, 50), np.arange(90, 200)]) * PERIOD_NS

    track = _normalize(stamps, max_gap_periods=50)

    assert len(track.timestamps_ns) > 100


def test_a_gap_is_judged_against_the_command_period_not_the_median():
    """A uniformly slow stream has a perfectly consistent median and is still
    unusable. Judging a gap against the stream's own spacing would pass it."""
    stamps = np.arange(0, 60) * (PERIOD_NS * 25)

    with pytest.raises(TrackError, match="gap"):
        _normalize(stamps)


def test_a_gap_in_the_command_stream_is_refused_too():
    """Our own publisher can stall; that leaves the same hole."""
    command_stamps = np.concatenate(
        [np.arange(0, 50), np.arange(90, 200)]
    ) * PERIOD_NS
    command = np.column_stack(
        [np.sin(command_stamps * 1e-9 * 2), np.cos(command_stamps * 1e-9 * 2)]
    )
    measured_stamps = np.arange(0, 200) * PERIOD_NS
    measured = np.column_stack(
        [np.sin(measured_stamps * 1e-9 * 2), np.cos(measured_stamps * 1e-9 * 2)]
    )

    with pytest.raises(TrackError, match="command"):
        normalize_track(
            command_stamps, command, measured_stamps, measured, ["a", "b"], RATE
        )
