"""A recorded stream, kept as it arrived.

Deliberately not a CanonicalTrack. That type is the *output* of normalize, on a
uniform grid; this one is what a subscription actually received, at whatever rate
and with whatever holes. The arrival pattern is data about the pipeline being
identified, so resampling it here would throw away the evidence.
"""

import numpy as np
import pytest

from robot_control.track import Recording, TrackError


def _recording(stamps, values=None, **kwargs):
    stamps = np.asarray(stamps, dtype=np.int64)
    if values is None:
        values = np.column_stack([np.arange(len(stamps), dtype=float)] * 2)
    return Recording(stamps, np.asarray(values, dtype=float), ("a", "b"), **kwargs)


def test_a_recording_keeps_what_arrived():
    stamps = [0, 10_000_000, 20_000_000]

    recording = _recording(stamps)

    np.testing.assert_array_equal(recording.timestamps_ns, stamps)
    assert recording.joint_names == ("a", "b")
    assert len(recording) == 3


def test_the_largest_gap_is_reported():
    """Best-effort QoS drops messages; a hole has to be visible as a hole."""
    recording = _recording([0, 10_000_000, 210_000_000, 220_000_000])

    assert recording.largest_gap_ns == 200_000_000
    assert recording.median_period_ns == 10_000_000


def test_a_recording_with_one_sample_has_no_gap_to_report():
    recording = _recording([5])

    assert recording.largest_gap_ns == 0
    assert recording.median_period_ns == 0


def test_out_of_order_arrival_is_visible_rather_than_sorted_away():
    """Reordering says something about the transport, so it is not hidden."""
    recording = _recording([0, 20_000_000, 10_000_000])

    assert not recording.is_monotonic


def test_an_ordinary_recording_is_monotonic():
    assert _recording([0, 10_000_000, 20_000_000]).is_monotonic


def test_messages_that_did_not_cover_the_joints_are_counted():
    recording = _recording([0, 10_000_000], incomplete=3)

    assert recording.incomplete == 3


def test_a_recording_of_the_wrong_width_is_refused():
    with pytest.raises(TrackError, match="joint"):
        Recording(
            np.array([0, 1], dtype=np.int64),
            np.zeros((2, 3)),
            ("a", "b"),
        )


def test_a_recording_whose_counts_disagree_is_refused():
    with pytest.raises(TrackError, match="sample"):
        Recording(
            np.array([0, 1, 2], dtype=np.int64),
            np.zeros((2, 2)),
            ("a", "b"),
        )


def test_a_recording_with_a_non_finite_value_is_refused():
    values = np.zeros((2, 2))
    values[1, 1] = np.nan

    with pytest.raises(TrackError, match="finite"):
        _recording([0, 1], values)


def test_an_empty_recording_is_refused():
    """Nothing arrived, which is a failed run rather than a short one."""
    with pytest.raises(TrackError, match="empty"):
        Recording(
            np.zeros(0, dtype=np.int64), np.zeros((0, 2)), ("a", "b")
        )
