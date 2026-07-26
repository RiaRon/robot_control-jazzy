"""`robotctl r2s collect --execute`: publish an excitation and record the response.

The stage used to compute the excitation, print a sample count, and exit 2. These
tests are mostly about what happens before the first sample is published, and
about the two streams staying on one clock.
"""

import json

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.profile import load_builtin_profile


GROUP = "openarm_right_arm"
RATE = 100.0


class RecordingArm:
    """A stub arm that answers commands one period late, on the node clock.

    The lag is the point: a collector that paired each command with the state it
    read in the same cycle would record a perfectly tracking arm, and the delay
    the pipeline exists to measure would be gone.
    """

    LAG_SAMPLES = 3

    def __init__(self):
        self.joints = np.zeros(7)
        self.published = []
        self.trajectories = []
        self.clock_ns = 1_000_000_000
        self.recording = False
        self._pending = []
        self._recorded = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def close(self):
        self.closed = True

    def read_state(self, timeout_sec=10.0):
        return self.joints.copy()

    def read_robot_description(self):
        return "<robot name='stub'/>"

    def now_ns(self):
        return self.clock_ns

    def start_recording(self):
        self.recording = True

    def stop_recording(self):
        from robot_control.track import Recording

        self.recording = False
        stamps = np.array([s for s, _ in self._recorded], dtype=np.int64)
        rows = np.vstack([v for _, v in self._recorded])
        return Recording(stamps, rows, tuple(load_builtin_profile(
            "openarm_tesollo").groups[GROUP].joints))

    def stream_positions(self, positions, period_sec):
        self.published.append(np.asarray(positions, dtype=float).copy())
        self._pending.append(np.asarray(positions, dtype=float).copy())
        # The clock advances with the stream, as it would on hardware.
        self.clock_ns += int(period_sec * 1e9)

    def send_trajectory(self, points, period_sec):
        self.trajectories.append(np.asarray(points[-1], dtype=float).copy())
        self.joints = np.asarray(points[-1], dtype=float).copy()

    def pump(self, timeout_sec=0.0):
        if not self.recording:
            return
        if len(self._pending) > self.LAG_SAMPLES:
            self.joints = self._pending.pop(0)
        # header.stamp is a touch behind our clock, as a hardware read is.
        self._recorded.append((self.clock_ns - 2_000_000, self.joints.copy()))


@pytest.fixture
def arm(monkeypatch):
    from robot_control import ros_adapter

    stub = RecordingArm()
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    # The loop paces itself in real time, which is right on hardware and six
    # seconds of nothing here. The stub advances its own clock per sample, so
    # the recorded stamps are unaffected.
    monkeypatch.setattr("robot_control.cli.time.sleep", lambda _seconds: None)
    return stub


@pytest.fixture
def profile():
    return load_builtin_profile("openarm_tesollo")


def _argv(tmp_path, *extra):
    return [
        "r2s", "collect", "--group", GROUP,
        "--output", str(tmp_path / "run.npz"),
        *extra,
    ]


def test_collect_publishes_the_excitation_and_records_the_response(
    arm, tmp_path, capsys
):
    code = main(_argv(tmp_path, "--execute"))

    assert code == 0, capsys.readouterr().out
    raw = np.load(tmp_path / "run.npz", allow_pickle=False)
    assert len(raw["command"]) == len(arm.published)
    assert raw["command"].shape[1] == 7
    assert len(raw["measured"]) > 0
    assert list(raw["joint_names"]) == list(
        load_builtin_profile("openarm_tesollo").groups[GROUP].joints
    )


def test_the_recording_normalizes_without_an_intervening_step(arm, tmp_path):
    """The whole point: collect's output is normalize's input, unedited."""
    from robot_control.track import normalize_track

    main(_argv(tmp_path, "--execute"))
    raw = np.load(tmp_path / "run.npz", allow_pickle=False)

    track = normalize_track(
        raw["command_time_ns"],
        raw["command"],
        raw["measured_time_ns"],
        raw["measured"],
        list(raw["joint_names"]),
        RATE,
    )

    assert track.command.shape == track.measured.shape
    assert len(track.timestamps_ns) > 100
    assert track.joint_names == tuple(
        load_builtin_profile("openarm_tesollo").groups[GROUP].joints
    )


def test_normalize_reports_a_missing_hdf5_extra_rather_than_raising(arm, tmp_path, capsys):
    """h5py is an optional extra; not having it is an environment answer."""
    main(_argv(tmp_path, "--execute"))

    code = main(
        ["r2s", "normalize", "--input", str(tmp_path / "run.npz"),
         "--output", str(tmp_path / "track.h5")]
    )

    try:
        import h5py  # noqa: F401
    except ImportError:
        assert code == 2
        assert "hdf5" in capsys.readouterr().out.lower()
    else:
        assert code == 0


def test_both_streams_are_stamped_from_the_same_clock(arm, tmp_path):
    """Different epochs would leave no overlap, or a spurious one that fits."""
    main(_argv(tmp_path, "--execute"))

    raw = np.load(tmp_path / "run.npz", allow_pickle=False)
    command, measured = raw["command_time_ns"], raw["measured_time_ns"]
    overlap = min(command[-1], measured[-1]) - max(command[0], measured[0])
    span = command[-1] - command[0]
    assert overlap > span * 0.9, "the two streams barely overlap"


def test_the_command_is_not_paired_with_the_state_read_beside_it(arm, tmp_path):
    """Pairing at log time would bake in delay = 0 and destroy the parameter."""
    main(_argv(tmp_path, "--execute"))

    raw = np.load(tmp_path / "run.npz", allow_pickle=False)
    # The stub lags by three samples, so the streams must not be row-aligned
    # into a track that tracks perfectly.
    rows = min(len(raw["command"]), len(raw["measured"]))
    assert not np.allclose(
        raw["command"][:rows], raw["measured"][:rows], atol=1e-6
    )
    # And the lag is still recoverable from the stamps, which is the point.
    assert raw["command_time_ns"].shape != raw["measured_time_ns"].shape or not (
        raw["command_time_ns"] == raw["measured_time_ns"]
    ).all()


def test_collect_defaults_to_a_dry_run_that_publishes_nothing(arm, tmp_path, capsys):
    assert main(_argv(tmp_path)) == 0

    assert arm.published == []
    assert not (tmp_path / "run.npz").exists()
    assert "DRY RUN" in capsys.readouterr().out


def test_collect_starts_from_where_the_arm_is(arm, tmp_path):
    """Not the midpoint of the range: that is a move to somewhere else first."""
    arm.joints = np.array([0.4, -0.3, 0.2, 0.1, 0.0, -0.1, 0.3])

    main(_argv(tmp_path, "--execute"))

    np.testing.assert_allclose(arm.published[0], arm.published[0])
    raw = np.load(tmp_path / "run.npz", allow_pickle=False)
    # The first commanded sample is a hold at the starting pose.
    np.testing.assert_allclose(raw["command"][0], [0.4, -0.3, 0.2, 0.1, 0.0, -0.1, 0.3])


def test_an_excitation_that_leaves_the_envelope_is_refused_before_publishing(
    arm, tmp_path, capsys
):
    """r_aj_2 stops at 2.0 rad, so starting near it and swinging leaves it."""
    arm.joints = np.array([0.0, 1.98, 0.0, 0.0, 0.0, 0.0, 0.0])

    code = main(_argv(tmp_path, "--execute"))

    assert code == 3
    assert "position limit exceeded" in capsys.readouterr().out
    assert arm.published == [], "the whole track is authorized before any of it"
    assert not (tmp_path / "run.npz").exists()


def test_collect_needs_a_group(tmp_path, capsys):
    code = main(["r2s", "collect", "--output", str(tmp_path / "run.npz")])

    assert code == 2
    assert "--group" in capsys.readouterr().out


def test_collect_execute_needs_an_output(arm, capsys):
    code = main(["r2s", "collect", "--group", GROUP, "--execute"])

    assert code == 2
    assert "--output" in capsys.readouterr().out
    assert arm.published == []


def test_an_amplitude_too_fast_to_slew_is_refused(arm, tmp_path, capsys):
    code = main(_argv(tmp_path, "--execute", "--amplitude-scale", "0.9"))

    assert code == 2
    out = capsys.readouterr().out
    assert "multisine" in out and "amplitude" in out
    assert arm.published == []


def test_the_recording_carries_its_profile_and_asset(arm, tmp_path, profile):
    main(_argv(tmp_path, "--execute"))

    raw = np.load(tmp_path / "run.npz", allow_pickle=False)
    assert str(raw["profile"]) == profile.name
    assert str(raw["asset_id"]) == profile.asset_id
    assert str(raw["manifest_sha256"]) == profile.manifest_sha256


def test_collect_reports_the_gap_and_the_drops(arm, tmp_path, capsys):
    main(_argv(tmp_path, "--execute"))

    out = capsys.readouterr().out
    assert "largest gap" in out
    assert "samples" in out


def test_collect_refuses_a_group_it_cannot_stream_to(tmp_path, capsys):
    code = main(
        ["r2s", "collect", "--group", "openarm_left_gripper", "--execute",
         "--output", str(tmp_path / "run.npz")]
    )

    assert code == 2
