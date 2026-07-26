"""Three repetitions and the manifest that says which is held out.

`split_repetitions` has always wanted exactly three, and the v2 bundle's `source`
block has always carried `fit_runs` and `holdout_runs`. Nothing produced them, so
`validate --metrics` read a hand-written file and the holdout half of validation
had never run on anything measured.
"""

import json

import numpy as np
import pytest

from robot_control.cli import main
from robot_control.profile import load_builtin_profile

from test_cli_collect_track import RecordingArm, arm, profile  # noqa: F401


GROUP = "openarm_right_arm"


def _argv(tmp_path, *extra):
    return [
        "r2s", "collect", "--group", GROUP,
        "--output", str(tmp_path / "run.npz"),
        *extra,
    ]


def test_collect_writes_three_repetitions_and_a_manifest(arm, tmp_path, capsys):
    code = main(_argv(tmp_path, "--execute", "--repetitions", "3"))

    assert code == 0, capsys.readouterr().out
    manifest = json.loads((tmp_path / "run.json").read_text())
    assert len(manifest["runs"]) == 3
    for entry in manifest["runs"]:
        assert (tmp_path / entry["path"]).exists()
    assert manifest["fit_runs"] == [0, 1]
    assert manifest["holdout_runs"] == [2]


def test_one_repetition_writes_no_manifest(arm, tmp_path):
    """A single run is a recording, not an identification: nothing to hold out."""
    assert main(_argv(tmp_path, "--execute")) == 0

    assert (tmp_path / "run.npz").exists()
    assert not (tmp_path / "run.json").exists()


def test_the_repetitions_are_the_same_track_run_again(arm, tmp_path):
    """A holdout only validates if it was excited the same way."""
    main(_argv(tmp_path, "--execute", "--repetitions", "3"))

    manifest = json.loads((tmp_path / "run.json").read_text())
    commands = [
        np.load(tmp_path / entry["path"], allow_pickle=False)["command"]
        for entry in manifest["runs"]
    ]
    for other in commands[1:]:
        np.testing.assert_allclose(other, commands[0])


def test_the_manifest_carries_its_profile_and_asset(arm, tmp_path, profile):
    main(_argv(tmp_path, "--execute", "--repetitions", "3"))

    manifest = json.loads((tmp_path / "run.json").read_text())
    assert manifest["profile"] == profile.name
    assert manifest["asset"]["id"] == profile.asset_id
    assert manifest["asset"]["manifest_sha256"] == profile.manifest_sha256
    assert manifest["group"] == GROUP


def test_two_repetitions_are_refused(arm, tmp_path, capsys):
    """Two would leave one to fit and one to hold out, which fits nothing."""
    code = main(_argv(tmp_path, "--execute", "--repetitions", "2"))

    assert code == 2
    assert "three" in capsys.readouterr().out


def test_the_arm_returns_to_the_start_between_repetitions(arm, tmp_path):
    """Each run has to start where the last one did, or they are not repetitions."""
    main(_argv(tmp_path, "--execute", "--repetitions", "3"))

    manifest = json.loads((tmp_path / "run.json").read_text())
    firsts = [
        np.load(tmp_path / entry["path"], allow_pickle=False)["command"][0]
        for entry in manifest["runs"]
    ]
    for other in firsts[1:]:
        np.testing.assert_allclose(other, firsts[0])


def test_a_run_that_fails_partway_leaves_the_manifest_unwritten(
    arm, tmp_path, monkeypatch, capsys
):
    """A manifest naming a file that was never written is worse than none."""
    calls = {"n": 0}
    original = RecordingArm.stop_recording

    def fail_on_second(self):
        calls["n"] += 1
        if calls["n"] == 2:
            raise OSError("the controller dropped out")
        return original(self)

    monkeypatch.setattr(RecordingArm, "stop_recording", fail_on_second)

    code = main(_argv(tmp_path, "--execute", "--repetitions", "3"))

    assert code == 2
    assert not (tmp_path / "run.json").exists()
