from __future__ import annotations

from tools.replay_follow_outer import summarize_outer_replay


def _sample(index, command, target, measured, *, phase="translation_hold"):
    return {
        "sample_index": index,
        "timestamp_sec": index * 0.1,
        "stage": f"profile_{phase}",
        "diagnostic_profile": {"phase": phase},
        "joint_positions_rad": {
            "command": [command],
            "ik_target": [target],
            "measured": [measured],
            "next_command": [command],
        },
        "limits": {
            "cartesian_speed": False,
            "cartesian_angular_speed": False,
            "joint": {"velocity": [], "lead": [], "position": []},
        },
    }


def test_structural_replay_compares_old_and_bounded_candidates():
    payload = {
        "schema_version": 2,
        "profile": "fake",
        "joint_names": ["joint"],
        "settings": {"kp_per_sec": 2.0, "command_rate_hz": 10.0},
        "trace": [
            _sample(0, 0.0, 1.0, 0.0, phase="translation_ramp_out"),
            _sample(1, 0.9, 1.0, 0.0),
            _sample(2, 1.0, 1.0, 0.8, phase="origin_hold"),
        ],
    }

    report = summarize_outer_replay(payload)

    assert report["mode"] == (
        "one_step_structural_replay_not_closed_loop_prediction"
    )
    assert report["compared_samples"] == 3
    assert report["total_joint_updates"] == 3
    assert report["unchanged_joint_updates"] == 1
    assert report["outer_clamp_joint_events"] == 2
    assert report["old_strict_crossing_joint_events"] == 1
    assert report["new_strict_crossing_joint_events"] == 0
    assert report["old_max_crossing_rad"] > 0.09
    assert report["new_max_crossing_rad"] == 0.0
    assert report["phase_clamp_joint_events"] == {
        "translation_ramp_out": 0,
        "translation_hold": 1,
        "origin_hold": 1,
    }
    joint = report["per_joint"][0]
    assert joint["strict_crossing_joint_events"] == 1
    assert joint["target_hold_joint_events"] == 1
    assert joint["clamp_joint_events"] == 2
