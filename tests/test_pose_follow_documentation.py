"""Operator-critical contracts in the pose-follow walkthrough."""

from pathlib import Path
import re


DOCUMENT = Path(__file__).parents[1] / "docs/pose-follow.md"


def _document() -> str:
    return DOCUMENT.read_text()


def test_walkthrough_uses_the_official_root_checkout():
    document = _document()

    assert "~/rl_ws/robot_control/.worktrees" not in document
    assert "cd /home/user/robot_control-jazzy" in document


def test_the_sixty_second_trial_is_bounded():
    document = _document()
    section = re.search(
        r"## 4\. 60초 시험 운전(?P<body>.*?)## 5\. 무기한 운전",
        document,
        re.DOTALL,
    )

    assert section is not None
    assert "--seconds 60" in section.group("body")
    assert "--seconds inf" not in section.group("body")


def test_walkthrough_says_follow_tracks_orientation():
    document = _document()

    assert "축 화살표와 회전 링을 모두 추종" in document
    assert "회전 링은 추종하지" not in document
    assert "--max-tcp-angular-speed" in document


def test_walkthrough_records_layered_diagnostics():
    document = _document()

    for evidence in (
        "--output /tmp/right-follow.json",
        "live marker to measured TCP",
        "marker_update_staleness",
        "accepted_marker_to_ik_target",
        "ik_target_to_command",
        "command_to_measured",
    ):
        assert evidence in document


def test_can_section_verifies_runtime_communication():
    document = _document()

    for evidence in (
        "ip -s -details link show can0",
        "ros2 control list_controllers",
        "ros2 control list_hardware_interfaces",
        "ros2 topic echo --once /joint_states",
    ):
        assert evidence in document
