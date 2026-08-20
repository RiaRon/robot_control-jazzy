"""Repository-level contracts for the optional MATLAB pose-follow analyzer."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
MATLAB = ROOT / "matlab" / "pose_follow"
DOCUMENT = ROOT / "docs" / "matlab-pose-follow-analysis.md"


def test_parser_uses_matlab_jsondecode_fileread_and_supports_both_variants():
    parser = (MATLAB / "read_pose_follow_json.m").read_text()

    assert "jsondecode(fileread(filePath))" in parser
    assert '"legacy-2026-08-18"' in parser
    assert '"extended"' in parser
    assert "reconstructProjections" in parser
    assert "canonicalPhase" in parser


def test_analyzer_declares_the_complete_bundle_without_robot_dependencies():
    analyzer = (MATLAB / "analyze_pose_follow.m").read_text()
    required = {
        "summary.csv",
        "analysis_summary.json",
        "analysis.mat",
        "tcp_error_timeseries.png",
        "error_layers.png",
        "joint_tracking.png",
        "j1_j4_j7_detail.png",
        "ik_events.png",
        "phase_comparison.png",
        "research_report.pdf",
    }

    assert all(name in analyzer for name in required)
    assert "import robot_control" not in analyzer.lower()
    assert "system('ros2" not in analyzer.lower()


def test_documentation_names_versions_phases_and_git_data_policy():
    document = DOCUMENT.read_text()

    for evidence in (
        "MATLAB R2021b",
        "추가 Toolbox 없음",
        "ramp",
        "hold",
        "return",
        "origin-hold",
        "원시 pose-follow JSON",
        "output 폴더",
        "ChatGPT Work",
    ):
        assert evidence in document


def test_generated_data_is_ignored_locally():
    ignore = (MATLAB / ".gitignore").read_text()

    for pattern in ("*.json", "*.csv", "*.mat", "*.png", "*.pdf", "output/"):
        assert pattern in ignore
