from robot_control.cli import main


def test_preflight_reports_profile(capsys):
    assert main(["r2s", "preflight", "--profile", "openarm_tesollo"]) == 0
    output = capsys.readouterr().out
    assert "openarm_tesollo" in output
    assert "publish_enabled: false" in output


def test_collect_defaults_to_dry_run(capsys):
    assert main(["r2s", "collect", "--profile", "openarm_tesollo"]) == 0
    assert "DRY RUN" in capsys.readouterr().out
