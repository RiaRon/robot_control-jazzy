from pathlib import Path

import yaml

from tools.verify_vendor_snapshot import verify_snapshot


ROOT = Path(__file__).parents[1]


def test_metadata_pins_validated_commits():
    openarm = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm/UPSTREAM.yaml").read_text()
    )
    tesollo = yaml.safe_load(
        (ROOT / "vendor_metadata/tesollo/UPSTREAM.yaml").read_text()
    )
    openarm_can = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm_can/UPSTREAM.yaml").read_text()
    )
    description = yaml.safe_load(
        (ROOT / "vendor_metadata/openarm_description/UPSTREAM.yaml").read_text()
    )
    assert openarm["commit"] == "4e837e1d0dae692ff67b560b69d8d281d7a8d4ed"
    assert tesollo["commit"] == "a68335919ee490d5293581574acc7aff12fe969d"
    assert openarm_can["commit"] == "c32ecd31da267967f0c913c2118c843177d88b91"
    assert description["commit"] == "c8696ebfd64ea08ee0a212a9bae21055b6f381bc"
    assert description["source_subpath"] == "src/openarm_description"


def test_snapshot_verifier_detects_changed_file(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "LICENSE").write_text("license")
    (snapshot / "LICENSE").write_text("modified")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: [.git, .github]\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "content mismatch: LICENSE"
    ]


def test_snapshot_verifier_reports_missing_and_unexpected_in_sorted_order(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "b.txt").write_text("b")
    (snapshot / "a.txt").write_text("a")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "missing: b.txt",
        "unexpected: a.txt",
    ]
