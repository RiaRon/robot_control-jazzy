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
    assert openarm["branch"] == "jazzy"
    assert openarm["commit"] == "8087bbc2b37c0b2b2652c0134a9b2b369c57567e"
    assert tesollo["branch"] == "jazzy-dev"
    assert tesollo["commit"] == "3926c2eab8d011046f64874d6252213b2cf18f48"
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


def test_snapshot_verifier_applies_declared_patches(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    (patches / "0001-change.patch").write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npatches:\n  - patches/0001-change.patch\n")

    assert verify_snapshot(metadata, source, snapshot) == []

    (snapshot / "example.txt").write_text("undeclared edit\n")
    assert verify_snapshot(metadata, source, snapshot) == [
        "content mismatch: example.txt"
    ]


def test_snapshot_verifier_rejects_missing_patch(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    source.mkdir()
    snapshot.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("before\n")
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text("excluded: []\npatches:\n  - patches/0001-change.patch\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        "patch failed: patches/0001-change.patch"
    ]


def test_snapshot_verifier_rejects_absolute_patch_path(tmp_path):
    source = tmp_path / "source"
    snapshot = tmp_path / "snapshot"
    patches = tmp_path / "patches"
    source.mkdir()
    snapshot.mkdir()
    patches.mkdir()
    (source / "example.txt").write_text("before\n")
    (snapshot / "example.txt").write_text("after\n")
    patch = patches / "0001-change.patch"
    patch.write_text(
        "--- a/example.txt\n"
        "+++ b/example.txt\n"
        "@@ -1 +1 @@\n"
        "-before\n"
        "+after\n"
    )
    metadata = tmp_path / "UPSTREAM.yaml"
    metadata.write_text(f"excluded: []\npatches:\n  - {patch}\n")

    assert verify_snapshot(metadata, source, snapshot) == [
        f"patch failed: {patch}"
    ]
